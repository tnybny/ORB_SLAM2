/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */



#include "System.h"
#include "Converter.h"
#include "Optimizer.h"
#include "Map.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

std::string formatInt(long num, int size) {
  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(size) << num;
  return oss.str();
};

namespace ORB_SLAM2
{

	System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
			const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
	mbDeactivateLocalizationMode(false)
	{
		// Output welcome message
		cout << endl <<
			"ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
			"This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
			"This is free software, and you are welcome to redistribute it" << endl <<
			"under certain conditions. See LICENSE.txt." << endl << endl;

		cout << "Input sensor was set to: ";

		if(mSensor==MONOCULAR)
			cout << "Monocular" << endl;
		else if(mSensor==STEREO)
			cout << "Stereo" << endl;
		else if(mSensor==RGBD)
			cout << "RGB-D" << endl;

		//Check settings file
		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if(!fsSettings.isOpened())
		{
			cerr << "Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}


		//Load ORB Vocabulary
		cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

		mpVocabulary = new ORBVocabulary();
		bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
		if(!bVocLoad)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Falied to open at: " << strVocFile << endl;
			exit(-1);
		}
		cout << "Vocabulary loaded!" << endl << endl;

		//Create KeyFrame Database
		mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

		//Create the Map
		mpMap = new Map();

		//Create Drawers. These are used by the Viewer
		mpFrameDrawer = new FrameDrawer(mpMap);
		mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

		//Initialize the Tracking thread
		//(it will live in the main thread of execution, the one that called this constructor)
		mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
				mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

		//Initialize the Local Mapping thread and launch
		mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
		mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

		//Initialize the Loop Closing thread and launch
		mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
		mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

		//Initialize the Viewer thread and launch
		if(bUseViewer)
		{
			mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
			mptViewer = new thread(&Viewer::Run, mpViewer);
			mpTracker->SetViewer(mpViewer);
		}

		//Set pointers between threads
		mpTracker->SetLocalMapper(mpLocalMapper);
		mpTracker->SetLoopClosing(mpLoopCloser);

		mpLocalMapper->SetTracker(mpTracker);
		mpLocalMapper->SetLoopCloser(mpLoopCloser);

		mpLoopCloser->SetTracker(mpTracker);
		mpLoopCloser->SetLocalMapper(mpLocalMapper);
	}

	cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
	{
		if(mSensor!=STEREO)
		{
			cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
			exit(-1);
		}   

		// Check mode change
		{
			unique_lock<mutex> lock(mMutexMode);
			if(mbActivateLocalizationMode)
			{
				mpLocalMapper->RequestStop();

				// Wait until Local Mapping has effectively stopped
				while(!mpLocalMapper->isStopped())
				{
					usleep(1000);
				}

				mpTracker->InformOnlyTracking(true);
				mbActivateLocalizationMode = false;
			}
			if(mbDeactivateLocalizationMode)
			{
				mpTracker->InformOnlyTracking(false);
				mpLocalMapper->Release();
				mbDeactivateLocalizationMode = false;
			}
		}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if(mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

		unique_lock<mutex> lock2(mMutexState);
		mTrackingState = mpTracker->mState;
		mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
		mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
		return Tcw;
	}

	cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
	{
		if(mSensor!=RGBD)
		{
			cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
			exit(-1);
		}    

		// Check mode change
		{
			unique_lock<mutex> lock(mMutexMode);
			if(mbActivateLocalizationMode)
			{
				mpLocalMapper->RequestStop();

				// Wait until Local Mapping has effectively stopped
				while(!mpLocalMapper->isStopped())
				{
					usleep(1000);
				}

				mpTracker->InformOnlyTracking(true);
				mbActivateLocalizationMode = false;
			}
			if(mbDeactivateLocalizationMode)
			{
				mpTracker->InformOnlyTracking(false);
				mpLocalMapper->Release();
				mbDeactivateLocalizationMode = false;
			}
		}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if(mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

		unique_lock<mutex> lock2(mMutexState);
		mTrackingState = mpTracker->mState;
		mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
		mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
		return Tcw;
	}

	cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
	{
		if(mSensor!=MONOCULAR)
		{
			cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
			exit(-1);
		}

		// Check mode change
		{
			unique_lock<mutex> lock(mMutexMode);
			if(mbActivateLocalizationMode)
			{
				mpLocalMapper->RequestStop();

				// Wait until Local Mapping has effectively stopped
				while(!mpLocalMapper->isStopped())
				{
					usleep(1000);
				}

				mpTracker->InformOnlyTracking(true);
				mbActivateLocalizationMode = false;
			}
			if(mbDeactivateLocalizationMode)
			{
				mpTracker->InformOnlyTracking(false);
				mpLocalMapper->Release();
				mbDeactivateLocalizationMode = false;
			}
		}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if(mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

		unique_lock<mutex> lock2(mMutexState);
		mTrackingState = mpTracker->mState;
		mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
		mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

		return Tcw;
	}

	cv::Mat System::TrackMonocular(const cv::Mat &im, const cv::Mat &im_ir, const double &timestamp)
	{
		if(mSensor!=MONOCULAR)
		{
			cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
			exit(-1);
		}

		// Check mode change
		{
			unique_lock<mutex> lock(mMutexMode);
			if(mbActivateLocalizationMode)
			{
				mpLocalMapper->RequestStop();

				// Wait until Local Mapping has effectively stopped
				while(!mpLocalMapper->isStopped())
				{
					usleep(1000);
				}

				mpTracker->InformOnlyTracking(true);
				mbActivateLocalizationMode = false;
			}
			if(mbDeactivateLocalizationMode)
			{
				mpTracker->InformOnlyTracking(false);
				mpLocalMapper->Release();
				mbDeactivateLocalizationMode = false;
			}
		}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if(mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		cv::Mat Tcw = mpTracker->GrabImageMonocular(im,im_ir,timestamp);

		unique_lock<mutex> lock2(mMutexState);
		mTrackingState = mpTracker->mState;
		mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
		mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

		return Tcw;
	}
	
	void System::ActivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbActivateLocalizationMode = true;
	}

	void System::DeactivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbDeactivateLocalizationMode = true;
	}

	bool System::MapChanged()
	{
		static int n=0;
		int curn = mpMap->GetLastBigChangeIdx();
		if(n<curn)
		{
			n=curn;
			return true;
		}
		else
			return false;
	}

	void System::Reset()
	{
		unique_lock<mutex> lock(mMutexReset);
		mbReset = true;
	}

	void System::Shutdown()
	{
		mpLocalMapper->RequestFinish();
		mpLoopCloser->RequestFinish();
		if(mpViewer)
		{
			mpViewer->RequestFinish();
			while(!mpViewer->isFinished())
				usleep(5000);
		}

		// Wait until all thread have effectively stopped
		while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
		{
			usleep(5000);
		}

		if(mpViewer)
			pangolin::BindToContext("ORB-SLAM2: Map Viewer");
	}

	void System::SaveTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if(mSensor==MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();
		for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
				lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
		{
			if(*lbL)
				continue;

			KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			while(pKF->isBad())
			{
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

			vector<float> q = Converter::toQuaternion(Rwc);

			f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}


	void System::SaveKeyFrameTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		//cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		for(size_t i=0; i<vpKFs.size(); i++)
		{
			KeyFrame* pKF = vpKFs[i];

			// pKF->SetPose(pKF->GetPose()*Two);

			if(pKF->isBad())
				continue;

			cv::Mat R = pKF->GetRotation().t();
			vector<float> q = Converter::toQuaternion(R);
			cv::Mat t = pKF->GetCameraCenter();
			f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

		}

		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	void System::SaveNVM(const string &strSettingsFile, const string &filename)
	{
		//Check settings file
		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if(!fsSettings.isOpened())
		{
			cerr << "Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}
		
		// modified from https://github.com/mojovski/ORB_SLAM/blob/export_nvm_2/src/main.cc
		//--------------
		//  Export the Poses and the Features to a NVM file
		//--------------

		// adjust the scene first via global BA
		cout << "Starting GlobalBA! This can take some minutes. \n";
		ORB_SLAM2::Optimizer::GlobalBundleAdjustemnt(mpMap,20); 

		cout << endl << "Saving NVM to " << filename << endl;
		// See http://ccwu.me/vsfm/doc.html#nvm for details about the file structure

		ofstream f;
		f.open(filename.c_str());
		f << "NVM_V3  <placeholder> \n\n";

		// Now: the model: 
		// <Number of cameras>   <List of cameras>
		// <Number of 3D points> <List of points>
		/*
		   <Camera> = <Image File name> <focal length> <quaternion WXYZ> <camera center> <radial distortion> 0
		   <Point>  = <XYZ> <RGB> <number of measurements> <List of Measurements>
		   with:
		   <Measurement> = <Image index> <Feature Index> <xy>
		*/

		// 1.------ Export the cameras
		// 1.1 count the amount of key frames
		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
		
		int count_good_KF=0;
		for(size_t i=0; i<vpKFs.size(); i++)
		{
			ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

			if(pKF->isBad())
				continue;
			count_good_KF+=1;
		}
		f << count_good_KF << "\n"; //now, the list of cameras follows

		// 1.2 export the camera parameters itself 
		// indexing of key frames by its consecutive number
		std::map<int,int> kf_index;
		int inc_frame_counter=-1;
		for(size_t i=0; i<vpKFs.size(); i++)
		{
			ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

			if(pKF->isBad())
				continue;
			inc_frame_counter+=1;

			cv::Mat R = pKF->GetRotation();//.t();
			vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
			cv::Mat t = pKF->GetCameraCenter();
			kf_index[pKF->mnFrameId]=inc_frame_counter;
			f << "image_"<<  formatInt(pKF->mnFrameId + 1, 4) << ".png\t" << (double)fsSettings["Camera.fx"] << " " << 
				q[3] << " " <<  q[0] << " " << q[1] << " " << q[2] << " " << //WXYZ
				t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << 
				(double)fsSettings["Camera.k1"] << " " << 0 << "\n";
		}

		// 2.------ Export the 3D feature observations
		// <Number of 3D points> <List of points>
		// <Point>  = <XYZ> <RGB> <number of measurements> <List of Measurements>
		// <Measurement> = <Image index> <Feature Index> <xy>
		std::vector<MapPoint*> all_points=mpMap->GetAllMapPoints();
		int count_good_map_points=0;
		for(size_t i=0, iend=all_points.size(); i<iend;i++)
		{
			if (!(all_points[i]->isBad()))
				count_good_map_points+=1;
		}

		f << count_good_map_points << "\n";
		for(size_t i=0, iend=all_points.size(); i<iend;i++)
		{
			if (all_points[i]->isBad())
				continue;

			MapPoint* pMP = all_points[i];
			cv::Mat pos=pMP->GetWorldPos();
			f << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << " " <<
			//rgb
			pMP->grayscaleValue << " 0 0 ";
			//now all the observations/measurements
			std::map<KeyFrame*,size_t> observations=pMP->GetObservations();
			//count good observations:
			int num_good_observations=0;
			for (std::map<KeyFrame*,size_t>::iterator ob_it=observations.begin(); ob_it!=observations.end(); ob_it++)
			{
				if (!(*ob_it).first->isBad())
					num_good_observations+=1;
			}

			f << num_good_observations << " ";
			for (std::map<KeyFrame*,size_t>::iterator ob_it=observations.begin(); ob_it!=observations.end(); ob_it++)
			{
				//skip if the key frame is "bad"
				if ((*ob_it).first->isBad())
					continue;

				//<Measurement> = <Image index> <Feature Index> <xy>
				std::vector<cv::KeyPoint> key_points=(*ob_it).first->GetKeyPoints();
				f << kf_index[(*ob_it).first->mnFrameId] << " " << (*ob_it).second << " " << 
					key_points[ob_it->second].pt.x << " " <<
					key_points[ob_it->second].pt.y << " ";
			}
			f << "\n";

		}
		f.close();

	}

	void System::SaveTrajectoryKITTI(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if(mSensor==MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
		{
			ORB_SLAM2::KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

			while(pKF->isBad())
			{
				//  cout << "bad parent" << endl;
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

			f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
				Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
				Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	int System::GetTrackingState()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackingState;
	}

	vector<MapPoint*> System::GetTrackedMapPoints()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedMapPoints;
	}

	vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedKeyPointsUn;
	}

} //namespace ORB_SLAM
