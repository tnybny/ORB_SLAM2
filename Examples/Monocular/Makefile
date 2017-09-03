CXX = /usr/bin/c++

LIBS = -L/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/ORB_SLAM2/lib -L/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/Pangolin/build/src 

LDFLAGS = -rdynamic -lORB_SLAM2 -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lpangolin -lrt -lpthread -lGLU -lGL -lSM -lICE -lX11 -lXext -lGLEW -lSM -lICE -lX11 -lXext -lGLEW -lpython2.7 -lpng -lz -ljpeg ../../Thirdparty/DBoW2/lib/libDBoW2.so ../../Thirdparty/g2o/lib/libg2o.so -Wl,-rpath,/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/ORB_SLAM2/lib:/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/Pangolin/build/src:/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/ORB_SLAM2/Thirdparty/DBoW2/lib:/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/ORB_SLAM2/Thirdparty/g2o/lib

CXX_FLAGS =  -Wall -O3 -march=native -std=c++11 -O3 -DNDEBUG -I/usr/include/opencv -I/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/ORB_SLAM2 -I/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/ORB_SLAM2/include -I/usr/local/include/eigen3 -I/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/Pangolin/include -I/afs/unity.ncsu.edu/users/b/bramach2/Documents/Thermal_sensing/Pangolin/build/src/include

CXX_DEFINES = -DCOMPILEDWITHC11

all :
	${CXX} ${LIBS} ${CXX_FLAGS} ${CXX_DEFINES} -o rgb_ir_duo_5 rgb_ir_duo.cc ${LDFLAGS}

.PHONY : clean
clean :
	rm rgb_ir_duo_5
