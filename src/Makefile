
CPP_FLAGS=-Werror -Wall

FREENECT_HEADER=/home/roman/ws/au/kinect/libfreenect/include
FREENECT_LIBS=/home/roman/ws/au/kinect/libfreenect/build/lib 

PCL_H=/usr/include/pcl-1.7
EIGEN_H=/usr/include/eigen3
VTK_H=/usr/include/vtk-5.8

PYTHON_HEADERS=/usr/include/python3.4/


all: PointCloud.o PointCloud_wrap.cxx PointCloud_wrap.o _PointCloudWrapper.so

PointCloud.o: PointCloud.cpp PointCloud.h
	g++ -O2 -fPIC -c PointCloud.cpp -I${PCL_H} -I${EIGEN_H}

PointCloud_wrap.cxx: PointCloud.h PointCloud.i
	swig -c++ -python PointCloud.i

PointCloud_wrap.o: PointCloud_wrap.cxx PointCloud.h
	g++ -O2 -fPIC -c PointCloud_wrap.cxx -I${PYTHON_HEADERS} -I${PCL_H} -I${EIGEN_H}

_PointCloudWrapper.so: PointCloud_wrap.o PointCloud.o
	g++ -shared PointCloud_wrap.o PointCloud.o -o _PointCloudWrapper.so \
	-lboost_system -lpcl_common -lpcl_filters -lpcl_io -lpcl_registration


clean:
	rm *.o PointCloud_wrap.cxx PointCloudWrapper.py _PointCloudWrapper.so

tmp: tmp.o
	g++ tmp.o -o tmp -lboost_system -lpcl_common -lpcl_filters

tmp.o: tmp.cpp
	g++ -c tmp.cpp -I${PCL_H} -I${EIGEN_H}	


.PHONY: all clean

