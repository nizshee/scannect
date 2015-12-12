
CPP_FLAGS=-Werror -Wall

PCL_H=/usr/include/pcl-1.7
EIGEN_H=/usr/include/eigen3
VTK_H=/usr/include/vtk-5.8

PYTHON_HEADERS=/usr/include/python3.4/


all: ../../bin/PointCloud.o PointCloud_wrap.cxx ../../bin/PointCloud_wrap.o ../_PointCloudWrapper.so

../../bin/PointCloud.o: PointCloud.cpp PointCloud.h
	g++ -O2 -fPIC -c PointCloud.cpp -o ../../bin/PointCloud.o -I${PCL_H} -I${EIGEN_H}

PointCloud_wrap.cxx: PointCloud.h PointCloud.i
	swig -c++ -python -outdir .. PointCloud.i

../../bin/PointCloud_wrap.o: PointCloud_wrap.cxx PointCloud.h
	g++ -O2 -fPIC -c PointCloud_wrap.cxx -o ../../bin/PointCloud_wrap.o -I${PYTHON_HEADERS} -I${PCL_H} -I${EIGEN_H}

../_PointCloudWrapper.so: ../../bin/PointCloud_wrap.o ../../bin/PointCloud.o
	g++ -shared ../../bin/PointCloud_wrap.o ../../bin/PointCloud.o -o ../_PointCloudWrapper.so \
	-lboost_system -lpcl_common -lpcl_filters -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_surface


clean:
	rm PointCloud_wrap.cxx ../PointCloudWrapper.py ../../bin/PointCloud.o ../../bin/PointCloud_wrap.o ../_PointCloudWrapper.so

# main: main.o
# 	g++ main.o -o main -lboost_system -lpcl_common -lpcl_filters

# main.o: main.cpp
# 	g++ -c main.cpp -I${PCL_H} -I${EIGEN_H}	


.PHONY: all clean

