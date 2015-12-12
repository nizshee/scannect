
CPP_FLAGS=-Werror -Wall
GLUT_FLAGS=-lglut -lGLU -lGL

FREENECT_HEADER=/home/roman/ws/au/kinect/lib/libfreenect/include
FREENECT_LIBS=/home/roman/ws/au/kinect/lib/libfreenect/build/lib 

PYTHON_HEADERS=/usr/include/python3.4/

BIN_DIR=../../bin
OUT_DIR=../

all: ../_KinectWrapper.so

../../bin/kinect.o: kinect.cpp kinect.h
	g++ -O2 -fPIC -c kinect.cpp -o ../../bin/kinect.o -I$(FREENECT_HEADER)

kinect_wrap.cxx: kinect.i kinect.h
	swig -c++ -python -outdir .. kinect.i

../../bin/kinect_wrap.o: kinect_wrap.cxx kinect.h
	g++ -O2 -fPIC -c kinect_wrap.cxx -o ../../bin/kinect_wrap.o -I${PYTHON_HEADERS} -I${FREENECT_HEADER}

../_KinectWrapper.so: ../../bin/kinect_wrap.o ../../bin/kinect.o
	g++ -L$(FREENECT_LIBS) -shared ../../bin/kinect_wrap.o ../../bin/kinect.o -o ../_KinectWrapper.so -lfreenect

clean:
	rm ../../bin/kinect.o kinect_wrap.cxx ../../bin/kinect_wrap.o ../_KinectWrapper.so ../KinectWrapper.py

.PHONY: all clean


# bin/main: bin/kinect.o
# 	g++ -L$(FREENECT_LIBS) -g3 -std=c++11 bin/kinect.o -lfreenect -o bin/main

# bin/kinect.o: kinect.cpp
# 	g++ -I$(FREENECT_HEADER) -g3 -std=c++11 -c kinect.cpp -o bin/kinect.o
