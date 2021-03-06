﻿CXX = "C:/murIDE/devkit-x86/mingw530_32/bin/g++.exe"
CXXFLAGS = -std=c++11 -Wall -O2

INCLUDES = -I. "-IC:/murIDE/devkit-x86/murlibs"
LIBS = "-LC:/murIDE/devkit-x86/murlibs" -lopencv_core2410 -lopencv_highgui2410 -lopencv_imgproc2410 -lopencv_flann2410 -lopencv_features2d2410 -lopencv_objdetect2410 -lopencv_videostab2410 -lopencv_video2410 -lzmq

cxxs = $(wildcard *.cpp)
objs = $(patsubst %.cpp,%.o,$(cxxs))
ds = $(wildcard *.d)

all: NTI3

include $(ds)

NTI3: $(objs)

	$(CXX) $(CXXFLAGS) $^ $(INCLUDES) $(LIBS) -o NTI3.exe

 %.o: %.cpp

	$(CXX) $(CXXFLAGS) -c $< $(INCLUDES) -o $@

clean:
	rm -r *.d
	rm -r *.o
	rm NTI3
