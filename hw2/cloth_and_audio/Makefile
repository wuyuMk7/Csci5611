STD=-std=c++17
INCLUDES=-Iincludes
LIBS=-lglfw -lsdl2
CFLAGS=-O3
TESTFLAGS=__TEST__

all: hw2

glad: glad.c
	$(CC) -o glad.o -c glad.c

camera: camera.cc
	$(CXX) -o camera.o -c camera.cc $(INCLUDES) $(STD)

hw2: hw2.cc audio.cc glad camera
	$(CXX) $(CFLAGS) -o hw2 hw2.cc camera.o glad.o $(INCLUDES) $(LIBS) $(STD)
	$(CXX) $(CFLAGS) -o audio audio.cc camera.o glad.o $(INCLUDES) $(LIBS) $(STD)

test_camera: camera.cc glad
	$(CXX) -D$(TESTFLAGS) -o test_camera camera.cc glad.o $(INCLUDES) $(LIBS) $(STD)

clean:
	rm -rf test audio hw2 *.o
