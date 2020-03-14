STD=-std=c++17
INCLUDES=-Iincludes
LIBS=-lglfw -lsdl2
CFLAGS=-O3

all: hw3

glad: glad.c
	$(CC) -o glad.o -c glad.c

camera: camera.cc
	$(CXX) -o camera.o -c camera.cc $(INCLUDES) $(STD)

hw3: hw3_checkin.cc glad camera
	$(CXX) $(CFLAGS) -o hw3_checkin hw3_checkin.cc camera.o glad.o $(INCLUDES) $(LIBS) $(STD)

clean:
	rm -rf hw3 *.o
