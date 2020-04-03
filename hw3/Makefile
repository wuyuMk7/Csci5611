STD=-std=c++17
INCLUDES=-Iincludes
LIBS=-lglfw -lsdl2
CFLAGS=-O3

all: hw3

glad: glad.c
	$(CC) -o glad.o -c glad.c

camera: camera.cc
	$(CXX) -o camera.o -c camera.cc $(INCLUDES) $(STD)

agent: agent.cc
	$(CXX) -o agent.o -c agent.cc $(INCLUDES) $(STD)

crowd: crowd.cc
	$(CXX) -o crowd.o -c crowd.cc $(INCLUDES) $(STD)

common: common.cc
	$(CXX) -o common.o -c common.cc $(INCLUDES) $(STD)

hw3: hw3_checkin.cc hw3_boid.cc hw3_main.cc glad camera agent crowd common
	$(CXX) $(CFLAGS) -o hw3_checkin hw3_checkin.cc camera.o glad.o $(INCLUDES) $(LIBS) $(STD)
	$(CXX) $(CFLAGS) -o hw3 hw3_main.cc agent.o common.o crowd.o camera.o glad.o $(INCLUDES) $(LIBS) $(STD)

clean:
	rm -rf hw3 hw3_checkin *.o
