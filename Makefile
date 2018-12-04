CXX?=g++
CC=$(CXX)
BOOST?=/usr/local/gcc-7.2.0/boost
CXXFLAGS?=-g -std=c++17 -Wall -Wextra -I $(BOOST)

steiner: steiner.o tinyxml2.o 

steiner.o: steiner.cpp grid_graph.h

clean:
	rm -f *.o *~ steiner
