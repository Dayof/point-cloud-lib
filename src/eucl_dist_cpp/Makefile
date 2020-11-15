CC         = g++
CFLAGS     = -c -Wall -std=c++14

SOURCES    = kdtree.cpp
OBJECTS    = $(SOURCES:.cpp=.o)
HEADERS    = nanoflann.hpp utils_mod.h
EXECUTABLE = kdtree
# pkg-config --cflags --libs python3
EXTRAS	   = -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu/python2.7 -I/usr/local/lib/python2.7/dist-packages/numpy/core/include -lpython2.7

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) $(HEADERS)
	$(CC) $(OBJECTS) -o $@ $(EXTRAS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@ $(EXTRAS)

clean:
	rm $(OBJECTS) $(EXECUTABLE)