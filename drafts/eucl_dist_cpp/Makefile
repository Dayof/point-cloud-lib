CC         = g++
CFLAGS     = -c -Wall

SOURCES    = kdtree.cpp
OBJECTS    = $(SOURCES:.cpp=.o)
HEADERS    = nanoflann.hpp utils_mod.h

# Name of executable target:
EXECUTABLE = kdtree

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) $(HEADERS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)