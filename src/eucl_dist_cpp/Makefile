CC          = g++
CFLAGS      = -g -Wall -pedantic -std=c++14

OBJ_DIR     = bin
SRC_DIR     = core
LIB_DIR     = lib

SOURCES     = pc_utils.cpp eucl_search.cpp
HEADERS     = nanoflann.hpp pc_utils.hpp
OBJECTS     = $(SOURCES:.cpp=.o)

SOURCES_D  := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS_D  := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES_D))
HEADERS_D  := $(wildcard $(SRC_DIR)/*.hpp)

EXEC 	    = bin/pc_main
#CFLAGS	  += $(pkg-config --cflags --libs python2)
HEADERS_L	= -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu/python2.7 -lpython2.7	
HEADERS_L  += -Ilib

all: mkdirs $(OBJECTS_D) $(EXEC)

mkdirs:
	mkdir -p $(OBJ_DIR)

$(EXEC): $(OBJECTS_D)
			$(CC) $(CFLAGS) $(HEADERS_L) -o $@ $^ 

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) $(HEADERS_L) -c -o $@ $<

clean:
	rm -f *.o *~ $(EXEC); rm -rf $(OBJ_DIR)
