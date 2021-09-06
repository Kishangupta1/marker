SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

all: $(OBJECTS) createlib clean

INCPATH = -I $(MARKER)/include
LIBPATH =
CFLAGS = -Wall -c -std=c++11

%.o: %.cpp
	$(CPPC) $(INCPATH) $(LIBPATH) $(CFLAGS) $<

createlib:
	ar ruvs $(MARKER)/lib/libmkr.a *.o

clean:
	rm *.o
