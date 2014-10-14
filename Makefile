CC=gcc
CXX=g++
LODE=Dropbox/4_FYP/starting/ode-0.13
CFLAGS=-c -Wall -O2 -D__OSX__ -I/$(HOME)/$(LODE)/include 
LDFLAGS= 
LIBS= -L/usr/local -L/usr/lib \
	-L/usr/X11R6/lib -lX11 -lXext -lm \
	-lode -framework OpenGL -framework Carbon -framework AGL -framework GLUT   \
	-Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib 
			
SOURCES= renderer.cpp simulator.cpp toybody.cpp toycontrol.cpp main.cpp drawstuff.cpp osx.cpp 

OBJECTS=$(SOURCES:.cpp=.o) 
EXECUTABLE=a3


all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@ $(LIBS)

.cpp.o:
	$(CXX) $(CFLAGS) $< -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean: 
	-rm *.o
	-rm $(EXECUTABLE) 

