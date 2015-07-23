PROJECT=program #executable file name
CC=g++
CFLAGS= -Wall -std=c++11
DEBUGFLAGS= -g
RELEASEFLAGS= -O3
LIBS= -lSDL2 -lSDL2_image

#directories:
INCLUDE=include
OBJDIR=obj

#needed source files
SRC=$(wildcard src/*.cpp)


OBJSR=$(patsubst src/%.cpp,obj/%.o,$(SRC))

all: $(PROJECT) 
$(PROJECT): $(OBJSR) 
	$(CC) $(CFLAGS) $(RELEASEFLAGS) -I $(INCLUDE) -o $(PROJECT) $^ $(LIBS)

$(OBJSR): obj/%.o : src/%.cpp
	$(CC) $(CFLAGS) $(RELEASEFLAGS) -I $(INCLUDE) -c $< $(LIBS) -o $@

run: $(PROJECT)
	./$(PROJECT)

clean:
	rm -f obj/*.o $(PROJECT)
