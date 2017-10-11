OBJS = main.o mapper.o randGen.o
CC = g++
CXXFLAGS =-Wall -std=c++11
LFLAGS = -Wall -std=c++11
HDIR = -I../include

psim : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o psim

obj/main.o : mapper.h randGen.h
	$(CC) $(CFLAGS) main.cpp

obj/mapper.o : mapper.h map_s.h randGen.h
	$(CC) $(CFLAGS) mapper.cpp

obj/randGen.o : randGen.h
	$(CC) $(CFLAGS) randGen.cpp

.PHONY: clean

clean:
	rm -f obj/*.o