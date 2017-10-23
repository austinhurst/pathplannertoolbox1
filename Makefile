CC = g++
HDIR = ./include
SRCDIR = ./src
OBJDIR = ./src/obj
CXXFLAGS = -Wall -std=c++11 -I$(HDIR) -c
LFLAGS = -Wall -std=c++11
HEADMAIN = $(HDIR)/mapper.h $(HDIR)/randGen.h $(HDIR)/fileReader.h $(HDIR)/simpleRRT.h
OBJS = $(OBJDIR)/main.o $(OBJDIR)/mapper.o $(OBJDIR)/randGen.o $(OBJDIR)/fileReader.o $(OBJDIR)/pathPlanner.o \
$(OBJDIR)/simpleRRT.o

psim : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o psim

$(OBJDIR)/main.o : $(SRCDIR)/main.cpp $(HEADMAIN)
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/main.o $(SRCDIR)/main.cpp

$(OBJDIR)/mapper.o : $(SRCDIR)/mapper.cpp $(HDIR)/mapper.h $(HDIR)/map_s.h $(HDIR)/randGen.h $(HDIR)/fileReader.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/mapper.o $(SRCDIR)/mapper.cpp

$(OBJDIR)/randGen.o : $(SRCDIR)/randGen.cpp $(HDIR)/randGen.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/randGen.o $(SRCDIR)/randGen.cpp

$(OBJDIR)/fileReader.o : $(SRCDIR)/fileReader.cpp $(HDIR)/fileReader.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/fileReader.o $(SRCDIR)/fileReader.cpp

$(OBJDIR)/pathPlanner.o : $(SRCDIR)/pathPlanner.cpp $(HDIR)/pathPlanner.h $(HDIR)/map_s.h $(HDIR)/randGen.h $(HDIR)/fileReader.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/pathPlanner.o $(SRCDIR)/pathPlanner.cpp

$(OBJDIR)/simpleRRT.o : $(SRCDIR)/simpleRRT.cpp $(HDIR)/simpleRRT.h $(HDIR)/pathPlanner.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/simpleRRT.o $(SRCDIR)/simpleRRT.cpp

.PHONY: clean
clean:
	rm -f $(OBJDIR)/*.o