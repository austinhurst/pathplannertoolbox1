CC = g++
HDIR = ./include
SRCDIR = ./src
OBJDIR = ./src/obj
CXXFLAGS = -Wall -std=c++11 -I$(HDIR) -c
LFLAGS = -Wall -std=c++11
OBJS = $(OBJDIR)/main.o $(OBJDIR)/mapper.o $(OBJDIR)/randGen.o


psim : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o psim

$(OBJDIR)/main.o : $(HDIR)/mapper.h $(HDIR)/randGen.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/main.o $(SRCDIR)/main.cpp

$(OBJDIR)/mapper.o : $(HDIR)/mapper.h $(HDIR)/map_s.h $(HDIR)/randGen.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/mapper.o $(SRCDIR)/mapper.cpp

$(OBJDIR)/randGen.o : $(HDIR)/randGen.h
	$(CC) $(CXXFLAGS) -o$(OBJDIR)/randGen.o $(SRCDIR)/randGen.cpp

.PHONY: clean

clean:
	rm -f $(OBJDIR)/*.o