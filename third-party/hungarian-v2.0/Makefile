
CXX = g++
CFLAGS = -O3 -Wall #-fpermissive -Wl,-b64

DIRS = . plot

PLOTDIR = plot
PLOTOBJS = $(PLOTDIR)/Cgnuplot.o
HGRN_OBJS = Hungarian.o Assignment.o BipartiteGraph.o PlotGraph.o

OBJS = $(HGRN_OBJS) $(PLOTOBJS) main.o 

EXE = hungarian
all: $(EXE)
$(EXE): $(OBJS)
	$(CXX) $(CFLAGS) -o $(EXE) $(OBJS)
	@echo Done.

%.o: %.cpp %.h
	$(CXX) $(CFLAGS) -c $<

$(PLOTDIR)/Cgnuplot.o:
#	@for $(i) in $(DIRS); do \
#        echo "make all in $(i)..."; 
#	(cd $(i); $(MAKE) $(MFLAGS)); done
	(cd $(PLOTDIR); $(MAKE) $(MFLAGS))
	@echo Done making Cgnuplot.

 
main.o: main.cpp Matrix.h CmdParser.h 
	$(CXX) $(CFLAGS) -c $<

clean:
	rm -f $(EXE) *.o *~ *.swp
	(cd $(PLOTDIR); $(MAKE) clean )


