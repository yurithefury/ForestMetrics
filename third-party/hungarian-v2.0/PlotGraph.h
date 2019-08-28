
///////////////////////////////////////////////////////////////////////////////
// File name: PlotGraph.h
// Description: This file defines the methods of plotting the Hungarian steps.
// You may need to install gnuplot first.
// A plotting lib is included in ./libCgnuplot (Author: N. Devillard) 
// The plotting funtion is stripped from Dr. Dylan Shell's code 
// (see ./libCgnuplot/graph_hungarian.c: void draw_graph(int new_member_of_T) )
// Lantao Liu, Nov 3, 2009
///////////////////////////////////////////////////////////////////////////////

#ifndef PLOT_GRAPH_H
#define PLOT_GRAPH_H

#include "Matrix.h"
#include "BipartiteGraph.h"
#include "plot/Cgnuplot.h"

//#define PERIOD 4  //uniformly defined in Matrix.h

extern int _Period;

class PlotGraph{
public:
  PlotGraph():g(NULL),p(NULL){ period = _Period != 0 ? _Period : PERIOD; }
  ~PlotGraph(){}

  void SetPeriod(unsigned int _period){ period = _period; }
  unsigned int GetPeriod(void){ return period; }
  
  //gnuplot_ctrl*  InitPlot(void){ return gnuplot_init(); }
  void InitPlot(void){ g = gnuplot_init(); }

  void ClosePlot(gnuplot_ctrl* _handle){ gnuplot_close(_handle); }
  void ClosePlot(void){ ClosePlot(g); }

  void PlotBipartiteGraph(BipartiteGraph& _bg,
		     	  vector<VID>& _S, 
		     	  vector<VID>& _T,
		     	  vector<VID>& _N,
		     	  vector<EID>& _EG,
		     	  vector<EID>& _M,
		      	  int target_task = -1);

  void PlotAugmentingPath(BipartiteGraph& _bg, vector<EID>& _path);

  void DisplayData(const vector<VID>& vs);
  void DisplayData(const vector<EID>& es);
  
//public:
private:
  gnuplot_ctrl *g;
  gnuplot_ctrl *p;

  unsigned int period;      //plot refreshing period 


};


#endif
