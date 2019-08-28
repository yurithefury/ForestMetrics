
#include "PlotGraph.h"
#include <sstream>
#include <iomanip>

//#define SAVE_PLOTS
int Cnt = 0;

void 
PlotGraph::PlotBipartiteGraph(BipartiteGraph& _bg,
			      vector<VID>& _S, 
			      vector<VID>& _T,
			      vector<VID>& _N,
			      vector<EID>& _EG,
			      vector<EID>& _M, 
			      int target_task){

    //get assignment size, assume the sizes of agents and tasks are identical
    size_t as_size = _bg.GetNumAgents();
    double plx[as_size];
    double ply[as_size];

    //g=gnuplot_init();
    gnuplot_resetplot(g);
    gnuplot_cmd(g, (char*)"unset label");
    gnuplot_cmd(g, (char*)"set xrange [-0.25:%f]", as_size-1+.25);
    gnuplot_cmd(g, (char*)"set yrange [-0.25:1.25]");
    gnuplot_cmd(g, (char*)"set xtics 0");
    gnuplot_cmd(g, (char*)"set ytics 0");
    gnuplot_cmd(g, (char*)"set nokey");

    // Plot matching
    //cout<<endl<<"Set M: "<<endl;
    //DisplayData(_M);    
    for (unsigned int i = 0; i < _M.size(); i++){
        plx[0] = _M[i].first;
        ply[0] = 0.0;
        plx[1] = _M[i].second;
        ply[1] = 1.0;

        gnuplot_setstyle(g, (char*)"lines lc rgb \"red\" lw 3");
        gnuplot_plot_xy(g, plx, ply, 2, NULL);
    }

    // Plot admissible edges (EG)
    //cout<<"EG: "<<endl;
    //DisplayData(_EG);
    for(unsigned int i=0; i<_EG.size(); i++){
        plx[0] = _EG[i].first;
        ply[0] = 0.0;
        plx[1] = _EG[i].second;
        ply[1] = 1.0;

        gnuplot_setstyle(g, (char*)"lines lc rgb \"gray\"");
        gnuplot_plot_xy(g, plx, ply, 2, NULL);
                
        plx[0] = (0.9*plx[0] + 0.1*plx[1]);
        ply[0] = 0.11;
        gnuplot_setstyle(g, (char*)"points pointsize 2 lc rgb \"#FFFFFF\" pt 7");
        gnuplot_plot_xy(g, plx, ply, 1, NULL);

        //cout<<"EG "<<i<<": "<<_bg.GetMatrix(_EG[i])->GetWeight()<<endl;
        gnuplot_cmd(g, (char*)"set label \"%d\" at %f-0.04,0.1 front tc rgb \"#4682B4\" font \",8\"", int(_bg.GetMatrix(_EG[i])->GetWeight()), plx[0]);
    }


    // Plot membership of set S
    //cout<<"S: "<<endl;
    //DisplayData(_S);
    for(unsigned int i=0; i<_S.size(); i++){
        plx[0] = _S[i];
        ply[0] = 0.0;
        gnuplot_setstyle(g, (char*)"points pointsize 3 lc rgb \"green\" pt 7");
        gnuplot_plot_xy(g, plx, ply, 1, NULL);
    }

    // Plot membership of the set T
    //cout<<"T: "<<endl;
    //DisplayData(_T);
    for(unsigned int i=0; i<_T.size(); i++){
        plx[0] = _T[i];
        ply[0] = 1.0;
        gnuplot_setstyle(g, (char*)"points pointsize 3 lc rgb \"#00BFFF\" pt 7");
        gnuplot_plot_xy(g, plx, ply, 1, NULL);
    }

        if (target_task != -1) // not yet unioned, but the reversal point for the 
        {                         // alternating path
            plx[0] = target_task;
            ply[0] = 1.0;
            gnuplot_setstyle(g, (char*)"points pointsize 3 lc rgb \"grey\" pt 7");
            gnuplot_plot_xy(g, plx, ply, 1, NULL);
        }


    // Plot nodes
    for(unsigned int i = 0; i < as_size; i++)
    {
        plx[i] = i;
        ply[i] = 0.0;
    }

    gnuplot_setstyle(g, (char*)"points pointsize 2 lc rgb \"#102063\" pt 7");
    gnuplot_plot_xy(g, plx, ply, as_size, NULL);

    for(unsigned int i = 0; i < as_size; i++)
    {
        plx[i] = i;
        ply[i] = 1.0;
    }

    gnuplot_setstyle(g, (char*)"points pointsize 2 lc rgb \"#102063\" pt 7");
    gnuplot_plot_xy(g, plx, ply, as_size, NULL);

    for (unsigned int i = 0; i < as_size; i++)
    {
        gnuplot_cmd(g, (char*)"set label \"x%d\" at %d-0.025,-0.08 tc rgb \"black\"", i, i);
        gnuplot_cmd(g, (char*)"set label \"y%d\" at %d-0.025,1.08 tc rgb \"black\"", i, i);
	//cout<<"Label "<<i<<": "<<_bg.GetAgent(i)->GetLabel()<<endl;
	//cout<<"Label "<<i<<": "<<_bg.GetTask(i)->GetLabel()<<endl;
        gnuplot_cmd(g, (char*)"set label \"%d\" at %d-0.025,-0.15 tc rgb \"#112244\"", int(_bg.GetAgent(i)->GetLabel()), i);
        gnuplot_cmd(g, (char*)"set label \"%d\" at %d-0.025,1.15 tc rgb \"#112244\"", int(_bg.GetTask(i)->GetLabel()), i);
    }

/*    //image setting
       set terminal png 
              {{no}transparent} {{no}interlace}
              {tiny | small | medium | large | giant}
              {font <face> {<pointsize>}}
              {size <x>,<y>} {{no}crop}
              {{no}enhanced}
              {<color0> <color1> <color2> ...}
*/

#ifdef SAVE_PLOTS
    stringstream ss;
    ss << setw(3) << setfill('0') << Cnt++;
    string postfix = ss.str();

    string name="save/plot"+postfix+".jpeg";
    gnuplot_cmd(g, (char*)"set terminal jpeg");
    //gnuplot_cmd(g, (char*)"set terminal jpeg small size 320,240");//bad
    gnuplot_cmd(g, (char*)"set output \"%s\"", name.c_str());
    printf("saved jpeg: \"%s\"\n", name.c_str());
#endif

    gnuplot_cmd(g, (char*)"replot");

    sleep(period);
    //gnuplot_close(g);
}


void
PlotGraph::PlotAugmentingPath(BipartiteGraph& _bg, vector<EID>& _path){

    //get assignment size, assume the sizes of agents and tasks are identical
    size_t as_size = _bg.GetNumAgents();
    double plx[as_size];
    double ply[as_size];
/*
    //gnuplot_resetplot(g);
    gnuplot_cmd(g, (char*)"unset label");
    gnuplot_cmd(g, (char*)"set xrange [-0.25:1.25]");
    gnuplot_cmd(g, (char*)"set yrange [-0.25:%f]", as_size-1+.25);
    gnuplot_cmd(g, (char*)"set xtics 0");
    gnuplot_cmd(g, (char*)"set ytics 0");
    gnuplot_cmd(g, (char*)"set nokey");
*/
    //gnuplot_cmd(g, (char*)"set style line 2 lt 2 lc rgb \"blue\" lw 3");
    // Plot matching
    //cout<<endl<<"Set M: "<<endl;
    //DisplayData(_M);    
    bool alter = true;
    for (unsigned int i = 0; i < _path.size(); i++){
        plx[0] = _path[i].first;
        ply[0] = 0.0;
        plx[1] = _path[i].second;
        ply[1] = 1.0;

        //gnuplot_setstyle(g, (char*)"linespoints lc rgb \"#55DD99\" lw 3");
        if(alter)
          gnuplot_setstyle(g, (char*)"lines lc rgb \"#458B74\" lw 3");
        else
          gnuplot_setstyle(g, (char*)"lines lc rgb \"#7CCD7C\" lw 3");

        alter = !alter;
        gnuplot_plot_xy(g, plx, ply, 2, NULL);
    }

#ifdef SAVE_PLOTS
    stringstream ss;
    ss << setw(3) << setfill('0') << Cnt++;
    string postfix = ss.str();

    string name="save/plot"+postfix+".jpeg";
    gnuplot_cmd(g, (char*)"set terminal jpeg");
    //gnuplot_cmd(g, (char*)"set terminal jpeg small size 320,240");//bad
    gnuplot_cmd(g, (char*)"set output \"%s\"", name.c_str());
    printf("saved jpeg: \"%s\"\n", name.c_str());
#endif

    gnuplot_cmd(g, (char*)"replot");

   sleep(period);
}

void
PlotGraph::DisplayData(const vector<VID>& vs){

  //for(vector<VID>::const_iterator itr = vs.begin(); itr != vs.end(); itr++)
    //cout<<*itr<<" ";
  //cout<<endl;

}

void
PlotGraph::DisplayData(const vector<EID>& es){

  //for(vector<EID>::const_iterator itr = es.begin(); itr != es.end(); itr++)
    //cout<<"("<<itr->first<<","<<itr->second<<") ";
  //cout<<endl;

}







