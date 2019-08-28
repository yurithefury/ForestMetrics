
///////////////////////////////////////////////////////////////////////////////
// File name: main.cpp
// This file calls the components and algorithms to complete Hungarian task.
// Lantao Liu, Nov 1, 2009
/////////////////////////////////////////////////////////////////////////////// 

#include "CmdParser.h"
#include "Assignment.h"
#include "Hungarian.h"
#include "BipartiteGraph.h"


int main(int argc, char** argv)
{
  //define a matrix;
  Matrix m;

  //parse command line and generate an assignment
  CmdParser parser(argc, argv);
  parser.DisplayInput();

  Assignment as;
  if(parser.GetInputFileName().empty()){
    if(parser.GetSeed())
      as.SetSeed(parser.GetSeed());
    else
      as.SetSeed(time(NULL));
    cout<<endl<<"  *Seed for random generator: "<<as.GetSeed()<<endl;
    m=as.RandomGenerate( parser.GetAssignmentSize(), 
			 parser.GetAssignmentSize(), 
			 MAX_RANDOM, 
			 parser.GetSeed() );
  }
  else{
    ifstream myfile(parser.GetInputFileName().c_str());
    m=as.ImportAssignment(myfile);
  }
  as.DisplayMatrix(m);

  //define a bipartite graph
  BipartiteGraph bg(m);
  
  //run Hungarian method
  Hungarian h(bg);
  h.HungarianAlgo();

  //for testing
  //Test t;
  //t.testHungarian();

  return 0;
}

