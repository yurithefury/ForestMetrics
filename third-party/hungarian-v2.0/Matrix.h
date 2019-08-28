
///////////////////////////////////////////////////////////////////////////////
// File name: Matrix.h
// This file defines the basic classes of vertex and edge, and the matrix data
// type is then defined based on them. 
// In the matrix, the grids store the edges, which denote the utilities or 
// costs in corresponding assignment matrix. There are two vectors of
// vertices, one for agents, and the other for tasks.
// Lantao Liu, Nov 1, 2009
// Last modified: 09/2011 -> MTL is removed and 2D vectors are used.
///////////////////////////////////////////////////////////////////////////////
  

#ifndef MATRIX_H
#define MATRIX_H


#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <math.h>

/*
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/matrix.h"
#include "mtl/matrix_implementation.h"

//#define NUMTYPE Edge
#define SHAPE mtl::rectangle<>
#define STORAGE mtl::dense<>
#define ORIEN mtl::row_major
*/

#define TASKS_SIZE 3
#define AGENTS_SIZE 3
#define DOUBLE_EPSILON 1e-7 //For double type comparison: <= as valid, > invalid

#define SEED 0
#define PERIOD 0
#define VERBOSE_LEVEL 1
#define PLOT 0

#define MAX_RANDOM 100


#define POS_INF 10e8
#define NEG_INF -10e8

#ifndef min 
  #define min(x, y) (((x) > (y)) ? (y) : (x))
#endif

#ifndef max
  #define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

using namespace std;
//using namespace mtl;

typedef size_t VID;
typedef pair<size_t, size_t> EID;

class Edge;
class Vertex;

/////////////////////////////////////////////////////////////
//   Define the matrix type
//   Edge is the basic data type used in it
//   The weight of edge denotes the utility or cost
/////////////////////////////////////////////////////////////

//typedef mtl::matrix<Edge, SHAPE, STORAGE, ORIEN>::type Matrix;
typedef vector<vector<Edge> > Matrix;



////////////////////////////////////////////////////////////
//
//  Edge class: represent an element in matrix
//              or an edge in bipartite graph
//
////////////////////////////////////////////////////////////

class Edge{
public:
  Edge(){
    weight = 0;
    matched_flag = false;         //unmatched
    admissible_flag = false;      //inadmissible
  }
  Edge(EID _eid):eid(_eid){
    weight = 0;
    matched_flag = false;         //unmatched
    admissible_flag = false;      //inadmissible
  }
  ~Edge(){}

  EID GetEID(void){ return eid; }
  void SetEID(EID _eid){ eid = _eid; }

  double GetWeight(void){ return weight; }
  void SetWeight(double _weight){ weight = _weight; }

  bool GetMatchedFlag(void){ return matched_flag; }
  void SetMatchedFlag(bool _matched_flag){ matched_flag = _matched_flag;}

  bool GetAdmissibleFlag(void){ return admissible_flag; }
  void SetAdmissibleFlag(bool _admissible){ admissible_flag = _admissible; }

private:
  //data members describing properties of an edge
  EID eid;
  double weight;
  bool matched_flag;
  bool admissible_flag;

};



///////////////////////////////////////////////////////////
//
//  Vertex class: represent an agent or a task 
//
//////////////////////////////////////////////////////////

class Vertex{
public:
  Vertex():label(0), matched(false), colored(false){ 
    //edges.resize(max(AGENTS_SIZE, TASKS_SIZE)); 
    //deltas.resize(max(AGENTS_SIZE, TASKS_SIZE), 0);
  }
  Vertex(VID _vid):vid(_vid),label(0), matched(false), colored(false){ 
    //edges.resize(max(AGENTS_SIZE, TASKS_SIZE)); 
    //deltas.resize(max(AGENTS_SIZE, TASKS_SIZE), 0);
  }
  ~Vertex(){}

  VID GetVID(void){ return vid; }
  void SetVID(VID _vid){ vid = _vid; }

  string GetObj(void){ return obj; }
  void SetObj(string _obj){ obj = _obj; }

  double GetLabel(void){ return label; }
  void SetLabel(double _label){ label = _label; }

  bool GetMatched(void){ return matched; }
  void SetMatched(bool _matched){ matched = _matched; }

  bool GetColored(void){ return colored; }
  void SetColored(bool _colored){ colored = _colored; }

  bool GetVisited(void){ return visited; }
  void SetVisited(bool _visited){ visited = _visited; }

  vector<EID>* GetPath(void){ return &path; }

private:
  //data members describing properties of a vertex
  VID vid;
  string obj;
  double label;
  bool matched;  
  bool colored;     //colored if in the set of T or S
  bool visited;     //to flag if visited when go through alternating tree

public:
  vector<EID> path; //record previous path so far

};


#endif
 

