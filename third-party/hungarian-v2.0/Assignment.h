
///////////////////////////////////////////////////////////////////////////////
// File name: Assignment.h
// This file defines the class of Assignment. 
// The assignment matrix can be obtained either by randomly generated, or by 
// reading from a specified input file. For the input file, user is required to 
// specify a nXn matrix. Parser has some intelligence, so no strict format is 
// required. E.g. put a file named 'example' in current directory.
// Lantao Liu, Nov 1, 2009
///////////////////////////////////////////////////////////////////////////////

#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H


#include "Matrix.h"

//#define SEED 0  //uniformly defined in Matrix.h

using namespace std;


///////////////////////////////////////////////////////////////////////////////
//
// Assignment class: defined methods for obtaining an assignment matrix
//
///////////////////////////////////////////////////////////////////////////////

class Assignment{
public:
  Assignment(unsigned int _seed = SEED):seed(_seed){}
  ~Assignment(){}
  
  //Set the seed for random generator
  inline void SetSeed(unsigned int _seed){ seed = _seed; }
  inline unsigned int GetSeed(void){return seed; }

  inline pair<size_t,size_t> GetAssignmentSize(void){return pair<size_t, size_t>(num_agents, num_tasks); }

  //Randomly generate an assignment-matrix, the default arguments are pre-set
  //Currently can generate only integer numbers
  Matrix RandomGenerate(size_t nrows = AGENTS_SIZE, size_t ncols = TASKS_SIZE, 
		int MAX = 100, unsigned int _seed = SEED);
 
  //Import an assignment-matrix from external file
  Matrix ImportAssignment(ifstream&);

  //Display matrix onto screen
  void DisplayMatrix(Matrix&) const;


private:
  //basic data members
  unsigned int seed;
  size_t num_agents;
  size_t num_tasks;
  
};

#endif
