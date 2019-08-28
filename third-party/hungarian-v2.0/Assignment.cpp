
#include "Assignment.h"


Matrix 
Assignment::RandomGenerate(size_t nrows, size_t ncols, int MAX, unsigned int _seed){
 
  //accept new seed for random generator
  if(_seed != SEED)
    srand(_seed);
  else 
    srand(this->seed);
  
  //update data members
  this->num_agents = nrows;
  this->num_tasks = ncols;

  //define a matrix
  Matrix matrix;
  matrix.resize(nrows);
  for(unsigned int i=0; i<nrows; i++)
    matrix[i].resize(ncols);

  //randomly generate
  for(unsigned int i=0; i<nrows; i++)
    for(unsigned int j=0; j<ncols; j++){
      int rdm=rand()%MAX-1; 
      if(rdm<0) rdm = 0;
      matrix[i][j].SetWeight(rdm);
  }      
  return matrix;
}

Matrix
Assignment::ImportAssignment(ifstream& input_file){
  Matrix matrix;
  string line;
  vector<double> numstream;

  size_t num_rows = 0;
  size_t num_cols = 0;

  if (input_file.is_open())
  {
    while (!input_file.eof() )
    {
      getline (input_file,line);

      size_t local_num_cols=0;
      vector<double> local_numstream;
      local_numstream.clear();
      string word;

      stringstream parse(line);
      while(parse >> word){
	//if comment line, ignore it
	if(word[0] == '#')
	  break;
	//numstream.push_back(atoi(word.c_str()));  //if not number, then convert to zero
	numstream.push_back(atof(word.c_str()));  //if not number, then convert to zero
	//local_numstream.push_back(atoi(word.c_str()));
	local_numstream.push_back(atof(word.c_str()));

	//matrix(num_rows, num_cols)= atoi(word.c_str()); 	
	local_num_cols++;
      } //end inner while

      //judge if the matrix format is correct or not
      if(num_cols && local_num_cols && num_cols!=local_num_cols){
	cerr<<endl<<"Please input a correct matrix format!"<<endl<<endl;
	exit(-1);
      }
      //update column number if everything looks normal
      if(local_num_cols)
	num_cols = local_num_cols;
      //update row number if everything looks normal
      if(line.length()&&local_numstream.size())
        num_rows++;

    } //end out layer while

    input_file.close();

    //update class data members
    this->num_agents = num_rows;
    this->num_tasks = num_cols;

    //put elements into matrix
    //matrix.resize(num_rows, num_cols);
    matrix.resize(num_rows);
    for(unsigned int i=0; i<num_rows; i++)
      matrix[i].resize(num_cols);

    vector<double>::iterator itr = numstream.begin();
    for(unsigned int i=0; i<num_rows; i++)
      for(unsigned int j=0; j<num_cols; j++)
	matrix[i][j].SetWeight(*itr++);
  } //end outmost if
  else{ 
    cerr <<endl<<"Error: Unable to open file! Stopped."<<endl<<endl; 
    exit(-1);
  }

  return matrix;
}


void
Assignment::DisplayMatrix(Matrix& m) const{

  if(m[0].size()>30){
    //cout<<endl<<"Queried assignment matrix is big, not displaying."<<endl;
    return;
  }

  //cout<<endl<<"The assignment problem (matrix) you queried is:"<<endl<<endl;
  //for(unsigned int i=0; i<m.size(); i++){
    //for(unsigned int j=0; j<m[0].size(); j++)
      //cout<<"  "<<m[i][j].GetWeight()<<"\t";
    //cout<<endl;
  //}
  //cout<<endl;
}


