
///////////////////////////////////////////////////////////////////////////////
// File Name: CmdParser.h
// This file defines a class for parsing command line
// Lantao Liu, Nov 1, 2009
///////////////////////////////////////////////////////////////////////////////


#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <iostream>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include "Matrix.h"

using namespace std;

int _Period;
int _Verbose;
bool _Plot;

class CmdParser{
public:
  CmdParser(int _argc, char **_argv):seed(SEED), 
				     assignment_size(AGENTS_SIZE),
				     period(PERIOD), 
				     verbose_level(VERBOSE_LEVEL),
				     plot(PLOT){ 
    ParseCmd(_argc, _argv); 
    _Plot = plot;
    _Period = period;	
    _Verbose = verbose_level;
  }
  ~CmdParser(){}
  
  string GetInputFileName(void){ return input_file; }
  unsigned int GetSeed(void){ return seed; }
  size_t GetAssignmentSize(void){ return assignment_size; }

  void ParseCmd(int argc, char **argv);
  
  void DisplayInput(void);

  void GiveUsage(char *cmd);
  
public:
  unsigned int seed;
  string input_file;
  size_t assignment_size;
  unsigned int period; 
  unsigned int verbose_level;
  bool plot;

};

void
CmdParser::ParseCmd(int argc, char **argv){

  int c;
  while ((c = getopt (argc, argv, "i:s:n:p:t:v:h")) != -1)
    switch (c)
    {
           case 'i':
             this->input_file = optarg;
             break;
           case 's':
             this->seed = atoi(optarg);
             break;
           case 'n':
             this->assignment_size = atoi(optarg);
             break;
           case 'p':
             this->plot = atoi(optarg);
             break;
           case 't':
             this->period = atoi(optarg);
             break;
           case 'v':
             this->verbose_level = atoi(optarg);
             break;
	   case 'h':
	     this->GiveUsage(argv[0]);
	     break;
           case '?':
             if (isprint (optopt))
               fprintf (stderr, "Unknown option `-%c'.\n", optopt);
             else
               fprintf (stderr,
                        "Unknown option character `\\x%x'.\n",
                        optopt);
	     this->GiveUsage(argv[0]);
             abort();
           default:
	     //cout<<"Error: Could not understand the command line."<<endl;
             exit(0);
    }
}

void
CmdParser::DisplayInput(void){

  //if(this->input_file != "")
    //cout<<"  Accept new input file: "<<input_file<<endl;
  
  //if(this->seed != SEED)
    //cout<<"  Accept new seed: "<<seed<<endl;

  //if(this->assignment_size != AGENTS_SIZE)
    //cout<<"  Accept new assignment size: "<<assignment_size<<endl;

  //if(this->period != PERIOD)
    //cout<<"  Accept new period: "<<period<<endl;
  
  //if(this->verbose_level != 0)
    //cout<<"  Accept new verbose level: "<<verbose_level<<endl;

}

void
CmdParser::GiveUsage(char *cmd)
{
   fprintf(stderr,"\nUsage:    %s \n",cmd);
   fprintf(stderr,"          -i   input an assignment file\n");
   fprintf(stderr,"          -s   random generator with seed: unsigned int\n");
   fprintf(stderr,"          -n   matrix size: size_t\n");
   fprintf(stderr,"          -p   plot level: 0, 1, 2\n");
   fprintf(stderr,"          -t   time period for slide show: size_t\n");
   fprintf(stderr,"          -v   verbose level: 0, 1, 2\n");
   fprintf(stderr,"More details can be found in README file\n\n");
   exit(-1);
}


#endif
