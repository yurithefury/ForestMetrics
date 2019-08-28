#ifndef GR_HUNG
#define GR_HUNG

#include <stdio.h>
#include <stdlib.h>


// FROM http://www.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm
// AUTHOR: x-ray

#define N 55             //max number of vertices in one part
#define INF 100000000    //just infinity

int cost[N][N];          //cost matrix
int n, max_match;        //n workers and n jobs
int lx[N], ly[N];        //labels of X and Y parts
int xy[N];               //xy[x] - vertex that is matched with x,
int yx[N];               //yx[y] - vertex that is matched with y
bool S[N], T[N];         //sets S and T in algorithm
int slack[N];            //as in the algorithm description
int slackx[N];           //slackx[y] such a vertex, that
                         // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
int prev[N];             //array for memorizing alternating paths


void init_labels();

void update_labels();

void add_to_tree(int x, int prevx);

void augment();

int hungarian();

#endif

