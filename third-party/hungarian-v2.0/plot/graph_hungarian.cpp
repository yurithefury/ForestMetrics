//#include "gnuplot_i_my.h"
#include "Cgnuplot.h"
#include "graph_hungarian.h"
#include <string.h>
#include <unistd.h>


#ifndef min 
#define min(x, y) (((x) > (y)) ? (y) : (x))
#endif

#ifndef max
#define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

int verbose = 1;
int graph = 1;

gnuplot_ctrl *g = NULL;
gnuplot_ctrl *h = NULL;

void swap_row(int m[N][N], int r1, int r2)
{
    static int hold[N];
    int i;

    for (i =0; i < n; i++)
        hold[i] = m[r1][i];

    for (i =0; i < n; i++)
    {
        m[r1][i] = m[r2][i];
        m[r2][i] = hold[i];
    }
}

void swap_col(int m[N][N], int c1, int c2)
{
    static int hold[N];
    int i;

    for (i =0; i < n; i++)
        hold[i] = m[i][c1];

    for (i =0; i < n; i++)
    {
        m[i][c1] = m[i][c2];
        m[i][c2] = hold[i];
    }
}

void pause_for_user()
{
    char c;
    usleep(200000);
    printf("<< press enter >>\n");
    scanf("%c", &c);
}

void reinit_graph(gnuplot_ctrl *gc)
{
    static int total = 0;
    char id = '0';
    if (gc == g)
        id = 'a';
    else
        id = 'b';
    
    gnuplot_resetplot(gc);
    //gnuplot_cmd(gc, "set terminal svg");
    //gnuplot_cmd(gc, "set output \"gnuplot-%c%04d.svg\"", id, total);
    total++;
}

void draw_graph(int new_member_of_T)
{
    double plx[N];
    double ply[N];
    int count = 0;

    if (!graph) return;

    reinit_graph(g);
    gnuplot_cmd(g, "unset label");
    gnuplot_cmd(g, "set xrange [-0.25:1.25]");
    gnuplot_cmd(g, "set yrange [-0.25:%f]", n-1+.25);
    gnuplot_cmd(g, "set xtics 0");
    gnuplot_cmd(g, "set ytics 0");
    gnuplot_cmd(g, "set nokey");

    // Plot matching
    for (int i = 0; i < n; i++)
    {
        if (xy[i] >= 0)
        {
            plx[0] = 0.0;
            ply[0] = n-i-1;
            plx[1] = 1.0;
            ply[1] = n-xy[i]-1;

            gnuplot_setstyle(g, "lines lc rgb \"#55DD99\" lw 3");
            gnuplot_plot_xy(g, plx, ply, 2, NULL);

        }

    }

    // Plot edges 
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (cost[i][j] == lx[i] + ly[j])
            {
                plx[0] = 0.0;
                ply[0] = n-i-1;
                plx[1] = 1.0;
                ply[1] = n-j-1;

                gnuplot_setstyle(g, "lines lc rgb \"gray\"");
                gnuplot_plot_xy(g, plx, ply, 2, NULL);

                
                plx[0] = 0.11;
                ply[0] = (0.9*ply[0] + 0.1*ply[1]);
                gnuplot_setstyle(g, "points pointsize 2 lc rgb \"#FFFFFF\" pt 7");
                gnuplot_plot_xy(g, plx, ply, 1, NULL);


                gnuplot_cmd(g, "set label \"%d\" at 0.1,%f front tc rgb \"magenta\"", cost[i][j], ply[0]);
            }
        }
    }

    // Plot membership of the S / T sets
    for (int i = 0; i < n; i++)
    {
        if (S[i])
        {
                plx[0] = 0.0;
                ply[0] = n-i-1;
                gnuplot_setstyle(g, "points pointsize 3 lc rgb \"#99AAFF\" pt 7");
                gnuplot_plot_xy(g, plx, ply, 1, NULL);
        }

        if (T[i])
        {
                plx[0] = 1.0;
                ply[0] = n-i-1;
                gnuplot_setstyle(g, "points pointsize 3 lc rgb \"#FF99AA\" pt 7");
                gnuplot_plot_xy(g, plx, ply, 1, NULL);
        }

        if (i == new_member_of_T) // not yet unioned, but the reversal point for the 
        {                         // laternating path
            plx[0] = 1.0;
            ply[0] = n-i-1;
            gnuplot_setstyle(g, "points pointsize 3 lc rgb \"#FF99FF\" pt 7");
            gnuplot_plot_xy(g, plx, ply, 1, NULL);
        }

    }


    // Plot nodes
    count = 0;
    for (int i = 0; i < n; i++)
    {
        plx[count] = 0.0;
        ply[count] = i;
        count++;
    }

    for (int i = 0; i < n; i++)
    {
        plx[count] = 1.0;
        ply[count] = i;
        count++;
    }

    gnuplot_setstyle(g, "points pointsize 2 lc rgb \"#102063\" pt 7");
    gnuplot_plot_xy(g, plx, ply, count, NULL);

    for (int i = 0; i < n; i++)
    {
        gnuplot_cmd(g, "set label \"x%d\" at -0.075,%d tc rgb \"black\"", i, n-i-1);
        gnuplot_cmd(g, "set label \"y%d\" at  1.02,%d tc rgb \"black\"", i, n-i-1);

        gnuplot_cmd(g, "set label \"%d\" at -0.175,%d tc rgb \"#112244\"", lx[i], n-i-1);
        gnuplot_cmd(g, "set label \"%d\" at 1.12,%d tc rgb \"#112244\"", ly[i], n-i-1);
    }

    gnuplot_cmd(g, "replot");

}

void draw_bfs_tree(int root, int addx, bool high_light_root)
{
    bool myS[N], myT[N];         //sets S and T in algorithm
    bool leaf[N];
    double edgex[2];
    double edgey[2];
    int i, j;
    int q[N], wr = 0, rd = 0;          //q - queue for bfs, wr,rd - write and read

    int level[N];

    memset(myS, false, sizeof(S));      
    memset(myT, false, sizeof(T));       
    memset(leaf, false, sizeof(leaf));       
    memset(level, -1, sizeof(level));       

    level[root] = 0;
    S[root] = true;

    q[wr++] = root;
    while (rd < wr)                                                 
    {
        i = q[rd++];         

        if ((root == i) && (high_light_root))
            gnuplot_cmd(h, "set label \"x%d\" at %f,%f front tc rgb \"red\"", i, (float)i+addx, (float)-level[i]);
        else
            gnuplot_cmd(h, "set label \"x%d\" at %f,%f front tc rgb \"black\"", i, (float)i+addx, (float)-level[i]);
        

        for (j = 0; j < n; j++)                                     
        {
                if (cost[i][j] == lx[i] + ly[j] &&  !myT[j])
                {
                    // plot edge between i and j
                    edgex[0] = i+addx;
                    edgey[0] = -level[i];
                    edgex[1] = j+addx;
                    edgey[1] = -(level[i]+0.5);

                    gnuplot_setstyle(h, "lines lc rgb \"#FF2222\" lw 2");
                    gnuplot_plot_xy(h, edgex, edgey, 2, NULL);

                    gnuplot_cmd(h, "set label \"y%d\" at %f,%f front tc rgb \"black\"", j, edgex[1], edgey[1]);

                    leaf[j] = (yx[j] == -1);
                    
                    myT[j] = true;

                    if (!leaf[j])
                    {
                        q[wr++] = yx[j];
                        myS[yx[j]] = true;
                        level[yx[j]] = level[i] + 1;

                        edgex[0] = yx[j]+addx;
                        edgey[0] = -level[yx[j]];
                        edgex[1] = j+addx;
                        edgey[1] = -(level[i]+0.5);

                        gnuplot_setstyle(h, "lines lc rgb \"#FFFF22\" lw 2");
                        gnuplot_plot_xy(h, edgex, edgey, 2, NULL);


                    }
                    else
                    {

                    }

                    
                }
        }
    }
    

}

void draw_all_bfs_trees(int root)
{

    if (!graph) return;

    reinit_graph(h);
    gnuplot_cmd(h, "unset label");
    gnuplot_cmd(h, "set xrange [-.5:%f]", n*(n+1)+0.5);
    gnuplot_cmd(h, "set yrange [%f:0.5]", -n+0.5);
    //gnuplot_cmd(h, "set xtics 0");
    //gnuplot_cmd(h, "set ytics 0");
    gnuplot_cmd(h, "set nokey");

    for (int i = 0;i < n; i++)
    {
        if (xy[i] == -1)
        {
            draw_bfs_tree(i, i*(n+1), (i==root)); // Unmatched roots only, please!
        }
    }

    gnuplot_cmd(h, "replot");

}


void print_S_T_sets()
{
    printf("S = { ");
    for (int i = 0; i < n; i++)
    {
        if (S[i])
            printf("x%d ", i);
    }
    printf("}\n");

    printf("T = { ");
    for (int i = 0; i < n; i++)
    {
        if (T[i])
            printf("y%d ", i);
    }
    printf("}\n");
}

void print_labels()
{
    for (int i = 0; i < n; i++)
    {
        printf("l[x%d] = %d\t", i, lx[i]);
    }
    printf("\n");

    for (int i = 0; i < n; i++)
    {
        printf("l[y%d] = %d\t", i, ly[i]);
    }
    printf("\n");
}

void print_equality_subgraph()
{
    printf("Equality subgraph:\n");
    for (int i = 0; i < n; i++)
    {
        printf("x%d: ", i);
        bool printed = false;
        for (int j = 0; j < n; j++)
        {
            if (cost[i][j] == lx[i] + ly[j])
            {
                printed = true;
                printf("y%d ", j);

            }
        }

        if (!printed)
        {
            printf("(empty)");
        }

        printf("\n");
    }      
    printf("\n");

}

void init_labels()
{
    memset(lx, 0, sizeof(lx));
    memset(ly, 0, sizeof(ly));
    for (int x = 0; x < n; x++)
        for (int y = 0; y < n; y++)
            lx[x] = max(lx[x], cost[x][y]);

    /*
    if (verbose)
    {
        printf("Initial labels set to:\n");
        printf("lx: [ ");
        for (int i = 0; i < n; i++)
            printf("%d\t", lx[i]);
        printf("]\n");
        printf("ly: [ ");
        for (int i = 0; i < n; i++)
            printf("%d\t", ly[i]);
        printf("]\n");

        printf("\t(These are the maximums for all outgoing edges)\n\n");
    }*/
    if (verbose)
    {
        print_equality_subgraph();
    }
 

}

void update_labels()
{
    if (verbose)
    {
        printf("About to update labels\n");
 
        print_S_T_sets();
        
        print_labels();
    }

    int x, y, delta = INF;             //init delta as infinity
    for (y = 0; y < n; y++)            //calculate delta using slack
    {
        if (!T[y])
        {
            //if (verbose)
            //    printf("delta = min(%d, %d) // doing y%d\n", delta, slack[y], y);
            
            delta = min(delta, slack[y]);
        }
    }

    if (verbose)
    {
        printf("New edges to be added will be to verts: { ");
        for (y = 0; y < n; y++)            //calculate delta using slack
        {
            if ((!T[y]) && (delta == slack[y]))
            {
                printf("y%d ", y);
            }
        }
        printf("}\n");


        // although delta is calculated efficiently using slack variables, 
        // it is just a calculation that satisfies:
        //  delta = min_{x,y} [ l(x) + l(y) - Cost(x,y) ] 
        //          where x \in S
        //          where y \in Y\T

        // We print this the long way

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if ((S[i]) && (!T[j]))
                {
                    printf("l(x%d) + l(y%d) - Cost(x%d, y%d) = %d + %d - %d = %d\n",
                                i, j, i, j, lx[i], ly[j], cost[i][j], lx[i] + ly[j] - cost[i][j]);
                }
            }
        }
        printf("min = %d\n", delta);
    }


    for (x = 0; x < n; x++)            //update X labels
        if (S[x]) lx[x] -= delta;
    for (y = 0; y < n; y++)            //update Y labels
        if (T[y]) ly[y] += delta;
    for (y = 0; y < n; y++)            //update slack array
        if (!T[y])
            slack[y] -= delta;

    if (verbose)
    {
        printf("Updated labels (delta = %d)\n", delta);



        print_labels();

        print_equality_subgraph();


    }
}

void add_to_tree(int x, int prevx) 
//x - current vertex,prevx - vertex from X before x in the alternating path,
//so we add edges (prevx, xy[x]), (xy[x], x)
{
    S[x] = true;                    //add x to S
    prev[x] = prevx;                //we need this when augmenting
    for (int y = 0; y < n; y++)    //update slacks, because we add new vertex to S
        if (lx[x] + ly[y] - cost[x][y] < slack[y])
        {
            slack[y] = lx[x] + ly[y] - cost[x][y];
            slackx[y] = x;
        }

    
}


void print_matching()
{
    bool printed = false;
    for (int i=0; i < n; i++)
    {
        if (xy[i] != -1)
        {
            printf("x%d <====> y%d\n", i, xy[i]);
            printed = true;
        }
    }
    if (!printed) 
        printf("(Empty)");
    printf("\n");
}


void augment()                         //main function of the algorithm
{
    if (verbose)
    {
        printf("\n\nAugment(.) call\n");
        printf("Current matching is as follows:\n");
        print_matching();
    }

    if (max_match == n) return;        //check wether matching is already perfect
    int x, y, root;                    //just counters and root vertex
    int q[N], wr = 0, rd = 0;          //q - queue for bfs, wr,rd - write and read

    root = 0;                          // stop warning, it will be assigned below before xy has
                                       // a bunch of -1's in it to start.

                                       //pos in queue
    memset(S, false, sizeof(S));       //init set S
    memset(T, false, sizeof(T));       //init set T
    memset(prev, -1, sizeof(prev));    //init set prev - for the alternating tree
    for (x = 0; x < n; x++)            //finding root of the tree
        if (xy[x] == -1)
        {
            q[wr++] = root = x;
            prev[x] = -2;
            S[x] = true;
            break;
        }


    for (y = 0; y < n; y++)            //initializing slack array
    {
        slack[y] = lx[root] + ly[y] - cost[root][y];
        slackx[y] = root;
    } //second part of augment() function

    
    while (true)                                                        //main cycle
    {
        while (rd < wr)                                                 //building tree with bfs cycle
        {
            x = q[rd++];                                                //current vertex from X part
            for (y = 0; y < n; y++)                                     //iterate through all edges in equality graph
                if (cost[x][y] == lx[x] + ly[y] &&  !T[y])
                {
                    if (yx[y] == -1) break;                             //an exposed vertex in Y found, so
                                                                        //augmenting path exists!
                    T[y] = true;                                        //else just add y to T,
                    q[wr++] = yx[y];                                    //add vertex yx[y], which is matched
                                                                        //with y, to the queue
                    add_to_tree(yx[y], x);                              //add edges (x,y) and (y,yx[y]) to the tree
                }
            if (y < n) break;                                           //augmenting path found!
        }
        if (y < n) break;                                               //augmenting path found!

        if (verbose)
        {
            printf("Augmenting path not found\n");

            print_S_T_sets();
        }

        update_labels();                                                //augmenting path not found, so improve labeling

        draw_all_bfs_trees(root);

        wr = rd = 0;                
        for (y = 0; y < n; y++)        
        //in this cycle we add edges that were added to the equality graph as a
        //result of improving the labeling, we add edge (slackx[y], y) to the tree if
        //and only if !T[y] &&  slack[y] == 0, also with this edge we add another one
        //(y, yx[y]) or augment the matching, if y was exposed
            if (!T[y] &&  slack[y] == 0)
            {
                if (yx[y] == -1)                                        //exposed vertex in Y found - augmenting path exists!
                {
                    x = slackx[y];
                    break;
                }
                else
                {
                    T[y] = true;                                        //else just add y to T,
                    if (!S[yx[y]])    
                    {
                        q[wr++] = yx[y];                                //add vertex yx[y], which is matched with
                                                                        //y, to the queue
                        add_to_tree(yx[y], slackx[y]);                  //and add edges (x,y) and (y,
                                                                        //yx[y]) to the tree
                    }
                }
            }
        if (y < n) break;                                               //augmenting path found!
    }


    if (y < n)                                                          //we found augmenting path!
    {

        if (verbose)
        {
            draw_graph(y); 

            printf("Augmenting path found\n");

            printf("Exposed vertex y%d (via x%d)\n", y, x);

            printf("Matching before is as follows:\n");
            print_matching();

            draw_all_bfs_trees(root);
            pause_for_user();

            printf("Augmenting path:\n");
            //printf("(y) y%d --> x%d (x) ", y, x);
            printf("y%d --> x%d ", y, x);
            fflush(stdout);
            int tx = x;
            while (prev[tx] != -2)
            {
                printf("<===> y%d ", xy[tx]);
                fflush(stdout);
                tx = prev[tx];
                printf("--> x%d ", tx);
                fflush(stdout);
                if (tx == -1)
                {
                    printf("error :-(\n");
                    break;
                }

            }
            printf("\n");

            
        }

        max_match++;                                                    //increment matching
        //in this cycle we inverse edges along augmenting path
        for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty)
        {
            ty = xy[cx];
            yx[cy] = cx;
            xy[cx] = cy;
        }

        if (verbose)
        {

            draw_graph(y); 
            pause_for_user();

            printf("Matching after is as follows:\n");
            print_matching();
            
            printf("\n");

        }



        augment();                                                      //recall function, go to step 1 of the algorithm
    }
}//end of augment() function

int hungarian()
{
    int ret = 0;                      //weight of the optimal matching
    max_match = 0;                    //number of vertices in current matching
    memset(xy, -1, sizeof(xy));    
    memset(yx, -1, sizeof(yx));
    init_labels();                    //step 0
    augment();                        //steps 1-3
    for (int x = 0; x < n; x++)       //forming answer there
        ret += cost[x][xy[x]];
    return ret;
}

int main() 
{
    if (graph)
    {
        g = gnuplot_init();
        h = gnuplot_init();
    }

    /*
    n = 3;
    cost[0][0] = 2;
    cost[0][1] = 1;
    cost[0][2] = 3;
    cost[1][0] = 7;
    cost[1][1] = 4;
    cost[1][2] = 3;
    cost[2][0] = 3;
    cost[2][1] = 0;
    cost[2][2] = 0;
    */
    

    // Worst case:
    n = 3;
    cost[0][0] = 11;  // x = 11, e = 1, k = 2, l = 2
    cost[0][1] = 10;
    cost[1][0] = 9;
    cost[1][1] = 7;
    cost[0][2] = 2;
    cost[1][2] = 3;
    cost[2][0] = 30;
    cost[2][1] = 40;
    cost[2][2] = 39;

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            printf("%d\t", cost[i][j]);
        }
        printf("\n");
    }
    printf("\n");

    hungarian();

    for (int i = 0; i < n; i++)
        printf("%d\t", lx[i]);
    printf("\n\n");

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (xy[i] == j)
                printf("%d\t", 1);
            else
                printf("%d\t", 0);
                
        }
        printf("| %d\n", ly[i]);
    }
    printf("\n");

    printf("Slack: ");
    for (int i = 0; i < n; i++)
        printf("%d\t", slack[i]);
    printf("\n");
    printf("Slackx: ");
    for (int i = 0; i < n; i++)
        printf("%d\t", slackx[i]);
    printf("\n");

    if (g != NULL)
        gnuplot_close(g);
    if (h != NULL)
        gnuplot_close(h);

    return 0;
}
