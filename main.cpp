#include <iostream>
#include <vector>
#include "Plan.h"
#include "ParticleFilter.h"
#include "parameter.h"

using namespace std;


int main() {
    ///////////////Path planning phase///////////////
    //install grid, start, and goal
    vector<vector<int> > grid(5);
    for(int i=0;i<5;i++) grid[i].resize(6);
    grid[0][0]=0; grid[0][1]=1; grid[0][2]=0; grid[0][3]=0; grid[0][4]=0; grid[0][5]=0;
    grid[1][0]=0; grid[1][1]=1; grid[1][2]=0; grid[1][3]=1; grid[1][4]=1; grid[1][5]=0;
    grid[2][0]=0; grid[2][1]=1; grid[2][2]=0; grid[2][3]=1; grid[2][4]=0; grid[2][5]=0;
    grid[3][0]=0; grid[3][1]=0; grid[3][2]=0; grid[3][3]=1; grid[3][4]=0; grid[3][5]=1;
    grid[4][0]=0; grid[4][1]=1; grid[4][2]=0; grid[4][3]=1; grid[4][4]=0; grid[4][5]=0;
    IJ start(0,0);
    IJ goal(grid.size()-1,grid[0].size()-1);
    Plan plan(grid,start,goal);
    plan.show_grid();

    //path planning with A*
    plan.proceed_Astar();
//plan.show_expand_table();
//plan.show_h();
//plan.show_f();
//plan.show_g();
    plan.show_number_step();
    plan.find_path();
//    plan.show_path();

    ///////////////Path smoothing phase///////////////
    plan.convert_path_ij_xy();
    plan.smooth_path();
//plan.show_change_path();
    plan.write_change_path();
    plan.write_block();
    vector<XY> Path=plan.getPath(); //this is the output! Its form is vector<XY>.
for(int i=0;i<Path.size();i++) Path[i].show_nonewline();

    ///////////////Test for ParticleFilter Class//////////////
    ParticleFilter test(start.getJ(), grid.size()-1-start.getI(), 3*PI/2);
    test.setGrid(grid);
    test.setGoal(goal);
    test.setPath(Path);

    //test for CTE
    test.set_particles(start.getJ()+0.3, grid.size()-1-start.getI()-0.001, 3*PI/2);
    test.evaluate_cross_track_error();
    test.set_particles(2, 2, 3*PI/2);
    test.evaluate_cross_track_error();


    ///////////////Robot movement and measurement phase///////////////
    //specify the starting point
    ParticleFilter filter(start.getJ(), grid.size()-1-start.getI(), 3*PI/2);
    filter.setGrid(grid);
    filter.setGoal(goal);
    filter.setPath(Path);
    filter.show_true_state();
    filter.proceed();


    return 0;
}

