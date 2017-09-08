//
// Created by 研究用 on 2017/09/06.
//

#ifndef LESSON6_A_SUMMARY_PLAN_H
#define LESSON6_A_SUMMARY_PLAN_H

#include <map>
#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include "IJ.h"
#include "GIJ.h"
#include "XY.h"

using namespace std;


class Plan {
private:
    vector<vector<int> > grid,h,expand_table,f;
    vector<IJ> path;    //[i,j] style
    vector<XY> Path,OriginalPath,PreviousPath;  //[x,y] style
    IJ goal, start;
    vector<GIJ> list,checked;
    int count_expand, number_step;

public:
    Plan(vector<vector<int> > grid_input, IJ start_input, IJ goal_input);   //constructor

    ///////////functions for processing (path planning)///////////
    void evaluate_h();
    void proceed_Astar();
    void expand_next_Astar();
    void expand(int,int,int);
    bool judge();
    void find_path();

    ///////////functions for processing (path smoothing)///////////
    void convert_path_ij_xy();
    void show_change_path();
    void smooth_path();
    bool judge_smoothness();
    void write_change_path();
    void write_block();

    ///////////interface//////////
    void show_grid();
    void show_h();
    void show_g();
    void show_f();
    void show_expand_table();
    void show_path();
    void show_list();
    void show_checked();
    void show_number_step();

    ///////////getter and setter//////////
    const vector<XY> &getPath() const;

};


#endif //LESSON6_A_SUMMARY_PLAN_H
