//
// Created by 研究用 on 2017/09/06.
//

#ifndef LESSON6_A_SUMMARY_PARTICLEFILTER_H
#define LESSON6_A_SUMMARY_PARTICLEFILTER_H

#include <map>
#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <random>
#include "Robot.h"
#include "XY.h"

using namespace std;

class ParticleFilter {
private:
    vector<Robot> robots;
    Robot true_position;
    double CTE,CTE_previous;
    double x_average, y_average, theta_average;
    int count_collision, count_step;
    vector<vector<int>> grid;
    IJ goal;
    vector<XY> path;

public:
    ParticleFilter(double,double,double);

    //main routine
    void proceed();

    //movement phase
    void move(double,double);
    double PD_control();
    void evaluate_cross_track_error();

    //measurement phase
    XY sense();
    void evaluate_weight(XY);
    void resample();
    void evaluate_average();

    //interface
    void show_true_state();
    void write_particles();
    void write_summary();
    void show_state_all();

    //routines for test
    void set_particles(double,double,double);

    //getter-setter
    void setGrid(const vector<vector<int>> &grid);
    void setGoal(const IJ &goal);
    void setPath(const vector<XY> &path);

};


#endif //LESSON6_A_SUMMARY_PARTICLEFILTER_H
