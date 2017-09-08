//
// Created by 研究用 on 2017/09/06.
//

#ifndef LESSON6_A_SUMMARY_ROBOT_H
#define LESSON6_A_SUMMARY_ROBOT_H

#include <map>
#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <random>
#include "IJ.h"

using namespace std;


class Robot {
private:
    double x,y,theta;   //position and direction of the car
    int id;
    double weight;

public:
    bool check_collision(vector<vector<int> >);
    bool check_goal(IJ, vector<vector<int> >);

    void move(double, double);
    void show_state();

    void set_state(double,double,double);
    void setWeight(double weight);
    double getX() const;
    double getY() const;
    double getTheta() const;
    double getWeight() const;

};


#endif //LESSON6_A_SUMMARY_ROBOT_H
