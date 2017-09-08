//
// Created by 研究用 on 2017/09/06.
//

#include "Robot.h"
#include "parameter.h"

#include <map>
#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <random>

using namespace std;

double Gaussian_random(double sigma=1);

bool Robot::check_collision(vector<vector<int> > grid) {
    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[0].size();j++){
            if(grid[i][j]==1){
                double x_center=j;
                double y_center=grid.size()-1-i;
                if(sqrt((x-x_center)*(x-x_center)+(y-y_center)*(y-y_center))<0.5){
                    cout<<"Collision! Position of block:["<<x_center<<", "<<y_center<<"]"<<endl;
                    return true;
                }
            }
        }
    }
    return false;
}

bool Robot::check_goal(IJ goal, vector<vector<int> > grid) {
    double x_goal=goal.getJ();
    double y_goal=grid.size()-1-goal.getI();

    if(sqrt((x-x_goal)*(x-x_goal)+(y-y_goal)+(y-y_goal))<=threshold_goal) return true;
    else return false;
}



void Robot::set_state(double input_x, double input_y, double input_theta) {
    x=input_x;
    y=input_y;
    theta=input_theta;
}

//move
void Robot::move(double input_alpha, double input_distance) {
//cout<<"move following order ["<<input_alpha<<", "<<input_distance<<"]"<<endl;

    //adding noise
    input_distance+=Gaussian_random(distance_noise);
    input_alpha+=Gaussian_random(steering_noise);
    //restricting steering angle
    if(input_alpha>max_steering_angle) input_alpha=max_steering_angle;
    else if(input_alpha<-max_steering_angle) input_alpha=-max_steering_angle;
    //implement mod 2PI for steering angle (resultant steering is in 0-2PI)
    if(input_alpha>2*PI) input_alpha-=2*PI;
    else if(input_alpha<0) input_alpha+=2*PI;

    double R=length/tan(input_alpha);   //R is the radius of the circle made by the rear tire
    double beta=input_distance/R;   //beta is the angle for the rear tire
    double cx=x-R*sin(theta);   //cx, cy are the position of the center of the circle made by the rear tire
    double cy=y+R*cos(theta);

    //if beta is not very small, we do the normal procedure
    if(abs(beta)>=tolerance_move) {
        x = cx + R * sin(theta + beta);
        y = cy - R * cos(theta + beta);
    }
        //if beta is very small, R diverges, so we regard that the particle is going straight.
    else{
        x+=input_distance*cos(theta);
        y+=input_distance*sin(theta);
    }
    theta += beta;

    //角度がmod 2piであることを実装
    if(theta<0) theta+=2.0*PI;
    else if(theta>2.0*PI) theta+= -2.0*PI;
}

//////////////interface//////////////
void Robot::show_state() {
//    cout<<"id: "<<id<<endl;
    cout<<"[x="<<x<<", y="<<y<<", theta="<<theta<<"]"<<endl;
}

//////////////getter and setter////////////
double Robot::getX() const {
    return x;
}
double Robot::getY() const {
    return y;
}
double Robot::getTheta() const {
    return theta;
}
double Robot::getWeight() const {
    return weight;
}
void Robot::setWeight(double weight) {
    Robot::weight = weight;
}
