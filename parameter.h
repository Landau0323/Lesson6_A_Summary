//
// Created by 研究用 on 2017/09/06.
//

#ifndef LESSON6_A_SUMMARY_PARAMETER_H
#define LESSON6_A_SUMMARY_PARAMETER_H

const double PI=3.14159265359;

//parameters for path smoothing
const double weight_data = 0.1;
const double weight_smooth = 0.2;
const double tolerance_smoothing = 0.000001;

//parameters for movement of robot
const double length=0.5;    //length of car
const double tolerance_move=0.001;  //if angle is smaller than this value, movement is approximated with going straightly.
const double max_steering_angle = PI/4.0;
const double speed=0.1;

const double threshold_goal=1.0;    //if robot is within this distance to the goal, regard as "goal in"

//parameters for control (PD)
const double p_gain = 2.0;
const double d_gain = 6.0;

//parameters for localization (Gaussian noise)
const double steering_noise    = 0.1;
const double distance_noise    = 0.03;
const double measurement_noise = 0.3;
const int N=100;    //particle number in particle filter

#endif //LESSON6_A_SUMMARY_PARAMETER_H
