//
// Created by 研究用 on 2017/09/06.
//

#include "ParticleFilter.h"
#include "parameter.h"

#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <random>

double Gaussian_random(double sigma=1);

using namespace std;

//constructor
ParticleFilter::ParticleFilter(double x_init,double y_init, double theta_init) {
    cout<<"particle filter start!"<<endl;
    robots.resize(N);
    //initial state of particles are known and specified
    for(int i=0;i<N;i++) robots[i].set_state(x_init,y_init,theta_init);
    //also set the real position and orientation
    true_position.set_state(x_init,y_init,theta_init);

    //initialize CTE and derivative of CTE
    CTE=CTE_previous=0;
    count_step=count_collision=0;
}

///////////////main routine/////////////////
void ParticleFilter::proceed() {
    while(true_position.check_goal(goal,grid)==false){
//for(count_step=0;count_step<10;count_step){ //for debug
        /////////movement phase////////
        evaluate_cross_track_error();
        move(PD_control(),speed);   //control (based on mean position of particles)
        if(true_position.check_collision(grid)==true) count_collision++;

        /////////measurement phase///////
        //measurement (generate measurement result based on true_state -> resampling for particles)
        evaluate_weight(sense());
        resample();

        /////////others//////////
        count_step++;
        write_particles();
        write_summary();
    }
    cout<<"Goal!"<<endl;
    cout<<"number of collision:"<<count_collision<<" , number of step:"<<count_step<<endl;
}

///////////////movement phase/////////////////
void ParticleFilter::move(double input_theta,double input_r=speed) {
    cout<<"Move with the order ("<<input_theta<<", "<<input_r<<")"<<endl;
    for(int i=0;i<N;i++) robots[i].move(input_theta,input_r);
    true_position.move(input_theta,input_r);
}

//Input average position of robot (x,y), then output steering angle with PD control
double ParticleFilter::PD_control() {
    double steering=-p_gain*CTE-d_gain*(CTE-CTE_previous);
    cout<<"CTE="<<CTE<<", steering="<<steering<<endl;
    return steering;
}

//Evaluate CTE from [x_average, y_average].
void ParticleFilter::evaluate_cross_track_error() {
    evaluate_average();
    CTE_previous=CTE;

    //specify the closest line
    for(int i=0;i<path.size()-1;i++){
        XY start=path[i];
        XY end=path[i+1];
        double delta_x=end.getX()-start.getX();
        double delta_y=end.getY()-start.getY();
        double R_x=x_average-start.getX();
        double R_y=y_average-start.getY();

        double u=(R_x*delta_x+R_y*delta_y)/(delta_x*delta_x+delta_y*delta_y);
        double x_mid=(start.getX()+end.getX())/2.0;
        double y_mid=(start.getY()+end.getY())/2.0;
        XY mid(x_mid,y_mid);
        XY true_position_XY(x_average,y_average);

        if(0<u && u<1 && mid.distance(true_position_XY)<1.0 ){   //線分の中間点から十分近い場合のみ処理を進める
            //calculate CTE for the specific line
            CTE=-(R_x*delta_y-R_y*delta_x)/(delta_x*delta_x+delta_y*delta_y);
            true_position_XY.show_nonewline();
            cout<<i<<"th line with middle point ["<<mid.getX()<<", "<<mid.getY()<<"], "<<"CTE="<<CTE<<endl;
        }
    }
}

///////////////measurement phase/////////////////
//show the average position and the angle
void ParticleFilter::evaluate_average() {
    x_average=y_average=theta_average=0;

    for(int i=0;i<N;i++){
        x_average+=robots[i].getX()/N;
        y_average+=robots[i].getY()/N;
        theta_average+=robots[i].getTheta()/N;
    }
    cout<<"average position is ["<<x_average<<", "<<y_average<<", "<<theta_average<<"]"<<endl;
    cout<<"true position is [" <<true_position.getX()<< ", " <<true_position.getY()
        << ", "<<true_position.getTheta()<< "]" <<endl;
}

//update the expected result of measurement based on true position
XY ParticleFilter::sense() {
    double x=true_position.getX()+Gaussian_random(measurement_noise);
    double y=true_position.getY()+Gaussian_random(measurement_noise);

    XY position_measured(x,y);
    return position_measured;
}

//evaluate weight for each particle based on measurement result and expected value of measurement
void ParticleFilter::evaluate_weight(XY position_measured) {
    cout<<"evaluate weight"<<endl;
    double x_measured=position_measured.getX();
    double y_measured=position_measured.getY();
    for(int n=0;n<N;n++) {
        double error_x=robots[n].getX()-x_measured;
        double error_y=robots[n].getY()-y_measured;
        double weight=exp(- (error_x * error_x + error_y * error_y) / (2.0* measurement_noise * measurement_noise));
        robots[n].setWeight(weight);

        //for debug
/*        if(weight>0.8){
            cout<<"weight:"<<weight<<" position:";//<<" "<<robots[n].getWeight()<<endl;
            robots[n].show_state();
        }
        */
    }

    double offset_x=true_position.getX()-x_measured;
    double offset_y=true_position.getY()-y_measured;
    double temp=(offset_x * offset_x + offset_y * offset_y) / (2.0* measurement_noise * measurement_noise);
    true_position.setWeight(exp(-temp));
    cout<<"weight for true position particle (it should be close to 1):"<<true_position.getWeight()<<endl;
}

//resampling from the set of the particles
void ParticleFilter::resample() {
    cout<<"resampling"<<endl;

    //weightsにrobot N個それぞれの重みを格納
    vector<double> weights(N);
    for(int i=0;i<N;i++) weights[i]=robots[i].getWeight();

    //discrete_distributionを使って重みを反映した乱数生成（乱数は整数、範囲は0からnum_robotまで）
    discrete_distribution<> distri (weights.begin(), weights.end());

    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    //resample後のparticle
    vector<Robot> resampled_particles(N);
    for(int i=0;i<N;i++) {
        int id=distri(engine);
        resampled_particles[i]=robots[id];
        if(resampled_particles[i].getWeight()>0.8){
            cout<<"weight:"<<resampled_particles[i].getWeight()<<" position:";//<<" "<<robots[n].getWeight()<<endl;
            resampled_particles[i].show_state();
        }
    }

    robots.clear();
    robots=resampled_particles;
    resampled_particles.clear();
}

///////////////////for test//////////////////
void ParticleFilter::set_particles(double x_input, double y_input, double theta_input) {
    cout<<"set particles as ["<<x_input<<", "<<y_input<<", "<<theta_input<<"]"<<endl;
    for(int i=0;i<N;i++) robots[i].set_state(x_input,y_input,theta_input);
    true_position.set_state(x_input,y_input,theta_input);
}

///////////////////interface//////////////////

void ParticleFilter::show_state_all() {
    for(int i=0;i<N;i++) robots[i].show_state();
}

void ParticleFilter::show_true_state() {
    true_position.show_state();
}

void ParticleFilter::write_particles() {
    cout<<"writing data"<<endl;

    string filename_data="particles.dat";
    ofstream fout;
    fout.open(filename_data,ios::app);

    for(int i=0;i<robots.size();i++){
        fout<< robots[i].getX()<<" "<<robots[i].getY()<<" "
            <<robots[i].getTheta() <<endl;
    }
    fout<<endl<<endl;

    fout.close();
}

void ParticleFilter::write_summary() {
    cout << "writing summary" << endl;
    string filename_data = "true_position.dat";
    ofstream fout;
    fout.open(filename_data, ios::app);
    fout << true_position.getX() << " " << true_position.getY() << " "
         << true_position.getTheta() << endl;
    fout.close();

    string filename_data2 = "average_position.dat";
    ofstream fout2;
    fout2.open(filename_data2, ios::app);
    fout2 << x_average << " " << y_average << " " << theta_average << endl;
    fout2.close();
}

////////////////setter and getter////////////
void ParticleFilter::setGrid(const vector<vector<int>> &grid) {
    ParticleFilter::grid = grid;
}
void ParticleFilter::setGoal(const IJ &goal) {
    ParticleFilter::goal = goal;
}
void ParticleFilter::setPath(const vector<XY> &path) {
    ParticleFilter::path = path;
}




