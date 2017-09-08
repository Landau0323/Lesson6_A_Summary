//
// Created by 研究用 on 2017/08/17.
//


#include <random>
#include <iostream>
#include "parameter.h"

using namespace std;

double uniform_random(double upper=1, double lower=0);
double Gaussian_random(double sigma=1);



//デフォルトでは上限と下限は1と0
double uniform_random(double upper, double lower){
    /*
    //一様乱数生成 (1-100)
    std::random_device rnd; //1からRNDMAXまでの整数をランダムに生成
//cout<<rnd()<<endl;
//    int rand=1+(rnd() % 100);   //1から100までの整数に変換
//cout << rand << endl;
    std::uniform_int_distribution<> rand100(0, 100);        // [0, 100] 範囲の一様乱数
    double rand_uniform=(double)rand100(rnd)/100.0;    //一様乱数 (0.0-1.0)

    //上限と下限に合わせてスケール変換・平行移動
    double size=upper-lower;
    rand_uniform*=size;
    rand_uniform+=lower;
     */

    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<> dist(lower, upper);

    return dist(engine);
}

double Gaussian_random(double sigma){

    double rand_uniform=uniform_random();    //一様乱数 (0.0-1.0)

    //指数乱数に変換 (lambda=1/2)
    double lambda=0.5;
    double rsquare=-(log(rand_uniform))/lambda;
    if(rand_uniform==0){ //上の一様乱数では頻繁にrand_uniform=0となるので、rsquareがinfになってしまう。
        cout<<"zero!"<<endl;
    }

    //ガウシアンに変換（Box-Muller法）
    double theta=uniform_random(2.0*PI,0); //一様乱数 (0.0-2*pi)
    double random=sigma*sqrt(rsquare)*sin(theta);

    return random;
}
