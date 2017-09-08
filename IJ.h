//
// Created by 研究用 on 2017/09/06.
//

#ifndef LESSON6_A_SUMMARY_IJ_H
#define LESSON6_A_SUMMARY_IJ_H

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


class IJ {
protected:
    int i,j;

public:
    IJ();
    IJ(int i, int j);

    void show();

    int getI() const;
    int getJ() const;

    void setI(int i_in);
    void setJ(int j_in);
    void setIJ(int,int);

};


#endif //LESSON6_A_SUMMARY_IJ_H
