//
// Created by 研究用 on 2017/09/06.
//

#ifndef LESSON6_A_SUMMARY_GIJ_H
#define LESSON6_A_SUMMARY_GIJ_H

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


class GIJ: public IJ {
private:
    int value;

public:
    int getValue() const;
    void show();
    void setValue(int value);
    void set_GIJ(int,int,int);

};


#endif //LESSON6_A_SUMMARY_GIJ_H
