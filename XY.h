//
// Created by 研究用 on 2017/09/06.
//

#ifndef LESSON6_A_SUMMARY_XY_H
#define LESSON6_A_SUMMARY_XY_H

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


class XY {
private:
    double x,y;

public:
    XY();
    XY(double x, double y);

    void setX(double x);
    void setY(double y);
    void setXY(double,double);
    double getX() const;
    double getY() const;

    double distance(XY other);
    void show_nonewline();
};


#endif //LESSON6_A_SUMMARY_XY_H
