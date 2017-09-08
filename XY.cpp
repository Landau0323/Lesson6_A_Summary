//
// Created by 研究用 on 2017/09/06.
//

#include "XY.h"

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

void XY::setX(double x) {
    XY::x = x;
}

void XY::setY(double y) {
    XY::y = y;
}

void XY::setXY(double X, double Y) {
    x=X;
    y=Y;
}

double XY::getX() const {
    return x;
}

double XY::getY() const {
    return y;
}

double XY::distance(XY other) {
    return sqrt((x-other.x)*(x-other.x)+(y-other.y)*(y-other.y));
}

void XY::show_nonewline() {
    cout<<"["<<x<<", "<<y<<"]";
}

XY::XY(double x, double y) : x(x), y(y) {}

XY::XY() {}

