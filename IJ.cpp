//
// Created by 研究用 on 2017/09/06.
//

#include "IJ.h"

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

int IJ::getI() const {
    return i;
}

int IJ::getJ() const {
    return j;
}

void IJ::setI(int i_in) {
    IJ::i = i_in;
}

void IJ::setJ(int j_in) {
    IJ::j = j_in;
}

IJ::IJ(int i, int j) : i(i), j(j) {}

IJ::IJ() {
}

void IJ::setIJ(int i_in, int j_in) {
    i=i_in;
    j=j_in;
}

void IJ::show() {
    cout<<"[i="<<i<<", j="<<j<<"]"<<endl;

}
