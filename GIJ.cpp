//
// Created by 研究用 on 2017/09/06.
//

#include "GIJ.h"

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

int GIJ::getValue() const {
    return value;
}

void GIJ::setValue(int value) {
    GIJ::value = value;
}

void GIJ::set_GIJ(int G, int I, int J) {
    value=G;
    i=I;
    j=J;
}

void GIJ::show() {
    cout<<"[g-value="<<value<<", i="<<i<<", j="<<j<<"]"<<endl;
}
