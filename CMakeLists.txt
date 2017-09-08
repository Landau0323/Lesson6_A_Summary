cmake_minimum_required(VERSION 3.8)
project(Lesson6_A_Summary)

set(CMAKE_CXX_STANDARD 17)

set(SOURCE_FILES main.cpp random.cpp Plan.cpp Plan.h ParticleFilter.cpp ParticleFilter.h Robot.cpp Robot.h parameter.h IJ.cpp IJ.h GIJ.cpp GIJ.h XY.cpp XY.h)
add_executable(Lesson6_A_Summary ${SOURCE_FILES})