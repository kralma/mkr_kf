/**
 * Kalman Filter 
 * Skeleton code for teaching 
 * A3M33MKR
 * Czech Technical University 
 * Faculty of Electrical Engineering
 * Intelligent and Mobile Robotics group
 *
 * Authors: Zdeněk Kasl, Karel Košnar kosnar@labe.felk.cvut.cz
 *
 * this code is inspired by tutorial on Kalman Filter by Greg Czerniak 
 *
 * Licence: MIT (see LICENSE file)
 **/

#include<cmath>
#include<cassert>
#include<cstdlib>
#include<fstream>
#include<iostream>
#include <Eigen/Dense>

#include "gui/gui.h"
#include "systemSimulator/system.h"
using namespace imr;
using namespace gui;
using namespace Eigen;

Point KalmanFilter(Point measuredPosition);

//    float v0 = 149;
//    float alpha = (float) (M_PI / 4 - 0.007);
    float v0 = 150;
    float alpha = (float) (M_PI / 4);
    float dt = 0.1;
    float g = (float) (9.8 * dt);

    Matrix4f A,R,Sigma;
    Matrix<float,4,1> B;
    Matrix2f Q;
    Matrix<float,2,4> C;
    Vector4f X;



void help(char** argv)
{
    std::cout << "\nUsage of the program " << argv[0]+2 << std::endl
         << "Parameter [-h or -H] displays this message." <<std::endl
	 << " Parameter [-n or -N] number of simulation steps."
         << std::endl;
}

int main(int argc, char** argv)
{
    A <<    1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;
    B <<    0,
            0,
            0,
            -1;
    C <<    1,0,0,0,
            0,1,0,0;
    X <<   0,
            0,
            cos(alpha) * v0,
            sin(alpha) * v0;
    R <<    3,0,0,0,
            0,3,0,0,
            0,0,3,0,
            0,0,0,3;
    Q <<    50,0,
            0,50;
    Sigma << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    int nSteps = 1000;
    char *dataFile;

    // Parse all console parameters
    for (int i=0; i<argc; i++) {
        if (argv[i][0]=='-') {
            switch(argv[i][1]) {
                //> HELP
                case 'H' : case 'h' :
                    help(argv);
                    break;
		case 'N': case 'n':
                    assert(i+1 < argc);
                    assert(atoi(argv[i+1])>1);
                    nSteps = atoi(argv[i+1]);
		    break;
                default :
                    std::cout << "Parameter \033[1;31m" << argv[i] << "\033[0m is not valid!\n"
                        << "Use parameter -h or -H for help." << std::endl;
                    break;
            }
        }
    }
    // All parameters parsed

    Gui gui;
    System system;
    
    //> comment line below in order to let the program
    //> continue right away
//    gui.startInteractor();

    Point measurement;
    Point truth;
    Point kfPosition;

    for(int i=1; i<nSteps; i++) {
       system.makeStep();
       truth = system.getTruthPosition();
       measurement = system.getMeasurement();
       kfPosition = KalmanFilter(measurement);
       gui.setPoints(truth, measurement, kfPosition);

       //> comment line below in order to let the program
       //> continue right away
       if (i%40 == 0) gui.startInteractor();
    
    }
    gui.startInteractor();

    return EXIT_SUCCESS;
}


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                               implement Kalman Filter here
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


Point  KalmanFilter(const Point measuredPosition)
{
    Point ret;
    Vector2f Z;
    Z <<    measuredPosition.x,
            measuredPosition.y;
    Matrix<float,4,1> input = g * B;
    Vector4f X_ = A * X + input;
    Matrix4f Sigma_ = A * Sigma * A.transpose() + R;
    Matrix<float,4,2> K =  Sigma_ * C.transpose() * (C * Sigma_ * C.transpose() + Q).inverse();
    X = X_ + 1*K * (Z - C * X_);

    ret.x = X(0);
    ret.y = X(1);
    return ret;
}

/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



