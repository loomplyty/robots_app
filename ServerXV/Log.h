//
// Created by tianyuan on 18-7-2.
//

#ifndef ROBOT_APP_LOG_H
#define ROBOT_APP_LOG_H
#include "aris.h"

struct robotData {
    //double bodyPee[6];
    //double legPee[18];
    //double waist;
    //double imu[3];
    double force[18];
    //int legPhase[6];
};

void startLogDataThread();

#endif //ROBOT_APP_LOG_H
