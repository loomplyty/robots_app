#ifndef GOSTAIR_H
#define GOSTAIR_H
#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct GoStairParam final:public aris::server::GaitParamBase
{
    //const char fileName[256]{ 0 };
    std::int32_t gaitCount{ 0 };

    // double *pIn{ nullptr };
};


void parseGoUpStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
void parseGoDownStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);

int GoUpStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
int GoDownStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);


#endif // GOSTAIR_H
