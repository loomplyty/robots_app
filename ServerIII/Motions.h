#ifndef MOTIONS_H
#define MOTIONS_H

//#include <aris_control_pipe.h>
#include <aris.h>


#include <thread>
#include <functional>
#include <cstdint>
#include <map>
#include <Robot_Base.h>
#include <Robot_Gait.h>

using namespace std;

struct MoveRotateParam final :public aris::server::GaitParamBase
{
    double targetBodyPE213[6]{0};
    std::int32_t totalCount;
};
void parseMoveWithRotate(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int moveWithRotate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

namespace GoStair
{



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

}


#endif
