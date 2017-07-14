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
    std::int32_t gaitCount{ 0 };
    int phase{0};
    int UpOrDown{0};
 };


void parseGoStair(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

int GoStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

struct Recover33Param final :public aris::server::GaitParamBase
{
    std::int32_t recover_count{ 3000 };
    std::int32_t align_count{ 3000 };
    bool active_leg[6]{ true,true,true,true,true,true };
    double margin_offset{0.01};//meter
    double alignPee[18]
    {  -0.20,   -0.55,   -0.3464,
       -0.40,   -0.55,    0,
       -0.20,   -0.55,    0.3464,
        0.20,   -0.55,   -0.3464,
        0.40,   -0.55,    0,
        0.20,   -0.55,    0.3464 };

//    {  -0.30,   -0.50,   -0.52,
//       -0.60,   -0.50,    0,
//       -0.30,   -0.50,    0.52,
//        0.30,   -0.50,   -0.52,
//        0.60,   -0.50,    0,
//        0.30,   -0.50,    0.52 };

    double recoverPee[18]
        {  -0.1892625,   -0.58,   -0.327812266,
           -0.5789739,   -0.58,   -0.102812266,
           -0.2004489,   -0.58,    0.552812266,
            0.2004489,   -0.58,   -0.552812266,
            0.5789739,   -0.58,    0.102812266,
            0.1892625,   -0.58,    0.327812266 };
//        {  -0.1893,   -0.58,   -0.3278,
//           -0.5357,   -0.58,   -0.1278,
//           -0.1571,   -0.58,    0.5278,
//            0.1571,   -0.58,   -0.5278,
//            0.5357,   -0.58,    0.1278,
//            0.1893,   -0.58,    0.3278 };

    };
struct RecoverSmallParam final :public aris::server::GaitParamBase
{
    std::int32_t recover_count{ 3000 };
    std::int32_t align_count{ 3000 };
    bool active_leg[6]{ true,true,true,true,true,true };
    double margin_offset{0.01};//meter
    double alignPee[18]
    {  -0.20,   -0.55,   -0.3464,
       -0.40,   -0.55,    0,
       -0.20,   -0.55,    0.3464,
        0.20,   -0.55,   -0.3464,
        0.40,   -0.55,    0,
        0.20,   -0.55,    0.3464 };

    double recoverPee[18]
    {  -0.20,   -0.58,   -0.3464,
       -0.40,   -0.58,    0,
       -0.20,   -0.58,    0.3464,
        0.20,   -0.58,   -0.3464,
        0.40,   -0.58,    0,
        0.20,   -0.58,    0.3464 };
    };
auto recover33Parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
auto recover33Gait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int;
auto recoverSmallParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
auto recoverSmallGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int;

#endif // GOSTAIR_H
