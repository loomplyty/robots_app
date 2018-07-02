#include "ForceTest.h"
#include "Log.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"

#include "Log.h"

#endif

extern aris::control::Pipe<robotData> logPipe;

auto ForceTestParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    aris::server::GaitParamBase param;
    msg.copyStruct(param);
}
auto ForceTestGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const aris::server::GaitParamBase &>(param_in);
    static robotData data;

    if(param.count%250 ==0)
        for(int i=0;i<6;i++)
        {
      //  rt_printf("Fz%f ",param.ruicong_data->
            rt_printf("Fz%f ",param.ruicong_data->at(0).force[i].Fz);
            rt_printf("Fx%f ",param.ruicong_data->at(0).force[i].Fx);
            rt_printf("Fy%f ",param.ruicong_data->at(0).force[i].Fy);
            memcpy(&data.force[i*3],&param.ruicong_data->at(0).force[i].Fx,sizeof(double));
            memcpy(&data.force[i*3+1],&param.ruicong_data->at(0).force[i].Fy,sizeof(double));
            memcpy(&data.force[i*3+2],&param.ruicong_data->at(0).force[i].Fz,sizeof(double));
            logPipe.sendToNrt(data);
        }

//    double euler[3];
//    param.imu_data->toEulBody2Ground(euler,"213");
//    rt_printf("euler yaw %f,pitch %f,roll %f\n",euler[0],euler[1],euler[2]);

    rt_printf("\n");
    rt_printf("\n");


    return 1000 - param.count - 1;

}
