#include "GoStair.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


#define GoUpStairLengthEnough 12000

static double GaitGoUpStair[18*GoUpStairLengthEnough*6];


static int GaitStairCount[6];


static bool isFileLoaded{false};

void parseGoUpStair(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    GoStairParam param;

    if(isFileLoaded==false)
    {
        //load the txt file

        std::ifstream file;
        std::string FileName[6];// = i.second;

        FileName[0]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[1]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[2]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[3]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[4]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[5]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";

        std::cout<<"file name:"<<FileName<<std::endl;

        for(int i=0;i<6;i++)
        {
            int gaitData{-1};
            file.open(FileName[i]);
            std::cout<<"loading .txt file number "<<i<<std::endl;
            if (!file) throw std::logic_error("File does not exist");
            for (double tem; !file.eof(); file >> tem) ++gaitData;
            if (gaitData % 18 != 0) throw std::logic_error("File invalid, because data size is not valid");
            GaitStairCount[i]=gaitData /18;
            file.close();

            file.open(FileName[i]);
            for (int j = 0; !file.eof(); file >> GaitGoUpStair[j+i*GoUpStairLengthEnough]) ++j;
            file.close();
            std::cout<<"GaitCountPhase"<<i<<" :"<<GaitStairCount[i]<<std::endl;

        }
        isFileLoaded=true;
    }

    for (auto &i:params)
    {
        if(i.first=="phase")
        {
            param.phase=std::stoi(i.second);
            param.gaitCount=GaitStairCount[param.phase];
        }
        else
        {
            throw std::logic_error("internal error happened, because invalid params in parseFastWalk");
        }
    }

    msg.copyStruct(param);

}


int GoUpStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<const GoStairParam &>(param_in);

    if(pSP.count==0)
    {
        std::cout<<"GoStair phase " <<pSP.phase<<" begins..."<<" gait length is: "<<pSP.gaitCount<<std::endl;
    }

    if(pSP.count < pSP.gaitCount)
    {
        robot.SetPin(&GaitGoUpStair[pSP.count*18+pSP.phase*GoUpStairLengthEnough]);
    }
    return pSP.gaitCount-pSP.count-1;
}

void parseGoStair33Init(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    aris::server::GaitParamBase param;
    msg.copyStruct(param);
}
int GoStair33Init(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);

//    static aris::dynamic::FloatMarker beginMak{robot.ground()};
//    static double beginPee[18];
//    static double beginPeb[6];
//    if (param.count == 0)
//    {
//        beginMak.setPrtPm(*robot.body().pm());
//        beginMak.update();
//        robot.GetPee(beginPee, beginMak);
//       // robot.GetPeb(beginPeb, beginMak);
//    }

//    int totalCount = param.totalCount;
//    int n = param.helloTimes;
//    const double sign = param.isForward ? 1 : -1;
//    double footDist = std::sqrt(std::pow(beginPee[2], 2) + std::pow(beginPee[1] - param.bodyUp, 2));
//    double theta = std::atan2(std::fabs(beginPee[2]), std::fabs(beginPee[1]) + param.bodyUp);

}


//void  parseGoDownStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
//{
//    GoStairParam param;
//    param.gaitCount=GaitStairCount;
//    msg.copyStruct(param);
//}

//int  GoDownStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
//{

//    auto &robot = static_cast<Robots::RobotBase &>(model);
//    auto &pSP=static_cast<const GoStair::GoStairParam &>(param_in);
//    if(pSP.count==0)
//    {
//        cout<<"go down stair begins"<<"gait length:"<<pSP.gaitCount<<endl;
//        cout<<"pin value"<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[1]<<" "<<Gait_GoUpStair[17]<<endl;
//    }


//    if(pSP.count < pSP.gaitCount)
//    {
//        robot.SetPin(&Gait_GoUpStair[(pSP.gaitCount-pSP.count-1)*18]);
//    }

//    return pSP.gaitCount-pSP.count-1;

//}
