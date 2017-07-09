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


#define GoUpStairLengthEnough 80000

static double GaitGoUpStair[18*GoUpStairLengthEnough*6]; //8 steps+ 2 steps+ 8 steps

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

        FileName[0]=        "../../ServerXV/Gait/upstairTest1.txt";
        FileName[1]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[2]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[3]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[4]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";
        FileName[5]="/home/hex/Desktop/RobotIII/resource/gaits/upstair_new90000.txt";

        std::cout<<"file name:"<<FileName[0]<<std::endl;

        for(int i=0;i<1;i++)
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
            for (int j = 0; !file.eof(); j++) file >> GaitGoUpStair[j+i*GoUpStairLengthEnough];
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[0*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[1+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[2+i*GoUpStairLengthEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[3+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[4+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[5+i*GoUpStairLengthEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[6+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[7+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[8+i*GoUpStairLengthEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[9+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[10+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[11+i*GoUpStairLengthEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[12+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[13+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[14+i*GoUpStairLengthEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[15+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[16+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[17+i*GoUpStairLengthEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[18+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[19+i*GoUpStairLengthEnough]<<" "<<GaitGoUpStair[20+i*GoUpStairLengthEnough]<<std::endl;

            file.close();
            std::cout<<"GaitCountPhase"<<i<<" count:"<<GaitStairCount[i]<<std::endl;

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


void recover33Parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)
{
    Recover33Param param;

    param.if_check_pos_min = false;
    param.if_check_pos_max = false;
    param.if_check_pos_continuous = false;

    for (auto &i : params)
    {
        if (i.first == "all")
        {
            std::fill_n(param.active_leg, 6, true);
        }
        else if (i.first == "first")
        {
            param.active_leg[0] = true;
            param.active_leg[1] = false;
            param.active_leg[2] = true;
            param.active_leg[3] = false;
            param.active_leg[4] = true;
            param.active_leg[5] = false;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 0, 3, true);
            std::fill_n(param.active_motor + 6, 3, true);
            std::fill_n(param.active_motor + 12, 3, true);
        }
        else if (i.first == "second")
        {
            param.active_leg[0] = false;
            param.active_leg[1] = true;
            param.active_leg[2] = false;
            param.active_leg[3] = true;
            param.active_leg[4] = false;
            param.active_leg[5] = true;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 3, 3, true);
            std::fill_n(param.active_motor + 9, 3, true);
            std::fill_n(param.active_motor + 15, 3, true);
        }
        else if(i.first == "leg")
        {
            auto leg_id = std::stoi(i.second);

            if (leg_id<0 || leg_id>5)\
                throw std::runtime_error("invalide param in parseRecover func");

            std::fill_n(param.active_leg, 6, false);
            param.active_leg[leg_id] = true;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + leg_id * 3, 3, true);
        }
        else if (i.first == "t1")
        {
            param.recover_count = std::stoi(i.second);
        }
        else if (i.first == "t2")
        {
            param.align_count = std::stoi(i.second);
        }
        else if (i.first == "margin_offset")
        {
            param.margin_offset = std::stod(i.second);
        }
        else if (i.first == "require_zero")
        {
            printf("Zeroing function is a TODO in this robot\n");
        }
        else
        {
            printf("Param: %s\n", i.first.c_str());
            throw std::runtime_error("unknown param in parseRecover func");
        }
    }

    msg_out.copyStruct(param);
}

int recover33Gait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const Recover33Param &>(plan_param);

    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

    static double beginPin[18], beginPee[18], alignPin[18];

    if (param.count == 0)
    {
        std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
        robot.GetPee(beginPee, robot.body());

        const double pe[6]{ 0 };
        robot.SetPeb(pe);
        robot.SetPee(param.alignPee);

        robot.GetPin(alignPin);
        robot.SetPee(beginPee, robot.body());
    }

    int leftCount = param.count < param.recover_count ? 0 : param.recover_count;
    int rightCount = param.count < param.recover_count ? param.recover_count : param.recover_count + param.align_count;

    double s = -(PI / 2)*cos(PI * (param.count - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

    for (int i = 0; i < 6; ++i)
    {
        if (param.active_leg[i])
        {
            if (param.count < param.recover_count)
            {
                for (int j = 0; j < 3; ++j)
                {
                    robot.motionPool().at(i * 3 + j).setMotPos(beginPin[i * 3 + j] * (cos(s) + 1) / 2 + alignPin[i * 3 + j] * (1 - cos(s)) / 2);
                }
            }
            else
            {
                double pEE[3];
                for (int j = 0; j < 3; ++j)
                {
                    pEE[j] = param.alignPee[i * 3 + j] * (cos(s) + 1) / 2 + param.recoverPee[i * 3 + j] * (1 - cos(s)) / 2;
                }

                robot.pLegs[i]->SetPee(pEE);
            }
        }
    }

    // recover 自己做检查 //
    for (int i = 0; i<18; ++i)
    {
        if (param.active_motor[i] && (param.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN))
        {
            std::int32_t offsetCount = param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio();
            std::int32_t maxPosCount = cs.controller().motionAtAbs(i).maxPosCount();
            std::int32_t minPosCount = cs.controller().motionAtAbs(i).minPosCount();

            if (param.if_check_pos_max &&
                param.motion_raw_data->at(i).target_pos >(maxPosCount + offsetCount))
            {
                rt_printf("Motor %i's target position is bigger than its MAX permitted\
                           value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n",
                            cs.controller().motionAtAbs(i).minPosCount(),
                            cs.controller().motionAtAbs(i).maxPosCount(),
                            param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed\n");
                return 0;
            }
            if (param.if_check_pos_min &&
                param.motion_raw_data->at(i).target_pos < (minPosCount - offsetCount))
            {
                rt_printf("Motor %i's target position is smaller than its MIN permitted\
                           value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n",
                            cs.controller().motionAtAbs(i).minPosCount(),
                            cs.controller().motionAtAbs(i).maxPosCount(),
                            param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed\n");
                return 0;
            }
        }
    }

    return param.align_count + param.recover_count - param.count - 1;
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
