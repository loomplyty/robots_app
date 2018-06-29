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


#define GoStairDataEnough 1800000
#define Nphase 2

static double GaitGoUpStair[GoStairDataEnough*Nphase]; //2 types of upstairs

static int GaitStairCount[Nphase];

static bool isFileLoaded{false};

void parseGoStair(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    GoStairParam param;

    if(isFileLoaded==false)
    {
        //load the txt file

        std::ifstream file;
        std::string FileName[Nphase];// = i.second;

        FileName[0]= "../../ServerXV/Gait/upstairPhaseSep.txt";
        FileName[1]= "../../ServerXV/Gait/upstairPhaseSep.txt";

        std::cout<<"file name 1:"<<FileName[0]<<std::endl;
        std::cout<<"file name 2:"<<FileName[1]<<std::endl;

        for(int i=0;i<Nphase;i++)
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
            for (int j = 0; !file.eof(); j++) file >> GaitGoUpStair[j+i*GoStairDataEnough];
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[0*GoStairDataEnough]<<" "<<GaitGoUpStair[1+i*GoStairDataEnough]<<" "<<GaitGoUpStair[2+i*GoStairDataEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[3+i*GoStairDataEnough]<<" "<<GaitGoUpStair[4+i*GoStairDataEnough]<<" "<<GaitGoUpStair[5+i*GoStairDataEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[6+i*GoStairDataEnough]<<" "<<GaitGoUpStair[7+i*GoStairDataEnough]<<" "<<GaitGoUpStair[8+i*GoStairDataEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[9+i*GoStairDataEnough]<<" "<<GaitGoUpStair[10+i*GoStairDataEnough]<<" "<<GaitGoUpStair[11+i*GoStairDataEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[12+i*GoStairDataEnough]<<" "<<GaitGoUpStair[13+i*GoStairDataEnough]<<" "<<GaitGoUpStair[14+i*GoStairDataEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[15+i*GoStairDataEnough]<<" "<<GaitGoUpStair[16+i*GoStairDataEnough]<<" "<<GaitGoUpStair[17+i*GoStairDataEnough]<<std::endl;
            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[18+i*GoStairDataEnough]<<" "<<GaitGoUpStair[19+i*GoStairDataEnough]<<" "<<GaitGoUpStair[20+i*GoStairDataEnough]<<std::endl;

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
        else if(i.first=="upordown")
        {
            param.UpOrDown=std::stoi(i.second);
        }
        else
        {
            throw std::logic_error("internal error happened, because invalid params in parseFastWalk");
        }
    }

    msg.copyStruct(param);
}


int GoStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<const GoStairParam &>(param_in);

    if(pSP.count==0)
    {
        std::cout<<"GoStair phase " <<pSP.phase<<" begins..."<<" gait length is: "<<pSP.gaitCount<<std::endl;

    }

    if(pSP.count < pSP.gaitCount)
    {
        if(pSP.UpOrDown==0)
        {
//            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[1+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[2+pSP.count*18+pSP.phase*GoStairDataEnough]<<std::endl;
//            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[3+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[4+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[5+pSP.count*18+pSP.phase*GoStairDataEnough]<<std::endl;
//            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[6+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[7+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[8+pSP.count*18+pSP.phase*GoStairDataEnough]<<std::endl;
//            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[9+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[10+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[11+pSP.count*18+pSP.phase*GoStairDataEnough]<<std::endl;
//            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[12+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[13+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[14+pSP.count*18+pSP.phase*GoStairDataEnough]<<std::endl;
//            std::cout<<"GaitGoUpStair: "<<GaitGoUpStair[15+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[16+pSP.count*18+pSP.phase*GoStairDataEnough]<<" "<<GaitGoUpStair[17+pSP.count*18+pSP.phase*GoStairDataEnough]<<std::endl;
            robot.SetPin(&GaitGoUpStair[pSP.count*18+pSP.phase*GoStairDataEnough]);


        }
        else if(pSP.UpOrDown ==1)
        {
            static int map[6]{0,1,2,3,4,5};
            static double pin[18];
            for (int i=0;i<6;i++)
            {
                memcpy(&pin[map[i]*3],&GaitGoUpStair[(pSP.gaitCount-pSP.count-1)*18+i*3+pSP.phase*GoStairDataEnough],sizeof(double)*3);
            }
            robot.SetPin(pin);
        }
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


void recoverSmallParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)
{
    RecoverSmallParam param;

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

int recoverSmallGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)
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
