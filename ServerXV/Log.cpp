//
// Created by tianyuan on 18-7-2.
//
#include "Log.h"
//#include "aris.h"
# include  <unistd.h>

aris::control::Pipe<robotData> logPipe(true);

void startLogDataThread()
{
    static auto logThread = std::thread([]()
                                        {
                                            sleep(1);
                                            //aris::core::logFileNames();

                                            std::time_t beginTime{};
                                            time(&beginTime);

                                            auto timeinfo = localtime(&beginTime);
                                            char timeCh[1024]={0};

                                            strftime(timeCh,1024,"_%Y-%m-%d_%H-%M-%S_log.txt",timeinfo);
                                            std::string filename = "data" + std::string(timeCh);

                                            static std::ofstream file;
                                            file.open(filename);
                                            std::cout << "Sleep for a second waiting for something to init\n";


                                            while(true)
                                            {

                                                robotData data;
                                                logPipe.recvInNrt(data);

                                                file << std::setprecision(15);
                                                for (int j=0;j<18;j++)
                                                {
                                                    file<<data.force[j]<<"  ";
                                                }
//                                                file<<data.waist<<" ";
//
//                                                for (int j = 0; j < 3; j++)
//                                                {
//                                                    file << data.imu[j] << "   ";
//                                                }
//
//                                                for (int j = 0; j < 6; j++)
//                                                {
//                                                    file << data.bodyPee[j] << "   ";
//                                                }
//
//                                                for (int j = 0; j < 18; j++)
//                                                {
//                                                    file << data.legPee[j] << "   ";
//                                                }
//                                                for (int j = 0; j < 6; j++)
//                                                {
//                                                    file << data.force[j] << "   ";
//                                                }
//                                                for (int j = 0; j < 6; j++)
//                                                {
//                                                    file << data.legPhase[j] << "   ";
//                                                }

                                                file << std::endl;

                                            }
                                            file.close();
                                        });
}
