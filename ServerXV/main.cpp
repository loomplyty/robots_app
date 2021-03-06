﻿#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#include "swing.h"
#include "move_body.h"
#include "twist_waist.h"
#include "say_hello.h"

//ty's gait
#include "GoStair.h"
#include "ForceTest.h"
#include "Log.h"
#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
	std::string xml_address;

	if (argc <= 1)
	{
		std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
		xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_XII/Robot_XII.xml";
	}
	else if (std::string(argv[1]) == "XII")
	{
		xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_XII/Robot_XII.xml";
	}
	else if (std::string(argv[1]) == "FH")
	{
		xml_address = "../../Server/RobotFaulhaber.xml";
	}
	else if (std::string(argv[1]) == "CP")
	{
		xml_address = "../../Server/RobotCopley.xml";
	}
	else if (std::string(argv[1]) == "EL")
	{
		xml_address = "../../Server/RobotElmo.xml";
	}
	else if (std::string(argv[1]) == "EDU")
	{
		xml_address = "../../Server/RobotEDU.xml";
	}
	else if (std::string(argv[1]) == "EDU2")
	{
        xml_address = "../../ServerXV/RobotEDU2.xml";
	}
	else
	{
		throw std::runtime_error("invalid robot name, please type in XII");
	}
        startLogDataThread();
	auto &rs = aris::server::ControlServer::instance();

	rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
	rs.addCmd("en", Robots::basicParse, nullptr);
	rs.addCmd("ds", Robots::basicParse, nullptr);
	rs.addCmd("hm", Robots::basicParse, nullptr);
	rs.addCmd("zrc", Robots::basicParse, nullptr);

	rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
	rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
	rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);
	rs.addCmd("mb", moveBodyParse, moveBodyGait);
	rs.addCmd("sw", swingParse, swingGait);
	rs.addCmd("tw", twistWaistParse, twistWaistGait);
	rs.addCmd("sh", sayHelloParse, sayHelloGait);

    rs.addCmd("gs",parseGoStair,GoStair);
    rs.addCmd("rc33", recover33Parse, recover33Gait);
    rs.addCmd("rcsmall", recoverSmallParse, recoverSmallGait);
    rs.addCmd("ft",ForceTestParse,ForceTestGait);

	rs.open();

	rs.setOnExit([&]() 
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.LoadFile(xml_address.c_str());
		auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
		if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
		rs.model().saveXml(*model_xml_ele);
		
		aris::core::stopMsgLoop();
	});
	aris::core::runMsgLoop();
	
	return 0;
}
