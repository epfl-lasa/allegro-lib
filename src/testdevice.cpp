/*
 * testdevice.cpp
 *
 *  Created on: Nov 15, 2012
 *      Author: seungsu
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include "ros/ros.h"

#include "MathLib.h"

#include "BHand/BHand.h"
#include "controlAllegroHand.h"


int main(int argc, char** argv)
{

	ros::init(argc, argv, "allegro");
	ros::Time::init();


	double torque[16] = {0.0};
	MathLib::Vector pos(16);
	double curr_torque[16] = {0.0};
	controlAllegroHand *canDevice;

	canDevice = new controlAllegroHand();
	canDevice->init();

	ros::Rate r(1000./5.);
	int frame =0;


	while(frame<200*10)
	{
		canDevice->setTorque(torque);
		canDevice->update();
		canDevice->getJointInfo(pos.Array(), curr_torque);


		pos.Print();

		r.sleep();
		frame++;
	}


	delete canDevice;
	printf("bye\n");

	return 0;
}
