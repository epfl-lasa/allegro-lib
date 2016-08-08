/*
 * testPDController.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: seungsu
 */


#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include "MathLib/MathLib.h"

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"

#include "controlAllegroHand.h"
#include "sController.h"

#define JOINT_STATE_TOPIC "/allegro/joint_state"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"

MathLib::Vector curr_position(DOF_JOINTS);
MathLib::Vector curr_torque(DOF_JOINTS);
MathLib::Vector desire_position(DOF_JOINTS);
MathLib::Vector desire_velocity(DOF_JOINTS);
MathLib::Vector desire_torque(DOF_JOINTS);
double out_torque[DOF_JOINTS];


boost::mutex *mutex;
ros::Publisher joint_state_pub;
ros::Subscriber joint_cmd_sub;
sensor_msgs::JointState msgJoint;


eMotionType gMotionType = eMotionType_NONE ;

ros::Time tstart;
ros::Time tend;


void SetjointCallback(const sensor_msgs::JointState& msg)
{
	// TODO check joint limits


	// copy
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desire_position(i) = msg.position[i];
	mutex->unlock();
	//desire_position.Print();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro");
	ros::Time::init();
	ros::NodeHandle nh;

	mutex = new boost::mutex();

	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	msgJoint.position.resize(DOF_JOINTS);


	// initialize BHand controller

	ros::Rate rate(1./ALLEGRO_CONTROL_TIME_INTERVAL);
	int frame =0;

	// initialize device
	controlAllegroHand *canDevice;
	sController *lController;
	MathLib::Vector lKP(16);
	MathLib::Vector lKV(16);
	lController = new sController(16, ALLEGRO_CONTROL_TIME_INTERVAL, 1000.);

	lKP.One();
	lKP *= 0.8;

	lKV.One();
	lKV *= 0.015;
	lController->SetGain(lKP.Array(), lKV.Array());

	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000); // wait 3 milisecond
	desire_torque.Zero();
	desire_velocity.Zero();

	tstart = ros::Time::now();
	double dt;

	bool lIsBegin = false;
	while(ros::ok())
	{

		tend = ros::Time::now();
		dt = 1e-9*(tend - tstart).nsec;
		tstart = tend;
		printf("%lf ", dt);
		/*
		printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n", dt,
				desire_torque[0], desire_torque[1], desire_torque[2], desire_torque[3],
				desire_torque[4], desire_torque[5], desire_torque[6], desire_torque[7],
				desire_torque[8], desire_torque[9], desire_torque[10], desire_torque[11]	);
*/
		// device read and write
		//desire_torque.Zero();
		//for(int i=eJOINTNAME_MIDDLE_0; i<DOF_JOINTS; i++ ) desire_torque[i] = 0.0;
		//desire_torque.Zero();
		canDevice->setTorque(desire_torque.Array());
		canDevice->update();
		canDevice->getJointInfo(curr_position.Array(), curr_torque.Array());

		// compute torque
		lController->SetJointPosition( curr_position );
		if( lIsBegin == false ){
			if(frame > 10){
				mutex->lock();
				desire_position = curr_position;
				mutex->unlock();

				lIsBegin = true;
			}

			lController->Update();
			desire_torque.Zero();
		}
		else{
			mutex->lock();
			lController->SetJointDesiredPosition(desire_position, desire_velocity );
			mutex->unlock();

			// calculate torque

			lController->Update();
			lController->GetJointTorque(desire_torque);

			//desire_torque.Zero();
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.position[i] = curr_position(i);
			joint_state_pub.publish(msgJoint);
		}
		frame++;

		ros::spinOnce();
		rate.sleep();
		ros::spinOnce();
	}

	nh.shutdown();
	delete canDevice;
	printf("bye\n");

	return 0;
}

