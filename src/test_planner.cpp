/*
 * main.cpp
 *
 *  Created on: Nov 9, 2012
 *      Author: seungsu
 */
#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"
#include "CDDynamics.h"
#include "MathLib.h"
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>

#define JOINT_STATE_TOPIC "/allegro/joint_state"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"


//rosrt::Publisher<sensor_msgs::JointState> *pub_handJoint;
//ros::Subscriber<sensor_msgs::JointState> sub_handJoint;
ros::Subscriber sub_handJoint;
ros::Publisher pub_handJoint;
CDDynamics *mFingerDyn;

sensor_msgs::JointState curr_joint_state;

boost::thread *reading_thread;
boost::thread *testcontroller_thread;

ros::Time tstart;
ros::Time tend;
double jntcmd[4][4];
double velcmd[4][4];

bool gIsPlannerINIT = false;
sensor_msgs::JointState msgJoint;

//double service_data_close[]={0.0, 60.0, 55.0, 30.0, 0.0, 60.0, 55.0, 30.0, 0.0, 60.0, 55.0, 30.0, 80.0, 45.0, 0.0, 70.0};
double service_data_open[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 88.0, 45.0, 0.0, 00.0};

double service_data_close[]={0.0, 90.0, 70.0, 30.0, 0.0, 90.0, 70.0, 30.0, 0.0, 90.0, 70.0, 30.0, 90.0, 45.0, 0.0, 70.0};
//double service_data_close[]={0.277385, 0.390469, 0.562012, 0.728427,  0.00000, 0.000000, 0.812502, 0.525780,  -0.277385, 0.390469, 0.762012, 0.728427,  1.475599, 0.452160, 1.48503, 0.0};
//double service_data_close[]={0.0, 60.0, 48.0, 23.0, 0.0, 60.0, 48.0, 23.0, 0.0, 60.0, 48.0, 23.0, 70.0, 45.0, 0.0, 70.0};
//double service_data_open[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 00.0};


void chatterCallback(const sensor_msgs::JointState& msg)
{
	int i;
	double dt;
	MathLib::Vector pos(16), vel(16), target(16);

	tend = ros::Time::now();
	dt  = 1e-9*(tend - tstart).nsec;
	tstart = tend;

	for(i=0; i<4; i++){
		jntcmd[i][0] = msg.position[i*4+0];
		jntcmd[i][1] = msg.position[i*4+1];
		jntcmd[i][2] = msg.position[i*4+2];
		jntcmd[i][3] = msg.position[i*4+3];
	}

	if( gIsPlannerINIT == false){
		for(int j=0; j<16; j++) pos(j) = msg.position[j];
		mFingerDyn->SetStateTarget(pos, pos);

		//for(int j=0; j<16; j++) target(j) = service_data_close[j];
		//mFingerDyn->SetTarget(target);

		gIsPlannerINIT = true;
	}

	if( gIsPlannerINIT == true){
		mFingerDyn->Update();
		mFingerDyn->GetState(pos, vel);


		for(i=0; i<16; i++){
			msgJoint.position[i] = pos(i);
			msgJoint.velocity[i] = vel(i);
		}

		pub_handJoint.publish(msgJoint);

	}

	i = 0;
/*
	printf("%6.4f : %06.4lf %06.4lf %06.4lf %06.4lf, %06.4lf %06.4lf %06.4lf %06.4lf \n",
			dt,
			jntcmd[i][0], jntcmd[i][1], jntcmd[i][2], jntcmd[i][3],
			velcmd[i][0], velcmd[i][1], velcmd[i][2], velcmd[i][3]);
*/
}


void CommandThread(void)
{
	unsigned char cmd[1024];
	MathLib::Vector target(16);

	while(cmd[0] != 'q'){
		printf("==============\n");
		printf("h : home \n");
		printf("c : catch \n");
		printf("==============\n");
		scanf("%c", &cmd);



		if( cmd[0] == 'h'){
			for(int j=0; j<16; j++) target(j) = service_data_open[j] *M_PI/180.;
			mFingerDyn->SetTarget(target);
		}
		else if( cmd[0] == 'c'){
			//for(int j=0; j<16; j++) target(j) = service_data_close[j] *M_PI/180.;
			for(int j=0; j<16; j++) target(j) = service_data_close[j]*M_PI/180.;
			mFingerDyn->SetTarget(target);
		}
	}


}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "allegro_test");
	ros::NodeHandle nh;
	//rosrt::init();


	msgJoint.position.resize(4*4);
	msgJoint.velocity.resize(4*4);
	mFingerDyn = new CDDynamics(4*4, 0.003, 10.0);

	tstart = ros::Time::now();

	sub_handJoint = nh.subscribe(JOINT_STATE_TOPIC, 3, chatterCallback);
	pub_handJoint = nh.advertise<sensor_msgs::JointState>(JOINT_CMD_TOPIC, 3);

	boost::thread *command_thread;
	command_thread = new boost::thread(&CommandThread);

	ros::spin();
	reading_thread->join();
	testcontroller_thread->join();

	return 0;
}
