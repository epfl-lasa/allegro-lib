/*
 * allegroNodeS.cpp
 *
 *  Created on: Jan 6, 2013
 *      Author: seungsu
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include "MathLib.h"

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"

#include "sController.h"
#include "controlAllegroHand.h"

#define JOINT_STATE_TOPIC "/allegro/joint_state"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"

MathLib::Vector curr_position(DOF_JOINTS);
MathLib::Vector curr_torque(DOF_JOINTS);
MathLib::Vector desire_position(DOF_JOINTS);
MathLib::Vector desire_velocity(DOF_JOINTS);
MathLib::Vector desire_torque(DOF_JOINTS);
double out_torque[DOF_JOINTS];


//boost::mutex *mutex;
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
	//mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desire_position(i) = msg.position[i];
	for(int i=0;i<DOF_JOINTS;i++) desire_velocity(i) = msg.velocity[i];

	//mutex->unlock();
	//desire_position.Print();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro");
	ros::Time::init();
	ros::NodeHandle nh;
	bool lEmergencyStop = false;

	//mutex = new boost::mutex();

	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	msgJoint.position.resize(DOF_JOINTS);
	msgJoint.header.seq = 0;

	// initialize BHand controller
	//BHand lBHand(eHandType_Left);
	//lBHand.SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
	sController *lController;
	MathLib::Vector lKP(16);
	MathLib::Vector lKV(16);
	lController = new sController(16, ALLEGRO_CONTROL_TIME_INTERVAL, 200.);

	lKP.One();
	lKP *= 2.80; // 2.25
	lKP(eJOINTNAME_INDEX_0 ) = 1.0;
	lKP(eJOINTNAME_MIDDLE_0) = 1.0;
	lKP(eJOINTNAME_PINKY_0 ) = 1.0;
	lKP(eJOINTNAME_THUMB_0 ) = 1.0;
	lKP(eJOINTNAME_THUMB_1 ) = 1.0;

	lKV.One();
	lKV *= 0.15;// 0.15
	//lKV(eJOINTNAME_INDEX_0) = 0.14;
	//lKV(eJOINTNAME_THUMB_0) = 0.14;
	lController->SetGain(lKP.Array(), lKV.Array());


	ros::Rate rate(1./ALLEGRO_CONTROL_TIME_INTERVAL);
	int frame =0;

	// initialize device
	controlAllegroHand *canDevice;
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);
	desire_torque.Zero();

	tstart = ros::Time::now();
	double dt;

	bool lIsBegin = false;
	FILE *fid;
	fid = fopen("./allegro_log.txt", "w+");

	while(ros::ok())
	{

		tend = ros::Time::now();
		dt = 1e-9*(tend - tstart).nsec;
		tstart = tend;

		//desire_torque *= 1.3;
		//desire_torque.Zero();
		//for(int i=eJOINTNAME_MIDDLE_0; i<DOF_JOINTS; i++ ) desire_torque[i] = 0.0;

		canDevice->_readDevices();
		lEmergencyStop = canDevice->mEmergencyStop;
		canDevice->getJointInfo(curr_position.Array(), curr_torque.Array());
/*
		curr_position(0)  +=   0.0*M_PI/180.;
		curr_position(1)  +=  5.0*M_PI/180.;
		curr_position(2)  += 15.0*M_PI/180.;
		curr_position(3)  +=   0.0*M_PI/180.;
		curr_position(4)  +=  -10.0*M_PI/180.;
		curr_position(5)  +=   -5.0*M_PI/180.;
		curr_position(6)  +=  -10.0*M_PI/180.;
		curr_position(7)  +=   0.0*M_PI/180.;
		curr_position(8)  +=  50.0*M_PI/180.;
		curr_position(9)  += 0.0*M_PI/180.;
		curr_position(10) += 0.0*M_PI/180.;
		curr_position(11) += 0.0*M_PI/180.;
		curr_position(12) += 0.0*M_PI/180.;
		curr_position(13) += 0.0*M_PI/180.;
		curr_position(14) += 0.0*M_PI/180.;
		curr_position(15) += 0.0*M_PI/180.;

*/

		for(int i=0; i<16; i++ ) fprintf(fid, "%lf ", desire_position[i]);
		for(int i=0; i<16; i++ ) fprintf(fid, "%lf ", curr_position[i]);
		for(int i=0; i<16; i++ ) fprintf(fid, "%lf ", desire_torque[i]);
		fprintf(fid, "\n");

		//printf("%lf ", dt );

//		printf(": %lf, %lf, %lf, %lf,  %lf, %lf, %lf, %lf,  %lf, %lf, %lf, %lf,  %lf, %lf, %lf, %lf, : %lf \n",
//				curr_position[0], curr_position[1], curr_position[2], curr_position[3],
//				curr_position[4], curr_position[5], curr_position[6], curr_position[7],
//				curr_position[8], curr_position[9], curr_position[10], curr_position[11],
//				curr_position[12], curr_position[13], curr_position[14], curr_position[15],
//				dt);

		if( lEmergencyStop == true )
		{
			cout << "break " << endl;
			desire_torque.Print("desire_torque");
			break;
		}

		int ID=desire_torque.MaxId();
		cout<<"ID "<<desire_torque(ID)<<endl;
		// compute torque
		lController->SetJointPosition( curr_position );
		if( lIsBegin == false ){
			if(frame > 10){
				//mutex->lock();
				desire_position = curr_position;
				//mutex->unlock();

				lIsBegin = true;
			}

			lController->Update();
			desire_torque.Zero();
		}
		else{
			//mutex->lock();
			//lBHand.SetJointDesiredPosition(desire_position.Array());
			lController->SetJointDesiredPosition(desire_position, desire_velocity );
			//mutex->unlock();

			lController->Update();
			lController->GetJointTorque(desire_torque);

			for(int i=0; i<DOF_JOINTS; i++) msgJoint.position[i] = curr_position(i);
			joint_state_pub.publish(msgJoint);
		}
		frame++;
		msgJoint.header.seq++;
		msgJoint.header.stamp = ros::Time::now();
		//desire_torque *= 3.0;

//		desire_torque /= 5.0;
//		for(int i=0; i<4; i++) desire_torque(i+ 4) = 0.0;
//		for(int i=0; i<4; i++) desire_torque(i+ 8) = 0.0;
//		for(int i=0; i<4; i++) desire_torque(i+12) = 0.0;

		canDevice->setTorque(desire_torque.Array());
		canDevice->_writeDevices();

		//ros::spinOnce();
		//rate.sleep();
		ros::spinOnce();
	}

	fclose(fid);
	nh.shutdown();
	delete canDevice;
	printf("bye\n");

	return 0;
}


