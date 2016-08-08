/*
 * sController.h
 *
 *  Created on: Nov 20, 2012
 *      Author: seungsu
 */

#ifndef SCONTROLLER_H_
#define SCONTROLLER_H_

#include "CDDynamics.h"
#include "MathLib.h"

class sController
{
public:
	sController(int dim, double dt, double wn);
	~sController();

	void SetGain(double *kp, double *kv);
	void SetJointPosition(MathLib::Vector& q);
	void SetJointDesiredPosition(MathLib::Vector&  qd, MathLib::Vector&  qd_dot);
	void Update(void);
	void GetJointTorque(MathLib::Vector& tau);
private:

	int mDim;
	int mWn;
	double mDt;

	bool mIsInitalized;

	MathLib::Vector mKp;
	MathLib::Vector mKv;

	MathLib::Vector mTargetPosition;
	MathLib::Vector mTargetVelocity;
	MathLib::Vector mFinteredPosition;
	MathLib::Vector mFinteredVelocity;

	MathLib::Vector mOutTorque;

	CDDynamics *mJointFilter;

};


#endif /* SCONTROLLER_H_ */
