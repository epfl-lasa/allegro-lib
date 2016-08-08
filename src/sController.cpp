/*
 * sController.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: seungsu
 */

#include "sController.h"

sController::sController(int dim, double dt, double wn)
{
	mDim = dim;
	mWn = wn;
	mDt = dt;

	mJointFilter = new CDDynamics(mDim, mDt, mWn);

	mKp.Resize(mDim);
	mKv.Resize(mDim);

	mTargetPosition.Resize(mDim);
	mTargetVelocity.Resize(mDim);
	mFinteredPosition.Resize(mDim);
	mFinteredVelocity.Resize(mDim);

	mOutTorque.Resize(mDim);

	mTargetVelocity.Zero();
	mOutTorque.Zero();

	mIsInitalized = false;
}

sController::~sController()
{
	delete mJointFilter;
}

void sController::SetGain(double *kp, double *kv)
{
	mKp.Set(kp, mDim);
	mKv.Set(kv, mDim);
}

void sController::SetJointPosition(MathLib::Vector& q)
{
	if( mIsInitalized == false)
	{
		mJointFilter->SetStateTarget(q, q);
		mIsInitalized = true;
	}
	else
	{
		mJointFilter->SetTarget(q);
	}
}

void sController::SetJointDesiredPosition(MathLib::Vector&  qd, MathLib::Vector&  qd_dot)
{
	mTargetPosition.Set(qd);
	mTargetVelocity.Set(qd_dot);
}

void sController::Update(void)
{
	mJointFilter->Update();
	mJointFilter->GetState(mFinteredPosition, mFinteredVelocity);
	//mJointFilter->GetTarget(mFinteredPosition);
	//mFinteredVelocity.Zero();

	// calculate torque
	for(int i=0; i<mDim; i++){
		mOutTorque(i) = (mTargetPosition(i) - mFinteredPosition(i))*mKp(i) + (mTargetVelocity(i) - mFinteredVelocity(i))*mKv(i);
		double maxT=0.65;
		if ((mOutTorque(i))>maxT)
		{
			mOutTorque(i)=maxT;
		}
		if ((mOutTorque(i))<-maxT)
		{
			mOutTorque(i)=-maxT;
		}

	}

}

void sController::GetJointTorque(MathLib::Vector& tau)
{
	tau.Set( mOutTorque );
}
