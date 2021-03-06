#ifndef JOINT_FACTORY_H
#define JOINT_FACTORY_H


#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"

using namespace MBSim;

class RotaryJoint : public Joint
{
public:
	RotaryJoint(const std::string& name, Frame* firstConnectFrame, Frame* secondConnectFrame) : Joint(name){

		this->setForceDirection("[1,0,0;0,1,0;0,0,1]");
		this->setForceLaw(new  BilateralConstraint());
		this->setMomentDirection("[1,0;0,0;0,1]");
		this->setMomentLaw(new BilateralConstraint());
		this->connect(firstConnectFrame,secondConnectFrame);
	};
};

class SphericalJoint : public Joint
{
public:
	SphericalJoint(const std::string& name, Frame* firstConnectFrame, Frame* secondConnectFrame) : Joint(name){

		this->setForceDirection("[1,0,0;0,1,0;0,0,1]");
		this->setForceLaw(new  BilateralConstraint());
		this->connect(firstConnectFrame,secondConnectFrame);
	};
};

class RigidJoint : public Joint
{
public:
	RigidJoint(const std::string& name, Frame* firstConnectFrame, Frame* secondConnectFrame) : Joint(name){

		this->setForceDirection("[1,0,0;0,1,0;0,0,1]");
		this->setForceLaw(new  BilateralConstraint());
		this->setMomentDirection("[1,0,0;0,1,0;0,0,1]");
		this->setMomentLaw(new BilateralConstraint());
		this->connect(firstConnectFrame,secondConnectFrame);
	};
};

#endif
