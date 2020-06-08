/*
 System: instances a friction wedge 2D dynamical system
 Copyright (C) 2013  Leonardo Baruffaldi leobart@fem.unicamp.br

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "system.h"
#include "mbsim/functions/composite_function.h"

using namespace fmatvec;

// next variables control the type of simulation
#define ALTERNATED_MOTION
#define MOVING_MASS

class Motion : public MBSim::Function<Vec3(double)> {
public:
	Motion(double _omegaInHertz, double _amp, double _tdelay, double _period, int _dof):omega(_omegaInHertz * 2 * M_PI),
	amp(_amp),delay(_tdelay),period(_period),dof(_dof){}
	int getArgSize() const {return 3;}

	Vec3 operator()(const double &t) {
		Vec3 r;
		r(dof) = 0.5 * amp * ( 1 - cos(om(t)) );
		return r;
	}
	Vec3 parDer(const double &t) {
		Vec3 jh;
		jh(dof) =  0.5 * amp * (sin(om(t))*omega);
		return jh;
	}
	Vec3 parDerParDer(const double &t) {
		Vec3 jb;
		jb(dof) = 0.5 * amp * (cos(om(t))*omega*omega);
		return jb;
	}
private:
	double om(const double &t) {
		double timeDependentArgument;
		if(t <= delay || t> delay + period) timeDependentArgument = 0.;
		else timeDependentArgument = omega * (t - delay);
		return timeDependentArgument;
	}
	double omega;
	double amp;
	double delay;
	double period;
	double dof;
};

class Angle : public MBSim::Function<double(double)> {
public:
	Angle(double _omegaInHertz, double _amp, double _tdelay):omega(_omegaInHertz * 2 * M_PI),amp(_amp),delay(_tdelay){}
	int getArgSize() const {return 3;}
	double operator()(const double& t) {
		double al;
	    al = amp * cos(om(t));
	      return al;
	    }
	    double parDer(const double& t) {
	      double angSp;
	      angSp = - amp * sin(om(t)) * omega;
	      return angSp;
	    }
	    double parDerParDer(const double& t) {
	      double angAc;
	      angAc = amp * cos(om(t)) * omega * omega;
	      return angAc;
	    }
private:
	double om(const double &t) {
		double timeDependentArgument;
		if(t <= delay) timeDependentArgument = 0.;
		else timeDependentArgument = omega * (t - delay) + M_PI;
		return timeDependentArgument;
	}
	double omega;
	double amp;
	double delay;
};

System::System(const string& projectName, const string& inputFileName) :
    						DynamicSystemSolver(projectName)
{
	/// --------------- INITIALIZATION ------------------------------------------
	if (initializeFromFile(inputFileName))
	{
		std::cout << "Error while initializing from file. File not found!"
				<< std::endl;
	}

	/// --------------- PARAMETERS ----------------------------------------------

	// set the acceleration of gravity
	getMBSimEnvironment()->setAccelerationOfGravity(Vec("[0;-9.81;0]"));

	this->getFrame("I")->enableOpenMBV();

	/// ----------------- TRUCKS ------------------------------------------------
	Vec3 posit(3, INIT, 0.0);
	posit(0) = truckBaseDistance / 2;
	BarberTruck *frontTruck = new BarberTruck("Front truck", bolsterBushing, truckWheelBase);
	frontTruck->addFrame(new FixedRelativeFrame("FT_RefFrame",posit,
			SqrMat(3,EYE)));
	frontTruck->setFrameOfReference(frontTruck->getFrame("FT_RefFrame"));
	this->addGroup(frontTruck);

	posit(0) = -truckBaseDistance / 2;
	BarberTruck *rearTruck = new BarberTruck("Rear truck", bolsterBushing, truckWheelBase);
	rearTruck->addFrame(new FixedRelativeFrame("RT_RefFrame",posit,
			SqrMat(3,EYE)));
	rearTruck->setFrameOfReference(rearTruck->getFrame("RT_RefFrame"));
	this->addGroup(rearTruck);


	//
	//


	/// ----------------------------- RAIL ------------------------------------------
	///

	posit(0) = 0;
	posit(1) = -0.672;
	posit(2) = - (0.8 + 0.03681); //0.03681 corrects the gauge distance

	RailProfile *leftRail = new RailProfile("Left rail profile","tr68.dat");
	addFrame(new FixedRelativeFrame("Left rail frame",posit,"[0,0,1;0,1,0;1,0,0]"));
	leftRail->setFrameOfReference(getFrame("Left rail frame"));
	leftRail->enableOpenMBV();
	addContour(leftRail);

	posit(2) = - posit(2);

	RailProfile *rightRail = new RailProfile("Right rail profile","tr68.dat");
	addFrame(new FixedRelativeFrame("Right rail frame",posit,"[0,0,-1;0,1,0;-1,0,0"));
	rightRail->setFrameOfReference(getFrame("Right rail frame"));
	rightRail->enableOpenMBV();
	addContour(rightRail);

	/// --------------------------- WAGONBOX ----------------------------------------
	///
	WagonSimple *wagon = new WagonSimple("Wagonbox");
	wagon->setTotalMass(wagonMass);
	wagon->setInertiaTensor(wagonInertiaTensor);

	posit(0) = 0.0;
	posit(1) = 1.3;
	posit(2) = 0.0;
	wagon->addFrame(new FixedRelativeFrame("WB_RefFrame",posit,
			SqrMat(3,EYE)));
	wagon->setFrameOfReference(wagon->getFrame("WB_RefFrame"));
	wagon->setFrontBolsterConnectionPosition(truckBaseDistance/2,-1.10118,0);
	wagon->setRearBolsterConnectionPosition(-truckBaseDistance/2,-1.10118,0);
	wagon->setLength(8.6);
	wagon->setHeight(1.48);
	wagon->setWidth(2.6);
	wagon->enableOpenMBV(true);


	this->addGroup(wagon);


	/// ----------- Center plates ---------------------------
	SymMatV centerPlateStiffness(6,INIT,0.0);
	centerPlateStiffness(0,0) = 175.130e6; // longitudinal
	centerPlateStiffness(2,2) = 175.130e6; // lateral
	centerPlateStiffness(1,1) = 875.634e6; // vertical
	centerPlateStiffness(3,3) = 875.634e6 * 0.33 / 4; // roll
	centerPlateStiffness(5,5) = 875.634e6 * 0.33 / 4; // pitch

	SymMatV centerPlateDamping(6,INIT,0.0);
	centerPlateDamping = centerPlateStiffness * 0.004;
	centerPlateDamping(4,4) = 700e3;

	ElasticJoint *frontPlate = new ElasticJoint("Front connection plate");
	frontPlate->setForceDirection("[1,0,0;0,1,0;0,0,1]");
	frontPlate->setMomentDirection("[1,0,0;0,1,0;0,0,1]");
	frontPlate->setGeneralizedForceFunction(
			new LinearElasticFunction(centerPlateStiffness,centerPlateDamping));
	frontPlate->connect(frontTruck->bolster->getFrame("WCP"),
			wagon->getFrontConnectionFrame());
	addLink(frontPlate);



	ElasticJoint *rearPlate = new ElasticJoint("Rear connection plate");
	rearPlate->setForceDirection("[1,0,0;0,1,0;0,0,1]");
	rearPlate->setMomentDirection("[1,0,0;0,1,0;0,0,1]");
	rearPlate->setGeneralizedForceFunction(
			new LinearElasticFunction(centerPlateStiffness,0.004*centerPlateStiffness));
	rearPlate->connect(rearTruck->bolster->getFrame("WCP"),
			wagon->getRearConnectionFrame());
	addLink(rearPlate);


	/// -------------- WHEEL-RAIL CONTACT --------------------------------------

	Contact* wheelRail = new Contact("Wheel Rail");
	wheelRail->connect(leftRail,frontTruck->wheelFront->getLeftWheel());
	wheelRail->setNormalForceLaw(new UnilateralConstraint);
	addLink(wheelRail);

//	ContactObserver *observer = new ContactObserver(std::string("Wheel rail contact"));
//	observer->setContact(wheelRail);
//	observer->enableOpenMBVContactPoints(_size=0.05);
//	observer->enableOpenMBVNormalForce(_size=0.05);
//	observer->enableOpenMBVTangentialForce(_size=0.05);
//	addObserver(observer);

	wheelRail->setPlotFeature(generalizedForce,true);
	wheelRail->setPlotFeature(generalizedRelativePosition,true);

	/// -------------- MOTION DEFINITION ----------------------------------------

	double tSpeedMeterPerSec = trainSpeed / 3.6;
	double period = 5/tSpeedMeterPerSec;
	if (tfinal == 0) period = 1e3;
	else period = tfinal;

	// front wheel, front truck = wheel 1
//	frontTruck->wheelFront->setTranslation(new Motion(freq,amplitude,t0,
//			period,2));
	frontTruck->wheelFront->setTranslation(new LinearTranslation<VecV> ("[1,0,0;0,1,0;0,0,1]"));
	// rear wheel, front truck = wheel2
	frontTruck->wheelRear->setTranslation(new Motion(freq,amplitude,
			t0+truckWheelBase/tSpeedMeterPerSec,
			period,2));
	// front wheel, rear truck = wheel3
	rearTruck->wheelFront->setTranslation(new Motion(freq,amplitude,
			t0+(truckBaseDistance)/tSpeedMeterPerSec,
			period,2));
	// rear wheek, rear truck = wheel4
	rearTruck->wheelRear->setTranslation(new Motion(freq,amplitude,
			t0+(truckBaseDistance+truckWheelBase)/tSpeedMeterPerSec,
			period,2));

	setPlotFeatureRecursive(generalizedPosition,true);
	setPlotFeatureRecursive(generalizedVelocity,true);
	setPlotFeatureRecursive(generalizedForce,true);
}

int
System::initializeFromFile(const string& inputFileName)
{
	amplitude = atof(searchParameter(inputFileName, "AMPLITUDE").c_str());

	bolsterBushing = atof(
				searchParameter(inputFileName, "BOLSTER_BUSHINGS").c_str());

	fillRatio = atof(searchParameter(inputFileName, "FILL_RATIO").c_str());

	freq = atof(searchParameter(inputFileName, "FREQUENCY").c_str());

	wagonMass = atof(searchParameter(inputFileName, "BOX_MASS").c_str());

	wagonInertiaTensor.resize(3);
	wagonInertiaTensor.init(fmatvec::EYE);
	wagonInertiaTensor(0, 0) = atof(
			searchParameter(inputFileName, "BOX_Iroll").c_str());
	wagonInertiaTensor(1, 1) = atof(
			searchParameter(inputFileName, "BOX_Iyaw").c_str());
	wagonInertiaTensor(2, 2) = atof(
			searchParameter(inputFileName, "BOX_Ipitch").c_str());

	t0 = atof(searchParameter(inputFileName, "MOVEMENT_DELAY").c_str());

	tfinal = atof(searchParameter(inputFileName, "MOVEMENT_TEND").c_str());

	trainSpeed = atof(searchParameter(inputFileName, "INITIAL_TRAIN_VELOCITY").c_str());

	truckBaseDistance = atof(
			searchParameter(inputFileName, "CAR_WHEEL_BASE").c_str());

	truckWheelBase = atof(
			searchParameter(inputFileName, "TRUCK_WHEEL_BASE").c_str());

	return 0;
}
