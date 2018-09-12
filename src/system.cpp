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
	Motion(double _omegaInHertz, double _amp, double _tdelay, int _dof):omega(_omegaInHertz * 2 * M_PI),
	amp(_amp/2),delay(_tdelay),dof(_dof){}
	int getArgSize() const {return 3;}
	Vec3 operator()(const double &t) {
		Vec3 r;
		r(dof) = amp * ( cos(om(t)) - 1 );
		return r;
	}
	Vec3 parDer(const double &t) {
		Vec3 jh;
		jh(dof) =  amp * (-sin(om(t))*omega);
		return jh;
	}
	Vec3 parDerParDer(const double &t) {
		Vec3 jb;
		jb(dof) =  amp * (cos(om(t))*omega*omega);
		return jb;
	}
private:
	double om(const double &t) {
		double timeDependentArgument;
		if(t <= delay) timeDependentArgument = 0.;
		else timeDependentArgument = omega * (t - delay);
		return timeDependentArgument;
	}
	double omega;
	double amp;
	double delay;
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
	MBSimEnvironment::getInstance()->setAccelerationOfGravity(Vec("[0;-9.81;0]"));

	this->getFrame("I")->enableOpenMBV();

	//	/// ----------------- TRUCKS ------------------------------------------------
	Vec3 position(3, INIT, 0.0);
	position(0) = truckBaseDistance / 2;
	BarberTruck *frontTruck = new BarberTruck("Front truck", bolsterBushing, truckWheelBase);
	frontTruck->addFrame(new FixedRelativeFrame("FT_RefFrame",position,
			SqrMat(3,EYE)));
	frontTruck->setFrameOfReference(frontTruck->getFrame("FT_RefFrame"));
	this->addGroup(frontTruck);

	position(0) = -truckBaseDistance / 2;
	BarberTruck *rearTruck = new BarberTruck("Rear truck", bolsterBushing, truckWheelBase);
	rearTruck->addFrame(new FixedRelativeFrame("RT_RefFrame",position,
			SqrMat(3,EYE)));
	rearTruck->setFrameOfReference(rearTruck->getFrame("RT_RefFrame"));
	this->addGroup(rearTruck);
	//
	//
	//	/// -------------- MOTION DEFINITION ----------------------------------------

	double tSpeedMeterPerSec = trainSpeed / 3.6;
	// front wheel, front truck = wheel 1
	frontTruck->wheelFront->setTranslation(new Motion(freq,2*amplitude,t0,2));
	// rear wheel, front truck = wheel2
	frontTruck->wheelRear->setTranslation(new Motion(freq,2*amplitude,
			t0+truckWheelBase/tSpeedMeterPerSec,2));
	// front wheel, rear truck = wheel3
	rearTruck->wheelFront->setTranslation(new Motion(freq,2*amplitude,
			t0+(truckBaseDistance-truckWheelBase)/tSpeedMeterPerSec,2));
	// rear wheek, rear truck = wheel4
	rearTruck->wheelRear->setTranslation(new Motion(freq,2*amplitude,
			t0+(truckBaseDistance-truckWheelBase)/tSpeedMeterPerSec,2));




	// ---------------------- Support plates ---------------------------
	// next plates are used to apply vertical displacements to the wheelsets
//	RigidBody *plateAA = new RigidBody("Plate AA");
//	Vec3 r;
//	r(0) = -truckBaseDistance/2 - truckWheelBase/2;
//	r(2) = 1.575;
//	r(1) = -0.00001;
//	addFrame(new FixedRelativeFrame("AA",r,BasicRotAIKz(M_PI/2)));
//	plateAA->setFrameOfReference(getFrame("AA"));
//	getFrame("AA")->enableOpenMBV();
//	addObject(plateAA);
//	Plate *plateAAcontour = new Plate("Plate AA contour");
//	plateAAcontour->enableOpenMBV();
//	plateAA->addContour(plateAAcontour);
//	plateAA->setTranslation(new Motion(freq,2*amplitude,t0,1));
//
//	Point *pt = new Point("PP");
//	pt->setFrameOfReference(rearTruck->wheelRear->getFrame("SFR"));
//	pt->enableOpenMBV();
//	rearTruck->wheelRear->addContour(pt);
//
//	Contact *newContact = new Contact("Novo contato");
//	newContact->connect(plateAAcontour,pt);
//	newContact->setNormalForceLaw(new UnilateralConstraint());
//	newContact->setNormalImpactLaw(new UnilateralNewtonImpact(0));
//	newContact->setPlotFeature("generalizedForce",enabled);
//	newContact->setPlotFeature("generalizedRelativePosition",enabled);
//	addLink(newContact);


	setPlotFeatureRecursive("generalizedPosition",enabled);
	setPlotFeatureRecursive("position",enabled);
	setPlotFeatureRecursive("generalizedVelocity",enabled);
	setPlotFeatureRecursive("generalizedForce",enabled);
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
			searchParameter(inputFileName, "BOX_Ixx").c_str());
	wagonInertiaTensor(1, 1) = atof(
			searchParameter(inputFileName, "BOX_Iyy").c_str());
	wagonInertiaTensor(2, 2) = atof(
			searchParameter(inputFileName, "BOX_Izz").c_str());

	t0 = atof(searchParameter(inputFileName, "MOVEMENT_DELAY").c_str());

	trainSpeed = atof(searchParameter(inputFileName, "INITIAL_TRAIN_VELOCITY").c_str());

	truckBaseDistance = atof(
			searchParameter(inputFileName, "CAR_WHEEL_BASE").c_str());

	truckWheelBase = atof(
			searchParameter(inputFileName, "TRUCK_WHEEL_BASE").c_str());

	return 0;
}
