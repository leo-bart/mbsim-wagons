/*
 * wheelset.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: leonardo
 */

#include "wheelset.h"

Wheelset::Wheelset(const std::string& name) : Wheelset(name, 1.0, 1.0)
{
}

Wheelset::Wheelset(const std::string& name,
		double _track, double _sfTrack) : MBSim::RigidBody(name), track(_track), sideframeTrack(_sfTrack)
{
	fmatvec::Vec3 pos(fmatvec::INIT,0.0);
	fmatvec::SqrMat3 rot (fmatvec::INIT,0.0);

	double ajusteBitola = -0.097; // TODO gambiarra para acertar a posição das rodas

	// rot = rotation matrix
	rot(0,0) = 1;
	rot(1,2) = 1;
	rot(2,1) = -1;

	// sideframe frames of connection
	pos(2) = - this->sideframeTrack / 2;
	this->addFrame ( new MBSim::FixedRelativeFrame ( "SFL",pos,fmatvec::SqrMat ( 3,fmatvec::EYE ) ) );

	pos(2) = -pos(2);
	this->addFrame ( new MBSim::FixedRelativeFrame ( "SFR",pos,fmatvec::SqrMat ( 3,fmatvec::EYE ) ) );

	// wheel contour reference frames
	pos(2) = -this->track / 2 - ajusteBitola;
	this->addFrame ( new MBSim::FixedRelativeFrame ( "WL",pos,rot ) );

	pos(2) = -pos(2);
	rot(1,2) = -1;
	rot(2,1) = 1;
	this->addFrame ( new MBSim::FixedRelativeFrame ( "WR",pos,rot ) );


	//3D visualization
//	std::shared_ptr<OpenMBV::IvBody> cad=OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
//	cad->setIvFileName("wrl/wheelset.wrl");
//	cad->setScaleFactor(1.0);
//	cad->setInitialRotation(3*M_PI/2,0,M_PI/2);
//	cad->setInitialTranslation(0,0.0,0);
//	this->setOpenMBVRigidBody(cad);


	wheelRight = new MBSim::WheelProfile("Roda direita","roda.dat",getFrame("WR"));
	wheelRight->enableOpenMBV();

	wheelLeft = new MBSim::WheelProfile("Roda esquerda","roda.dat",getFrame("WL"));
	wheelLeft->enableOpenMBV();

	addContour(wheelRight);
	addContour(wheelLeft);

}

void Wheelset::enableOpenMBV()
{
	std::shared_ptr<std::vector<std::shared_ptr<OpenMBV::PolygonPoint>>> points =
					std::make_shared<std::vector<std::shared_ptr<OpenMBV::PolygonPoint>>>();
  points->push_back(OpenMBV::PolygonPoint::create(.0,-0.067,0.));
  points->push_back(OpenMBV::PolygonPoint::create(.4344,-0.067,0.));
  points->push_back(OpenMBV::PolygonPoint::create(.4572,0.0,0.));
  points->push_back(OpenMBV::PolygonPoint::create(.4593,0.043,0.));
  points->push_back(OpenMBV::PolygonPoint::create(.4824,0.043,0.));
  points->push_back(OpenMBV::PolygonPoint::create(.4824,0.077,0.));
  points->push_back(OpenMBV::PolygonPoint::create(.0,0.077,0.));

  std::shared_ptr<OpenMBV::Rotation> openMBVwheel1 =
  				OpenMBV::ObjectFactory::create<OpenMBV::Rotation>();
  openMBVwheel1->setName("Roda 1");;
  openMBVwheel1->setContour(points);
  openMBVwheel1->setInitialRotation(M_PI/2,0.,0.);
  openMBVwheel1->setInitialTranslation(0.,0.,track/2);
//
  std::shared_ptr<OpenMBV::Rotation> openMBVwheel2 =
		  OpenMBV::ObjectFactory::create<OpenMBV::Rotation>();
  openMBVwheel1->setName("Roda 2");
  openMBVwheel2->setContour(points);
  openMBVwheel2->setInitialRotation(3*M_PI/2,0.,0.);
  openMBVwheel2->setInitialTranslation(0.,0.,-track/2);

  std::shared_ptr<OpenMBV::Frustum> openMBVshaft =
		  OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  openMBVshaft->setName("Eixo");
  openMBVshaft->setBaseRadius(0.125);
  openMBVshaft->setTopRadius(0.125);
  openMBVshaft->setHeight(track);
  openMBVshaft->setInitialTranslation(0,0,track/2.0);
  openMBVshaft->setDiffuseColor(0.25,0.25,0.25);
//
  std::shared_ptr<OpenMBV::CompoundRigidBody> openMBVwheelset =
		  OpenMBV::ObjectFactory::create<OpenMBV::CompoundRigidBody>();
  openMBVwheelset->addRigidBody(openMBVwheel1);
  openMBVwheelset->addRigidBody(openMBVwheel2);
  openMBVwheelset->addRigidBody(openMBVshaft);
//
  this->setOpenMBVRigidBody(openMBVshaft);
}
