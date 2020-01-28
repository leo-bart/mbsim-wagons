/*
 * contact_utils.cc
 * Copyright (C) 2019 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 *  Created on: 9 de mai de 2019
 *      Author: leonardo
 *	   Contact: <e-mail>
 */

/*
 * This is a modification of the contact_utils.cc original file to include
 * wheel-rail contact
 */

#include <config.h>
#include "mbsim/utils/contact_utils.h"
#include "stdio.h"

// --- List of contact kinematic implementations - BEGIN ---
#include <mbsim/contact_kinematics/circle_frustum.h>
#include <mbsim/contact_kinematics/circle_circle.h>
#include <mbsim/contact_kinematics/circle_planarcontour.h>
#include <mbsim/contact_kinematics/circle_planarfrustum.h>
#include <mbsim/contact_kinematics/circle_line.h>
#include <mbsim/contact_kinematics/circle_linesegment.h>
#include <mbsim/contact_kinematics/circle_plane.h>
#include <mbsim/contact_kinematics/compoundcontour_compoundcontour.h>
#include <mbsim/contact_kinematics/compoundcontour_contour.h>
#include <mbsim/contact_kinematics/edge_edge.h>
#include <mbsim/contact_kinematics/line_planarcontour.h>
#include <mbsim/contact_kinematics/point_plate.h>
#include <mbsim/contact_kinematics/point_planarcontour.h>
#include <mbsim/contact_kinematics/point_contourinterpolation.h>
#include <mbsim/contact_kinematics/point_frustum.h>
#include <mbsim/contact_kinematics/point_line.h>
#include <mbsim/contact_kinematics/point_circle.h>
#include <mbsim/contact_kinematics/point_plane.h>
#include <mbsim/contact_kinematics/point_sphere.h>
#include <mbsim/contact_kinematics/point_planewithfrustum.h>
#include <mbsim/contact_kinematics/point_linesegment.h>
#include <mbsim/contact_kinematics/sphere_frustum.h>
#include <mbsim/contact_kinematics/sphere_plane.h>
#include <mbsim/contact_kinematics/sphere_plate.h>
#include <mbsim/contact_kinematics/sphere_polynomialfrustum.h>
#include <mbsim/contact_kinematics/sphere_sphere.h>
#include <mbsim/contact_kinematics/plate_polynomialfrustum.h>
#include <mbsim/contact_kinematics/point_polynomialfrustum.h>
#include <mbsim/contact_kinematics/point_spatialcontour.h>

#include "wheel_rail.h"
// --- List of contact kinematic implementations - END ---

namespace MBSim {

  double computeAngleOnUnitCircle(const fmatvec::Vec3& r) {
    return r(1)>=0 ? acos(r(0)) : 2*M_PI-acos(r(0));
  }

  fmatvec::Vec2 computeAnglesOnUnitSphere(const fmatvec::Vec3& r) {
    fmatvec::Vec2 zeta(fmatvec::NONINIT);
    double l = sqrt(r(0)*r(0) + r(1)*r(1));
    zeta(0)= r(1)>=0 ? acos(r(0)/l) : 2*M_PI-acos(r(0)/l);
    zeta(1)= asin(r(2));
    return zeta;
  }

  ContactKinematics* findContactPairingRigidRigid(const char* contour0, const char* contour1) {

    if(( strcmp(contour0, "Circle")==0 && strcmp(contour1, "Frustum")==0 ) ||
       ( strcmp(contour0, "Circle")==0 && strcmp(contour1, "Frustum")==0 ) )
      return new ContactKinematicsCircleFrustum;

    else if ( strcmp(contour0, "Circle")==0 && strcmp(contour1, "Circle")==0 )
      return new ContactKinematicsCircleCircle;

    else if ( strcmp(contour0, "Circle")==0 && strcmp(contour1, "PlanarContour")==0 )
      return new ContactKinematicsCirclePlanarContour;

    else if ( strcmp(contour0, "Line")==0 && strcmp(contour1, "PlanarContour")==0 )
      return new ContactKinematicsLinePlanarContour;

    else if ( strcmp(contour0, "Circle")==0 && strcmp(contour1, "PlanarFrustum")==0 )
      return new ContactKinematicsCirclePlanarFrustum;

    else if ( strcmp(contour0, "Circle")==0 && strcmp(contour1, "Line")==0 )
      return new ContactKinematicsCircleLine;

    else if ( strcmp(contour0, "Circle")==0 && strcmp(contour1, "LineSegment")==0 )
      return new ContactKinematicsCircleLineSegment;

    else if ( strcmp(contour0, "Circle")==0 && strcmp(contour1, "Plane")==0 )
      return new ContactKinematicsCirclePlane;

    else if (( strcmp(contour0, "Cuboid")==0 && strcmp(contour1, "Plane")==0 ) or
        ( strcmp(contour0, "Room")==0 && strcmp(contour1, "Point")==0 ) or
        ( strcmp(contour0, "Cuboid")==0 && strcmp(contour1, "Frustum")==0 ))
      return new ContactKinematicsCompoundContourContour;

    else if (( strcmp(contour0, "Cuboid")==0 && strcmp(contour1, "Room")==0 ) or
        ( strcmp(contour0, "Cuboid")==0 && strcmp(contour1, "Cuboid")==0 ))
      return new ContactKinematicsCompoundContourCompoundContour;

    /*
     *else if ( strcmp(contour0, "CompoundContour")==0 )
     *  if ( strcmp(contour1, "CompoundContour")==0 )
     *    return new ContactKinematicsCompoundContourCompoundContour;
     *  else
     *    return new ContactKinematicsCompoundContourContour;
     */

    else if ( strcmp(contour0, "Edge")==0 && strcmp(contour1, "Edge")==0 )
      return new ContactKinematicsEdgeEdge;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Plate")==0 )
      return new ContactKinematicsPointPlate;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "ContourInterpolation")==0 )
      return new ContactKinematicsPointContourInterpolation;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Frustum")==0 )
      return new ContactKinematicsPointFrustum;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "PolynomialFrustum")==0 )
      return new ContactKinematicsPointPolynomialFrustum;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Line")==0 )
      return new ContactKinematicsPointLine;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Circle")==0 )
      return new ContactKinematicsPointCircle;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Plane")==0 )
      return new ContactKinematicsPointPlane;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Sphere")==0 )
      return new ContactKinematicsPointSphere;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "PlaneWithFrustum")==0 )
      return new ContactKinematicsPointPlaneWithFrustum;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "LineSegment")==0 )
      return new ContactKinematicsPointLineSegment;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "PlanarContour")==0 )
      return new ContactKinematicsPointPlanarContour;

    else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "SpatialContour")==0 )
      return new ContactKinematicsPointSpatialContour;

    else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "Frustum")==0 )
      return new ContactKinematicsSphereFrustum;

    else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "Plane")==0 )
      return new ContactKinematicsSpherePlane;

    else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "Plate")==0 )
      return new ContactKinematicsSpherePlate;

    else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "PolynomialFrustum")==0 )
      return new ContactKinematicsSpherePolynomialFrustum;

    else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "Sphere")==0 )
      return new ContactKinematicsSphereSphere;

    else if ( strcmp(contour0, "Plate")==0 && strcmp(contour1, "PolynomialFrustum")==0 )
      return new ContactKinematicsPlatePolynomialFrustum;

    else if ( strcmp(contour0, "Wheel Profile Contour")==0 && strcmp(contour1, "Rail Profile Contour")==0 )
          return new ContactKinematicsWheelRail;

    else
      return 0;
  }
}



