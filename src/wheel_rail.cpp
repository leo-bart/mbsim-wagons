/*
 * wheel_rail.cpp
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
 *  Created on: 29 de mar de 2019
 *      Author: Leonardo Baruffaldi
 *	   Contact: leonardo.baruffaldi@ifsp.edu.br
 */

#include  "wheel_rail.h"

namespace MBSim{

ContactKinematicsWheelRail::~ContactKinematicsWheelRail() {



}

void ContactKinematicsWheelRail::assignContours(
		const std::vector<Contour*>& contour) {
	if(dynamic_cast<RailProfile*>(contour[0])){
		irail = 0;
		iwheel = 1;
		rail = static_cast<RailProfile*>(contour[0]);
		wheel = static_cast<WheelProfile*>(contour[1]);
	} else {
		irail = 1;
		iwheel = 0;
		rail = static_cast<RailProfile*>(contour[1]);
		wheel = static_cast<WheelProfile*>(contour[0]);
	}
}

void ContactKinematicsWheelRail::updateg(double& g,
		std::vector<ContourFrame*>& cFrame, int index) {
	/* TODO implement GJK distance algorithm
	 * 1. Calcular ponto de contato
	 * 1.1. Determinar posição global do perfil da roda
	 * 1.2. Determinar posição global do perfil do trilho
	 * 1.3. Rodar algoritmo de localização do ponto de contato
	 * 2. Calcular normal
	 * 3. Atualizar os cFrames dos contornos para serem compatíveis com os pontos de contato
	*/

	// Parte 1: determinação do ponto de contato roda-trilho
	// Parte 1.1.: posição global do perfil da roda
	Vec3 rWheel = wheel->getFrame()->evalPosition();
	/// MatVx2 wheelProfilePoints = wheel->getPoints();

	// Parte 1.2.: posição global do perfil do trilho
	/*
	 * estabelecer a posição de um frame em relação à
	 * linha média do trilho no local mais próximo ao
	 * centro de massa do rodeiro
	 *
	 */
	Vec3 rRail = rail->getFrame()->evalPosition();

	cFrame[iwheel]->setPosition(rWheel);
	cFrame[irail]->setPosition(rRail);
	cFrame[iwheel]->enableOpenMBV();

	findContactPoints(cFrame);

	g = 0.003;

}

void ContactKinematicsWheelRail::updatewb(Vec& wb, double g,
		std::vector<ContourFrame*>& cFrame) {
}

/**
 * Implementation of the improved search method from Gino van den Bergen's
 * paper on the fast implementation of the GJK distance algorithm
 * G. V. den Bergen, “A Fast and Robust GJK Implementation for Collision Detection
 * of Convex Objects”, Journal of Graphics Tools, vol. 4, nº 2, p. 7–25, jan. 1999.
 */
void ContactKinematicsWheelRail::findContactPoints(std::vector<ContourFrame*>& cFrame) {
	/* CHECK THE SMALLEST DISTANCE BETWEEN WHEEL AND RAIL
	 * as a preliminary adjustment, we separate the convex sets on the wheel and
	 * rail to accelerate the contact calculation
	 */

	/**
	 * \brief a matrix containig the sets of initial and final
	 * points of the convex segments of the profile
	 */

	// TODO: move this code as a function under ProfileContour
	 MatVx2 wheelConvex;
	 MatVx2 railConvex;

	 wheelConvex = wheel->getConvexSetIndexes();
	 railConvex = rail->getConvexSetIndexes();

	 double g = gjkDistance(wheelConvex,railConvex,0.002,false);

	 std::cout << g << std::endl;

//	 std::cout << wheelConvex << std::endl;
//	 std::cout << railConvex << std::endl;


	/* we start with a coarse check by numbering the segments of the wheel and rail pro-
	 * files and checking their proximity
	 */

//	double mindist = 1e6;
//	int noderail, nodewheel;
//	bool madeContact = false;
//
//	noderail = 0;
//	nodewheel = 0;



}

/**
 * Calculation of the Minkowski sum of two polyhedra A and B
 */
MatVx2 ContactKinematicsWheelRail::minkowskiSum(MatVx2 A,
		MatVx2 B){


	unsigned k = A.rows();
	unsigned p = B.rows();
	int c = 0;
	MatVx2 mSum(k*p,0);

	for (unsigned i=0;i<p;i++){
		for (unsigned j=0;j<k;j++){
			mSum(c,0) = -A(j,0) + B(i,0);
			mSum(c,1) = -A(j,1) + B(i,1);
			c++;
		}
	}

	return mSum;

}

/**
 * based on Gino van den Bergen's paper on GJK algorithm
 * G. V. den Bergen, “A Fast and Robust GJK Implementation for Collision Detection
 * of Convex Objects”, Journal of Graphics Tools, vol. 4, nº 2, p. 7–25, jan. 1999.
 * @param A : coordinates of the vertices of some polyhedron
 * @param B : coordinates of the vertices of some other polyhedron
 * @param maxPenetration : maximum allowed penetration between polyhedra
 * @param verbose : should verbose mode be active?
 * @return d : the distance between to polyhedra
 */

double ContactKinematicsWheelRail::gjkDistance(MatVx2 A,
		MatVx2 B, double maxPenetration, bool verbose) {

	bool contact = true;
	bool closeEnough = false;
	unsigned int dim = A.cols(); // the dimensionality of the problem
	double epsilon = 1e-8;
	VecV v, w;
	MatVx2 W, idY, idW;
	double mu = 0;
	unsigned int iter = 0; // iterations
	double normv; // the euclidian norm of v
	double delta; // normalized projection of w in v


	// the Minskowski sum of the two polyhedra
	MatVx2 minkowski = minkowskiSum(A,B);
	unsigned int nmab = minkowski.rows();

	// select the first test points
	unsigned int ptAid = 0;
	unsigned int ptBid = 0;
	MatVx2 ptA = A(Range<Var, Var>(ptAid,0),Range<Var,Var>(ptAid,1));
	MatVx2 ptB = B(Range<Var, Var>(ptBid,0),Range<Var,Var>(ptBid,1));

	// algorithm start
//	v = (VecV)(minkowski(Range<Var,Var>(0,0),Range<Var,Var>(0,1)));
	v = (VecV)minkowski.T().col(0);

	unsigned int index = 0;

	while (~closeEnough & contact & (iter < nmab)){

		polytopeMap(-v,minkowski,w,index);

		normv = fmatvec::nrm2(v);

		if (normv != 0){

		}
		else {
			delta = 0;
		}

		if (delta >= 0){
			contact = false;
		}

		epsilon++;mu++;
		iter++;
	}

	if (contact) dim = dim-1;
	std::cout << iter << std::endl;

	return 0.003;

}


/** polytope map (van den Bergen)
 * G. V. den Bergen, “A Fast and Robust GJK Implementation for Collision Detection
 * of Convex Objects”, Journal of Graphics Tools, vol. 4, nº 2, p. 7–25, jan. 1999.
 * @param vector - a vertex
 * @param polyhedron - an array of vertices from a polytope
 * @param sa - proximal point of the map
 * @param index - index of sa on the polyhedron list of vertices
 */
void ContactKinematicsWheelRail::polytopeMap(VecV vector, MatVx2 polyhedron, VecV& sa, unsigned int& index) {


	VecV X = (VecV)(polyhedron * vector);

	index = fmatvec::maxIndex(X);
	sa = polyhedron.T().col(index);

}

} /* END namespace MBSim */


