/**
 * Planner
 * Almost, it is just move body and legs to the new point
 * Also it looks like a good place to check if new possition possible
 *
 * MIT Licence
 * Developed by Gleb Devyatkin (SovGVD) in 2020
 */




#ifndef PLANNER_H
#define PLANNER_H
#include <stdint.h>

#include "geometry.h"
#include "leg.h"

class planner
{
	public:
		planner(moveVector &vector, figure &bodyObj, leg &legLF, leg &legRF, leg &legLH, leg &legRH);
		void predictPosition(uint8_t steps);
		figure getBodyPosition();
		point getLegPosition(uint8_t legId);
	private:
		moveVector *_vector;
		figure     *_body;
	leg*       _legs[4]; // [LF, RF, LH, RH]
	figure     _predictedBody;
	point      _predictedLegFoot[4]; // [LF, RF, LH, RH]
	double     moveInc;    // Khởi tạo trong cpp
	double     rotateInc;  // Khởi tạo trong cpp
	// ...existing code...
};

#endif
