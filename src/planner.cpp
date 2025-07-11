/**
 * Planner
 *
 * MIT Licence
 * Developed by Gleb Devyatkin (SovGVD) in 2020
 */
 // TODO use gait data to avoid unnecessary legs update
 // TODO do this with matrix and for 3D


#include "planner.h"
#include <math.h>

planner::planner(moveVector &vector, figure &bodyObj, leg &legLF, leg &legRF, leg &legLH, leg &legRH)
{
	_vector = &vector;
	_body   = &bodyObj;
	_legs[0] = &legLF;
	_legs[1] = &legRF;
	_legs[2] = &legLH;
	_legs[3] = &legRH;
	moveInc = 1.0;
	rotateInc = 1.0;
}

void planner::predictPosition(uint8_t steps)
{
	// TODO normalize angle !!! very important !!!
	_predictedBody.orientation.pitch = _body->orientation.pitch;
	_predictedBody.orientation.roll  = _body->orientation.roll;
	_predictedBody.orientation.yaw   = _body->orientation.yaw + rotateInc * _vector->rotate.yaw;
	
	
	double tmpSin = sin(_predictedBody.orientation.yaw);
	double tmpCos = cos(_predictedBody.orientation.yaw);
	
	// TODO use matrix
	_predictedBody.position.x = _body->position.x + moveInc * (_vector->move.x * tmpCos - _vector->move.y * tmpSin);
	_predictedBody.position.y = _body->position.y + moveInc * (_vector->move.x * tmpSin + _vector->move.y * tmpCos);
	_predictedBody.position.z = _body->position.z;
	
	
	// This is terible (code)
	// Im trying to get new position of legs based on rotation of default position for XY-plane
	// this is OK for first time, but terrible for anything else
	
	for (int i = 0; i < 4; ++i) {
		_predictedLegFoot[i].x = _predictedBody.position.x + _legs[i]->defaultFoot.x * tmpCos - _legs[i]->defaultFoot.y * tmpSin;
		_predictedLegFoot[i].y = _predictedBody.position.y + _legs[i]->defaultFoot.x * tmpSin + _legs[i]->defaultFoot.y * tmpCos;
		_predictedLegFoot[i].z = _legs[i]->defaultFoot.z;
	}

}

figure planner::getBodyPosition()
{
	return _predictedBody;
}

point planner::getLegPosition(uint8_t legId)
{
	if (legId < 4) return _predictedLegFoot[legId];
	return {0,0,0};
}
