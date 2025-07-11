
#ifndef gait_h
#define gait_h
#include <stdint.h>

#include "IK.h"
#include "transition.h"



/* Gait actions */
#define IGNORE 0 // Leg ignored
#define STANCE 1 // Leg on ground
#define SWING  2 // Leg off ground



typedef struct gaitSequence_t {
	uint8_t leg[4];
} gaitSequence;

typedef struct gaitConfig_t {
	gaitSequence sequence[16];
	uint8_t      sequenceLength;
	double       loopTime;       // loop time to correclty calculate number of sub moves per gate item, milliseconds
	double       offTheGround;   // in mm
	double       swingDuration;  // duration of swing in milliseconds (1sec = 1000 millisec, 1sec = 1000000microsec)
	double       duration;       // duration of gaitSequence item, in milliseconds (1sec = 1000 millisec), should be equal or longer than swingDuration
} gaitConfig;

class gait
{
	public:
		gait(gaitConfig &config, leg &legObj);
		void start(point from, point to);
		double next();
	private:
		gaitConfig *_config;
		leg        *_leg;

		transition           _transition;
		transitionParameters tParams;
		
		double   progress;
		uint16_t ticksToStop = 0;
		uint16_t ticksMax    = 0;
		uint8_t  _currentGait = 255;
	
};
#endif
