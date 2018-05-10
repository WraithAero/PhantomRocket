#ifndef PhantomComm_h
#define PhantomComm_h

#include "Arduino.h"
#include "PhantomConstants.h"
#include "Stage.h"

class PhantomComm {
	public:
	PhantomComm(PhantomConstants constants);
	
	void signalPad();
	Stage getPadCommand();
};

#endif