#ifndef PhantomUtils_h
#define PhantomUtils_h

#include "Arduino.h"
#include "PhantomConstants.h"
#include "PhantomModules.h"
#include "Stage.h"

class PhantomUtils {
	public:
	PhantomUtils(PhantomConstants constants, PhantomModules modules);
	
	boolean arrayEquals(int array_one[], int array_two[]);
	void logPrint(String message, boolean ln);
	void stagePrint(Stage toPrint, boolean ln);
	void dataPrint(int toPrint, boolean ln);
	
	private:
	PhantomConstants _constants;
	PhantomModules _modules;
	File main_log; 
};

#endif