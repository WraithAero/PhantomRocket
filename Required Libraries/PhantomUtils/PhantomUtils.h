#ifndef PhantomUtils_h
#define PhantomUtils_h

#include "Arduino.h"
#include "Stage.h"

class PhantomUtils {
	public:
	PhantomUtils(PhantomConstants constants);
	
	void logPrint(String msg, boolean ln);
	void printStage(Stage toPrint, boolean ln);
	boolean arrayEquals(int array_one[], int array_two[]);
};

#endif