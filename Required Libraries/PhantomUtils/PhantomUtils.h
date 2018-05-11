#ifndef PhantomUtils_h
#define PhantomUtils_h

#include "Arduino.h"
#include "Stage.h"

class PhantomUtils {
	public:
	PhantomUtils(PhantomConstants constants);
	
	void logPrint(String msg, boolean ln);
	void stagePrint(Stage toPrint, boolean ln);
	void dataPrint(int data, boolean ln);
	boolean arrayEquals(int array_one[], int array_two[]);
};

#endif