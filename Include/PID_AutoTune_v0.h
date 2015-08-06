#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION	0.0.1

//commonly used functions **************************************************************************
void PID_ATune(double* Input, double* Output);                       	// * Constructor.  links the Autotune to a given PID
int Runtime();						   			   	// * Similar to the PID Compue function, returns non 0 when done
void Cancel();									   	// * Stops the AutoTune	

void SetOutputStep(double);						   	// * how far above and below the starting value will the output step?	
double GetOutputStep();							   	// 

void SetControlType(int); 						   	// * Determies if the tuning parameters returned will be PI (D=0)
int GetControlType();							   	//   or PID.  (0=PI, 1=PID)			

void SetLookbackSec(int);							// * how far back are we looking to identify peaks
int GetLookbackSec();								//

void SetNoiseBand(double);							// * the autotune will ignore signal chatter smaller than this value
double GetNoiseBand();								//   this should be acurately set

double GetKp();										// * once autotune is complete, these functions contain the
double GetKi();										//   computed tuning parameters.  
double GetKd();										//

#endif

