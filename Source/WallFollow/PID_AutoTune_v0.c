#include <PID_AutoTune_v0.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

void FinishUp();
bool isMax, isMin;
double *input, *output;
double setpoint;
double noiseBand;
int controlType;
bool running;
unsigned long peak1, peak2, lastTime;
int sampleTime;
int nLookBack;
int peakType;
double lastInputs[101];
double peaks[10];
int peakCount;
bool justchanged;
bool justevaled;
double absMax, absMin;
double oStep;
double outputStart;
double Ku, Pu;
static uint32_t ui32_msTime = 0;	
	
void vATuningTickMs(void)
{
	ui32_msTime++;
}
	
uint32_t millis(void)
{
	return ui32_msTime;
}

void PID_ATune(double* Input, double* Output)
{
	input = Input;
	output = Output;
	controlType =0 ; //default to PI
	noiseBand = 0.5;
	running = false;
	oStep = 30;
	SetLookbackSec(10);
	lastTime = millis();
}

void Cancel(void)
{
	running = false;
} 
 
int Runtime(void)
{
	double avgSeparation;
	
	justevaled=false;
	if(peakCount>9 && running)
	{
		running = false;
		FinishUp();
		return 1;
	}
	unsigned long now = millis();
	
	if((now-lastTime)<sampleTime) return false;
	lastTime = now;
	double refVal = *input;
	justevaled=true;
	if(!running)
	{ //initialize working variables the first time around
		peakType = 0;
		peakCount=0;
		justchanged=false;
		absMax=refVal;
		absMin=refVal;
		setpoint = refVal;
		running = true;
		outputStart = *output;
		*output = outputStart+oStep;
	}
	else
	{
		if(refVal>absMax)absMax=refVal;
		if(refVal<absMin)absMin=refVal;
	}
	
	//oscillate the output base on the input's relation to the setpoint
	
	if(refVal>setpoint+noiseBand) *output = outputStart-oStep;
	else if (refVal<setpoint-noiseBand) *output = outputStart+oStep;
	
	
  //bool isMax=true, isMin=true;
  isMax=true;isMin=true;
  //id peaks
  for(int i=nLookBack-1;i>=0;i--)
  {
    double val = lastInputs[i];
    if(isMax) isMax = refVal>val;
    if(isMin) isMin = refVal<val;
    lastInputs[i+1] = lastInputs[i];
  }
  lastInputs[0] = refVal;  
  if(nLookBack<9)
  {  //we don't want to trust the maxes or mins until the inputs array has been filled
	return 0;
	}
  
  if(isMax)
  {
    if(peakType==0)peakType=1;
    if(peakType==-1)
    {
      peakType = 1;
      justchanged=true;
      peak2 = peak1;
    }
    peak1 = now;
    peaks[peakCount] = refVal;
   
  }
  else if(isMin)
  {
    if(peakType==0)peakType=-1;
    if(peakType==1)
    {
      peakType=-1;
      peakCount++;
      justchanged=true;
    }
    
    if(peakCount<10)peaks[peakCount] = refVal;
  }
  
  if(justchanged && peakCount>2)
  { //we've transitioned.  check if we can autotune based on the last peaks
		if ((peaks[peakCount-1] > peaks[peakCount-2]) && (peaks[peakCount-2] > peaks[peakCount-3]))
		{
			avgSeparation = ((peaks[peakCount-1]-peaks[peakCount-2])+(peaks[peakCount-2]-peaks[peakCount-3]))/2;
		}
		else if (peaks[peakCount-1] > peaks[peakCount-2])
		{
			avgSeparation = ((peaks[peakCount-1]-peaks[peakCount-2])+(peaks[peakCount-3]-peaks[peakCount-2]))/2;
		}
		else if (peaks[peakCount-2] > peaks[peakCount-3])
		{
			avgSeparation = ((peaks[peakCount-2]-peaks[peakCount-1])+(peaks[peakCount-2]-peaks[peakCount-3]))/2;
		}
		else
		{
			avgSeparation = ((peaks[peakCount-2]-peaks[peakCount-1])+(peaks[peakCount-3]-peaks[peakCount-2]))/2;
		}
    
    if( avgSeparation < 0.05*(absMax-absMin))
    {
		FinishUp();
      running = false;
	  return 1;
	 
    }
  }
   justchanged=false;
	return 0;
}
void FinishUp(void)
{
	  *output = outputStart;
		//we can generate tuning parameters!
		Ku = 4*(2*oStep)/((absMax-absMin)*3.14159);
		Pu = (double)(peak1-peak2) / 1000;
}

double GetKp(void)
{
	return controlType==1 ? 0.6 * Ku : 0.4 * Ku;
}

double GetKi(void)
{
	return controlType==1? 1.2*Ku / Pu : 0.48 * Ku / Pu;  // Ki = Kc/Ti
}

double GetKd(void)
{
	return controlType==1? 0.075 * Ku * Pu : 0;  //Kd = Kc * Td
}

void SetOutputStep(double Step)
{
	oStep = Step;
}

double GetOutputStep(void)
{
	return oStep;
}

void SetControlType(int Type) //0=PI, 1=PID
{
	controlType = Type;
}
int GetControlType(void)
{
	return controlType;
}
	
void SetNoiseBand(double Band)
{
	noiseBand = Band;
}

double GetNoiseBand(void)
{
	return noiseBand;
}

void SetLookbackSec(int value)
{
	if (value<1) value = 1;
	
	if(value<25)
	{
		nLookBack = value * 4;
		sampleTime = 250;
	}
	else
	{
		nLookBack = 100;
		sampleTime = value*10;
	}
}

int GetLookbackSec(void)
{
	return nLookBack * sampleTime / 1000;
}
