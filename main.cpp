#include <iostream>
#include <fstream>
#include "toybody.h"
#include "toycontrol.h"
#include "simulator.h"
#include "renderer.h"

#ifdef __OSX__
#define VPX 1024
#define VPY 768
#else 
#define VPX 1024
#define VPY 768
#endif 


int main (int argc, char **argv)
{
	int render_steps = 10000; 
	Environment myEnv(0.0,-9.8,0.0);
	ToyBody myBody(myEnv);
	ToyControl myControl(myBody);
	Simulator mySim(myBody, myEnv, render_steps, STEP_SIZE);
	mySim.setController(&myControl);
	myControl.setSimulator(&mySim); 

	Renderer myRend(mySim);
	dInitODE(); 
	mySim.reset();
	myRend.renderSim(argc, argv, VPX, VPY);
	dCloseODE();
	return 0;
}

