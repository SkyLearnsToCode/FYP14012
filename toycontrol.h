#ifndef _TOY_CONTROL_
#define _TOY_CONTROL_

#include <iostream>
#include <fstream>
#include "controller.h"
#include "toybody.h"

class Simulator; 
class ToyControl : public Controller {
	ToyBody& _toy; 
	Simulator* _sim;
	double _ks[NUM_BODY-1]; 
	double _kd[NUM_BODY-1]; 
	double _target[NUM_BODY-1];
	
public: 
	ToyControl( ToyBody& toy );  
	~ToyControl(); 
	int action();
	double eval(); 
	double discount() { return 1; }
	double norm(); 
	void render();  
	void reset();

	void setSimulator( Simulator* simptr ) { 
		_sim = simptr; 
	}

	int getSimLength(); 
};

#endif 
