#include "toycontrol.h"
#include "simulator.h"
#include "drawstuff/drawstuff.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>

#include <ode/contact.h>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

ToyControl::ToyControl( ToyBody& human )  
	: _toy(human) {
	for (int i = 0; i < NUM_BODY-1; i++) {
		_ks[i] = 300.0; // determine
		_kd[i] = 30.0;  //how stifness 
		_target[i] = 0;
	}
/*
	_target[0] = -1; 
	_target[1] = 0.5; 
	_target[2] = 1;
*/

}


ToyControl::~ToyControl() {
}

void ToyControl::setTarget(double *t, int size){
	if (size>sizeof(_target)/sizeof(double)){
		printf("bad target array\nsize of target is %lu, size of _target is %lu, given size id %d\n",sizeof(*t)/sizeof(double),sizeof(_target)/sizeof(double),size);
		return;
	}
	for(int i = 0; i < size; i++){
		_target[i] = t[i];
	}
}


int ToyControl::action() {
	if (!_sim) {
		return -1; 	
	}
	//target->joint
	//0 left hip
	//1 right hip
	//2 left knee
	//3 right knee
	//4 left ankle
	//5 right ankle	
	
	double s0[3] = {-1,0.5,1};
	setTarget(s0,3);

	/*
	// collision detection
	dContactGeom contact[100];
	dGeomID leftfootId = _toy.box[5].id();
	dGeomID rightfootId = _toy.box[6].id();
	dGeomID groundId = _toy._env.ground.id();
	int left = dCollide(leftfootId, groundId, 100, &contact[0], sizeof(dContactGeom));
	int right = dCollide(rightfootId, groundId, 100, &contact[0], sizeof(dContactGeom));
	
	// end of collision detection
	*/
        // Add joint torques to each DOF, pulling the body towards the 
	// desired state defined by _target. 
	for (int i = 0; i < NUM_BODY-1; i++) {
		dJointID jt = _toy.joint[i].id(); 
		double limit = _ks[i]; 

		dReal theta = dJointGetHingeAngle(jt); 
		dReal thetav = dJointGetHingeAngleRate(jt); 
		dReal torque = 
			_ks[i]*(_target[i] - theta) - _kd[i]*thetav; // PD controler equation
		if (torque > limit) torque = limit; 
		if (torque < -limit) torque = -limit; // a limit on torque

		dJointAddHingeTorque(jt, torque); 
		//use this torque in the next step of simulation 
	}	
	return 0; 
}

double ToyControl::eval() {
	
	return 0.0; 
}

double ToyControl::norm() {

	return 0.0; 
}

void ToyControl::reset() {
}
	
void ToyControl::render() {
}
	
