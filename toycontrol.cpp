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

typedef struct State{
	int num;
	double deltaT;
	double c_d;
	double c_v;
	double tor;
	double swh; //0,1
	double swk; //2,3
	double swa; //4,5
	double stk; //2,3
	double sta; //4,5	
}State;

State s0 = {0,0.3,0.0,0.20,0.0,0.50,-1.10,0.20,-0.05,0.20};
State s1 = {1,0.0,2.20,0.00,0.0,-0.80,-0.05,0.20,-0.10,0.20};
State s2 = {2,0.3,0.0,0.20,0.0,0.50,-1.10,0.20,-0.05,0.20};
State s3 = {3,0.0,2.20,0.00,0.0,-0.80,-0.05,0.20,-0.10,0.20};

State now = {4,0.3,0.0,0.20,0.0,0.50,-1.10,0.20,-0.05,0.20};
clock_t before,after;
float diff = 0;

int ToyControl::action() {
	if (!_sim) {
		return -1; 	
	}
	/*
	_target[0] = -1;
	_target[1] = s0.swh;
	_target[2] = -s0.swk;
	_target[3] = -s0.stk;
	_target[4] = s0.swa;
	_target[5] = s0.sta;
*/
	// collision detection
	dContactGeom contact[100];
	dGeomID leftfootId = _toy.box[5].id();
	dGeomID rightfootId = _toy.box[6].id();
	dGeomID groundId = _toy._env.ground.id();
	int left = dCollide(leftfootId, groundId, 100, &contact[0], sizeof(dContactGeom));
	int right = dCollide(rightfootId, groundId, 100, &contact[1], sizeof(dContactGeom));
	// end of collision detection

	//target->joint
	//0 left hip
	//1 right hip
	//2 left knee
	//3 right knee
	//4 left ankle
	//5 right ankle	

	if (now.num==0){
		if (diff >= now.deltaT){
			now = s1;
			before = clock();
			std::cout<<"0->1"<<std::endl;
		}else if (right > 0){
			now = s2;
			before = clock();
			diff = 0;
			std::cout<<"0->2"<<"\tright "<<right<<std::endl;
		}else{
			//left lift
			_target[0] = -s0.swh;
			_target[1] = s0.swh;
			_target[2] = -s0.swk;//-
			_target[3] = s0.stk;
			_target[4] = s0.swa;
			_target[5] = s0.sta;
		}
	}
	if(now.num==1){
		if (left>0){
			std::cout<<"1->2"<<"\tleft "<<left<<std::endl;
			now = s2;
			before = clock();
			diff = 0;
		}else{
			//left contact
			_target[0] = -s1.swh;
			_target[1] = s1.swh;
			_target[2] = -s1.swk;//-
			_target[3] = s1.stk;
			_target[4] = s1.swa;
			_target[5] = s1.sta;
		
		}
	}
	if(now.num==2){
		if (diff >= now.deltaT){
			now = s3;
			before = clock();
			std::cout<<"2->3"<<std::endl;
		}else if (left > 0){
			now = s0;
			before = clock();
			diff = 0;
			std::cout<<"2->0"<<"\tleft "<<left<<std::endl;
		}else{
			//right lift
			_target[1] = -s2.swh;
			_target[0] = s2.swh;
			_target[3] = -s2.swk;//-
			_target[2] = s2.stk;
			_target[5] = s2.swa;
			_target[4] = s2.sta;
		}
	}
	if(now.num==3){
		if (right>0){
			std::cout<<"3->0"<<"\tright "<<right<<std::endl;
			now = s0;
			before = clock();
			diff = 0;
		}else{
			_target[1] = -s3.swh;
			_target[0] = s3.swh;
			_target[3] = -s3.swk;//-
			_target[2] = s3.stk;
			_target[5] = s3.swa;
			_target[4] = s3.sta;
		}
	}
	if(now.num==4){
		std::cout<<"starting..."<<std::endl;
		before = clock();
		_target[0] = -s0.swh;
		_target[1] = s0.swh;
		_target[2] = -s0.swk;//-
		_target[3] = s0.stk;
		_target[4] = s0.swa;
		_target[5] = s0.sta;
		now.num=0;
	}

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

	after = clock();
	diff = ((float)(after-before))/CLOCKS_PER_SEC;
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
	
