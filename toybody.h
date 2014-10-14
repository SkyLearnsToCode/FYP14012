#ifndef _TOY_BODY_
#define _TOY_BODY_

#include <fstream>
#include "body.h"
#include "environment.h"

#define NUM_BODY 7 

class ToyBody : public Body {
	friend class ToyControl; 
	dBody body[NUM_BODY];
	dHingeJoint joint[NUM_BODY-1];
	dJointID joint_2d_cons[NUM_BODY];
	dBox box[NUM_BODY];
	Environment& _env; 

	void positionBody(); 
public: 
	ToyBody( Environment& env ); 
	void render(); 
	void reset();  
}; 

#endif
