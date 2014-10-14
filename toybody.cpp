#include <iostream>
#include "drawstuff/drawstuff.h"
#include "toybody.h"
#include "simulator.h"

#define YSIDE (0.2)		// side length of a box
#define XSIDE (0.04)		// side length of a box
#define ZSIDE (1e-6)		// side length of a box
#define MASS (1.0)		// mass of a box

#define TORSOL (0.480) 
#define TORSOM (7.0) 
#define THIGHL (0.45) 
#define THIGHM (5) 
#define SHINL (0.4) 
#define SHINM (5) 
#define FOOTL (0.2) 
#define FOOTM (1) 
#define TORSO_POS (0.5*TORSOL+THIGHL+SHINL+0.5*XSIDE) 

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

static const int drawOrder[NUM_BODY] = {1, 3, 5, 0, 2 ,4, 6}; 

void ToyBody::positionBody() {
	dMass m;
	// torso
	body[0].setPosition(0, TORSO_POS, 0) ; 
	m.setBox (1,XSIDE,TORSOL,ZSIDE);
	m.adjust (TORSOM);
	body[0].setMass (&m);
	box[0].setBody (body[0]);
	
	// left thigh
	body[1].setPosition(0, TORSO_POS-0.5*TORSOL-0.5*THIGHL, 0) ; 
	m.setBox (1,XSIDE,THIGHL,ZSIDE);
	m.adjust (THIGHM);
	body[1].setMass (&m);
	box[1].setBody (body[1]);

	// left hip	
	joint[0].attach (body[0],body[1]);
	joint[0].setAnchor (0,TORSO_POS-0.5*TORSOL,0);
	joint[0].setAxis (0,0,1.0);
	
	// right thigh
	body[2].setPosition(0, TORSO_POS-0.5*TORSOL-0.5*THIGHL, 0) ; 
	m.setBox (1,XSIDE,THIGHL,ZSIDE);
	m.adjust (THIGHM);
	body[2].setMass (&m);
	box[2].setBody (body[2]);
	
	// right hip
	joint[1].attach (body[0],body[2]);
	joint[1].setAnchor (0,TORSO_POS-0.5*TORSOL,0);
	joint[1].setAxis (0,0,1.0);
	
	// left shin
	body[3].setPosition(0, TORSO_POS-0.5*TORSOL-THIGHL-0.5*SHINL, 0) ; 
	m.setBox (1,XSIDE,SHINL,ZSIDE);
	m.adjust (SHINM);
	body[3].setMass (&m);
	box[3].setBody (body[3]);
	
	// left knee	
	joint[2].attach (body[1],body[3]);
	joint[2].setAnchor (0,TORSO_POS-0.5*TORSOL-THIGHL,0);
	joint[2].setAxis (0,0,1.0);
	
	// right shin
	body[4].setPosition(0, TORSO_POS-0.5*TORSOL-THIGHL-0.5*SHINL, 0) ; 
	m.setBox (1,XSIDE,SHINL,ZSIDE);
	m.adjust (SHINM);
	body[4].setMass (&m);
	box[4].setBody (body[4]);

	// right knee	
	joint[3].attach (body[2],body[4]);
	joint[3].setAnchor (0,TORSO_POS-0.5*TORSOL-THIGHL,0);
	joint[3].setAxis (0,0,1.0);
	
	// left foot
	body[5].setPosition(FOOTL/2, TORSO_POS-0.5*TORSOL-THIGHL-SHINL, 0) ; 
	m.setBox (1,FOOTL,XSIDE,ZSIDE);
	m.adjust (FOOTM);
	body[5].setMass (&m);
	box[5].setBody (body[5]);

	// left ankle	
	joint[4].attach (body[3],body[5]);
	joint[4].setAnchor (0,TORSO_POS-0.5*TORSOL-THIGHL-SHINL,0);
	joint[4].setAxis (0,0,1.0);
	
	// right foot
	body[6].setPosition(FOOTL/2, TORSO_POS-0.5*TORSOL-THIGHL-SHINL, 0) ; 
	m.setBox (1,FOOTL,XSIDE,ZSIDE);
	m.adjust (FOOTM);
	body[6].setMass (&m);
	box[6].setBody (body[6]);
	
	// right ankle
	joint[5].attach (body[4],body[6]);
	joint[5].setAnchor (0,TORSO_POS-0.5*TORSOL-THIGHL-SHINL,0);
	joint[5].setAxis (0,0,1.0);
}

ToyBody::ToyBody( Environment& env ) : _env(env) {
	for (int i = 0; i < NUM_BODY; i++) {
		body[i].create(_env.world);
	}
	for (int i = 0; i < NUM_BODY-1; i++) {
		joint[i].create(_env.world);
	}
	box[0].create (_env.space,XSIDE,TORSOL,ZSIDE);
	box[1].create (_env.space,XSIDE,THIGHL,ZSIDE);
	box[2].create (_env.space,XSIDE,THIGHL,ZSIDE);
	box[3].create (_env.space,XSIDE,SHINL,ZSIDE);
	box[4].create (_env.space,XSIDE,SHINL,ZSIDE);
	box[5].create (_env.space,FOOTL,XSIDE,ZSIDE);
	box[6].create (_env.space,FOOTL,XSIDE,ZSIDE);
	
	for (int i=0; i<NUM_BODY; i++) {
		joint_2d_cons[i] = dJointCreatePlane2D(_env.world.id(),0); 
		dJointAttach(joint_2d_cons[i], body[i].id(),0); 
	}
	
	positionBody(); 
}

void ToyBody::reset() {
}

void ToyBody::render() {
	dsSetTexture (DS_WOOD);
	for (int i=0; i<NUM_BODY; i++) {
		dVector3 sides;
		int j = drawOrder[i];  
		dGeomBoxGetLengths(box[j], sides); 
		dBodyID bId = box[j].getBody();
		
		if (j < 1) {
			dsSetColor(1,1,0); 
		}
		if (j == 2 || j == 4 || j == 6) {
			dsSetColor(0.3,0.3,0.6); 
		}
		if (j == 1 || j == 3 || j == 5) {
			dsSetColor(0.3,0.6,0.3); 
		}
		dsDrawBox (dBodyGetPosition(bId),dBodyGetRotation(bId),sides);
	}

	for (int i = 0; i < NUM_BODY-1; i++) {
		dMatrix3 R; 	
		dRSetIdentity(R);
		dVector3 jpos, sides;
		sides[0] = sides[1] = sides[2] = 0.05; 
		dsSetColor(1.0,0.0, 1.0);  
		dJointGetHingeAnchor(joint[i].id(), jpos); 
		dsDrawBox (jpos,R,sides);
	}
}
	
