/*
Compile with: g++ -DdDOUBLE -o bugtest bugtest.cxx -lode

Cylinder drops on a tilted box and rolls off. The situation is set up such that
the cylinder should never move in the y direction. This works for capsule +
plane, or for cylinder + box/plane, but fails for capsule + box.
*/

#include <iostream>
#include <ode/ode.h>

const int STEPS_PER_SEC = 16;
const int INT_PER_STEP = 16;
const int END_TIME = 20;

dWorldID fWorld;
dJointGroupID fContactGroup;

void Collide(dGeomID g1, dGeomID g2)
{
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = 0;
        contact[i].surface.bounce = 0.0;
        contact[i].surface.mu = 100.0;
        dJointID c = dJointCreateContact(fWorld, fContactGroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
    }
}

int main()
{
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -1.);
    
    dMass mass;
    dBodyID fThing = dBodyCreate(fWorld);
    dBodySetPosition(fThing, 0., 0., 0.);
    dMassSetSphereTotal(&mass, 1., .01);
    dBodySetMass(fThing, &mass);
    
    dQuaternion q;
    const double GAMMA = 0.5;   // Plane slope
    const double H = 0.1;       // Drop height
    //dGeomID fFloorG = dCreatePlane(0, sin(GAMMA), 0., cos(GAMMA), -1.0-H);
    dGeomID fFloorG = dCreateBox(0, 1000., 1000., 1.);
    const double d = -1.5-H;
    dGeomSetPosition(fFloorG, d*sin(GAMMA), 0., d*cos(GAMMA));
    dQFromAxisAndAngle(q, 0., 1., 0., GAMMA);
    dGeomSetQuaternion(fFloorG, q);
    
    dGeomID fThingG = dCreateCapsule(0, 1.0, 5.0);
    //dGeomID fThingG = dCreateCylinder(0, 1.0, 5.0);
    dGeomSetBody(fThingG, fThing);
    
    dQFromAxisAndAngle(q, 1., 0., 0., M_PI/2.);
    dGeomSetOffsetQuaternion(fThingG, q);
    
    fContactGroup = dJointGroupCreate(0);
    
    for(int t=0; t<(STEPS_PER_SEC*END_TIME); t++) {
        for(int i=0; i<INT_PER_STEP; ++i) {
            Collide(fFloorG, fThingG);
            
            dWorldStep(fWorld, 1./(STEPS_PER_SEC * INT_PER_STEP));
            dJointGroupEmpty(fContactGroup);
        }
        
        std::cout << ((double)t / STEPS_PER_SEC) << " ";
        std::cout << dBodyGetPosition(fThing)[0] << " ";
        std::cout << dBodyGetPosition(fThing)[1] << " ";
        std::cout << dBodyGetPosition(fThing)[2] << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
