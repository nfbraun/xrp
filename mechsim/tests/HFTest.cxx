#include "HFTest.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include <GL/gl.h>

const int HFTest::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int HFTest::INT_PER_STEP = 16;
const char HFTest::TITLE[] = "Height field test";

dGeomID gSlideG;

void near_callback(void* data, dGeomID g1, dGeomID g2)
{
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactBounce;
        contact[i].surface.bounce = 0.9;
        contact[i].surface.mu = 100.0;
        dJointID c = dJointCreateContact(((HFTest*)data)->fWorld,
                                         ((HFTest*)data)->fContactGroup,
                                         &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
    }
}

void HFState::Draw(int) const
{
    glPushMatrix();
    
    glColor3f(1., 1., 0.);
    GL::drawSphere(.2, fBPos);
    
    GL::drawHeightfield(fParent->fHField);
    
    glPopMatrix();
}

HFTest::HFTest()
    : fHField(128, 128, -5., 5., -5., 5.)
{
    dInitODE();
    
    for(unsigned int y=0; y<fHField.ysize(); y++) {
        for(unsigned int x=0; x<fHField.xsize(); x++) {
            double xc = fHField.xcoord(x);
            double yc = fHField.ycoord(y);
            fHField.at(x, y) = cos(sqrt(xc*xc + yc*yc));
        }
    }
    
    fWorld = dWorldCreate();
    fSpace = dSimpleSpaceCreate(0);
    
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fHFData = dGeomHeightfieldDataCreate();
    dGeomHeightfieldDataBuildDouble(fHFData, fHField.raw_data(), false,
                                    fHField.x2() - fHField.x1(),
                                    fHField.y2() - fHField.y1(),
                                    fHField.xsize(), fHField.ysize(),
                                    1., // scale
                                    0., // offset
                                    1., // thickness
                                    0); // wrap
    dGeomHeightfieldDataSetBounds(fHFData, -1.0, 1.0);
    
    fFloorG = dCreateHeightfield(fSpace, fHFData, true);
    dQuaternion Q = { 1./sqrt(2.), 1./sqrt(2.), 0., 0. };
    dGeomSetQuaternion(fFloorG, Q);
    
    //fFloorG = dCreatePlane(fSpace, 0., 0., 1., 0.);
    
    fBall = dBodyCreate(fWorld);
    dMass m;

    dMassSetSphere(&m, 1000.0, 0.2);
    dBodySetMass(fBall, &m);
    dBodySetPosition(fBall, 1., 1., 5.);
    
    fBallG = dCreateSphere(fSpace, 0.2);
    dGeomSetBody(fBallG, fBall);
    
    fContactGroup = dJointGroupCreate(0);
}

HFTest::~HFTest()
{
    dWorldDestroy(fWorld);
    dSpaceDestroy(fSpace);
    dJointGroupDestroy(fContactGroup);
}

void HFTest::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        dSpaceCollide(fSpace, this, near_callback);
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        dJointGroupEmpty(fContactGroup);
    }
}

HFState HFTest::GetCurrentState()
{
    HFState state;
    
    state.fParent = this;
    state.fBPos = ODE::BodyGetPosition(fBall);
    
    return state;
}
