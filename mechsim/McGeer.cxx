#include "McGeer.h"
#include "GLWidget.h"
#include <GL/gl.h>
#include <cmath>

// ** Simulation parameters **
const int McGeer::STEP_PER_SEC = 16;
const int McGeer::INT_PER_STEP = 16;   // Integration intervals per timestep
const char McGeer::TITLE[] = "McGeer Passive Walker";

// ** System parameters **
const double McGeer::GAMMA   = 0.0456;
const double McGeer::M_T     = 0.1;
const double McGeer::R_GYR_T = 0.135;
const double McGeer::L_T     = 0.46;
const double McGeer::C_T     = 0.20;
const double McGeer::W_T     = 0.00;
const double McGeer::ALPHA_T = atan2(McGeer::W_T, McGeer::C_T);
const double McGeer::M_S     = 0.062;
const double McGeer::R_GYR_S = 0.186;
const double McGeer::L_S     = 0.54;
const double McGeer::C_S     = 0.24;
const double McGeer::W_S     = 0.01;
const double McGeer::ALPHA_S = atan2(McGeer::W_S, McGeer::C_S);
const double McGeer::R       = 0.2;
const double McGeer::EPS_K   = 0.2;

// ** INITIAL PARAMETERS **
const double McGeer::INI_PHI_LT =  0.0; //-0.2;
const double McGeer::INI_PHI_LS =  0.0; //0.3;
const double McGeer::INI_PHI_RT =  0.0; //0.5;
const double McGeer::INI_PHI_RS =  1.0; //-0.2;


// ** Display parameters (these do not enter into the simulation) **
// Distance between the two legs
const double MGState::DISP_LEGDIST = .2;
// Width of the boxes representing the legs
const double MGState::DISP_LEGWIDTH = .05;


/* void near_callback(void* data, dGeomID g1, dGeomID g2)
{
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactBounce;
        contact[i].surface.bounce = 0.9;
        contact[i].surface.mu = 100.0;
        dJointID c = dJointCreateContact(((BallSlide*)data)->fWorld,
                                         ((BallSlide*)data)->fContactGroup,
                                         &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
    }
} */

void MGState::Draw() const
{
    /* glColor3f(1., 1., 0.);
    GLWidget::drawSphere(.3, fBPos);
    
    glColor3f(0., 1., 0.);
    GLWidget::drawODEBox(gSlideG, 3., 1., .2); */
    
    // GLWidget::drawCheckerboardFloor();
    // GLWidget::drawDiscSegment(1., 1., 1.);
    
    glMatrixMode(GL_MODELVIEW);
    
    glColor3f(1., 1., 0.);
    glPushMatrix();
    glTranslatef(0., DISP_LEGDIST/2., 0.);
    DrawLeg(fPhiLT, fPhiLS);
    glPopMatrix();
    
    glColor3f(1., 0., 1.);
    glPushMatrix();
    glTranslatef(0., -DISP_LEGDIST/2., 0.);
    DrawLeg(fPhiRT, fPhiRS);
    glPopMatrix();
}

/* void MGState::DrawSlide() const
{
    glBegin(GL_QUADS);
    
    glEnd(GL_QUADS);
} */

void MGState::DrawLeg(double t, double s) const
{
    glPushMatrix();
    
    // Draw thigh
    glRotatef(t*GLWidget::RAD_TO_DEG, 0., 1., 0.);
    glPushMatrix();
    glScalef(DISP_LEGWIDTH, DISP_LEGWIDTH, McGeer::L_T);
    glTranslatef(0., 0., -0.5);
    GLWidget::drawUnitCube();
    glPopMatrix();
    
    // Draw shank
    // Foot segment opening angle
    const double alpha = 2. * asin(((McGeer::L_S-McGeer::R)*sin(McGeer::EPS_K)+DISP_LEGWIDTH/2.)/McGeer::R);
    // Shank length (to beginning of foot)
    // (Note: shank length to bottom of foot is McGeer::L_S)
    const double h = (McGeer::L_S-McGeer::R)*cos(McGeer::EPS_K)  + McGeer::R * cos(alpha/2.);
    glTranslatef(0., 0., -McGeer::L_T);
    glRotatef(s*GLWidget::RAD_TO_DEG, 0., 1., 0.);
    glPushMatrix();
    glScalef(DISP_LEGWIDTH, DISP_LEGWIDTH, h);
    glTranslatef(0., 0., -0.5);
    GLWidget::drawUnitCube();
    glPopMatrix();
    
    // Draw foot
    glTranslatef((McGeer::L_S-McGeer::R)*sin(McGeer::EPS_K),
                  0.,
                  -(McGeer::L_S-McGeer::R)*cos(McGeer::EPS_K));
    glRotatef(-90., 1., 0., 0.);
    GLWidget::drawDiscSegment(McGeer::R, DISP_LEGWIDTH, alpha);
    
    glPopMatrix();
}

McGeer::McGeer()
{
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    // fFloorG = dCreatePlane(fSpace, sin(GAMMA), 0., cos(GAMMA), 0.);
    
    InitLeg(fLeftThigh, fLeftShank, 1., INI_PHI_LT, INI_PHI_LS);
    InitLeg(fRightThigh, fRightShank, -1., INI_PHI_RT, INI_PHI_RS);
    
    fContactGroup = dJointGroupCreate(0);
}

McGeer::~McGeer()
{
    dWorldDestroy(fWorld);
    dJointGroupDestroy(fContactGroup);
}

void McGeer::InitLeg(dBodyID& thigh, dBodyID& shank, double y, double iniPhiT, double iniPhiS)
{
    dMass mass;

    // NOTE: we intend the motion to be confined to a two-dimensional plane, so
    // the inertia tensor effectively reduces to a single number.
    thigh = dBodyCreate(fWorld);
    const double R_T = sqrt(C_T*C_T + W_T*W_T);
    dBodySetPosition(thigh, -R_T * sin(iniPhiT + ALPHA_T),
                                 y,
                                 -R_T * cos(iniPhiT + ALPHA_T));
    dMassSetParameters(&mass, M_T, 0., 0., 0., 1., M_T * R_GYR_T, 1., 0., 0., 0.);
    dBodySetMass(thigh, &mass);
    
    dJointID j1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j1, thigh, 0);
    dJointSetHingeAnchor(j1, 0., y, 0.);
    dJointSetHingeAxis(j1, 0., 1., 0.);
    
    shank = dBodyCreate(fWorld);
    const double R_S = sqrt(C_S*C_S + W_S*W_S);
    dBodySetPosition(shank, -L_T * sin(iniPhiT) - R_S * sin(iniPhiS + ALPHA_S),
                                 y,
                                 -L_T * cos(iniPhiT) - R_S * cos(iniPhiS + ALPHA_S));
    dMassSetParameters(&mass, M_S, 0., 0., 0., 1., M_S * R_GYR_S, 1., 0., 0., 0.);
    dBodySetMass(shank, &mass);
    
    dJointID j2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j2, thigh, shank);
    dJointSetHingeAnchor(j2, -L_T * sin(iniPhiT),
                             y,
                             -L_T * cos(iniPhiT));
    dJointSetHingeAxis(j2, 0., 1., 0.);
    dJointSetHingeParam(j2, dParamHiStop, iniPhiS);
}

void McGeer::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        // dSpaceCollide(fSpace, this, near_callback);
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        // dJointGroupEmpty(fContactGroup);
    }
}

MGState McGeer::GetCurrentState()
{
    MGState state;
    const dReal *pt, *ps;
    
    state.fPhiRT = 0.;
    state.fPhiRS = 0.;
    
    // left leg
    pt = dBodyGetPosition(fLeftThigh);
    state.fPhiLT = atan2(-pt[0], -pt[2]) - ALPHA_T;
    
    ps = dBodyGetPosition(fLeftShank);
    state.fPhiLS = atan2(-ps[0] - L_T * sin(state.fPhiLT),
                         -ps[2] - L_T * cos(state.fPhiLT)) - ALPHA_S;
    
    // right leg
    pt = dBodyGetPosition(fRightThigh);
    state.fPhiRT = atan2(-pt[0], -pt[2]) - ALPHA_T;
    
    ps = dBodyGetPosition(fRightShank);
    state.fPhiRS = atan2(-ps[0] - L_T * sin(state.fPhiRT),
                         -ps[2] - L_T * cos(state.fPhiRT)) - ALPHA_S;
    
    return state;
}
