#include "McGeer.h"
#include "GLWidget.h"
#include <GL/gl.h>
#include <cmath>

/* Reference: Tad McGeer, Passive Walking with Knees */

// ** Simulation parameters **
const int McGeer::STEP_PER_SEC = 16;
const int McGeer::INT_PER_STEP = 16;   // Integration intervals per timestep
const char McGeer::TITLE[] = "McGeer Passive Walker";

// ** System parameters **
const double McGeer::GAMMA   = 0.0456;   // Floor slope
const double McGeer::FLOOR_DIST = 1.0;   // shortest distance floor to origin
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
const double McGeer::INI_PHI_LT = -0.35;
const double McGeer::INI_PHI_LS = -0.35;
const double McGeer::INI_PHI_RT =  0.35;
const double McGeer::INI_PHI_RS =  0.35;


// ** Display parameters (these do not enter into the simulation) **
// Distance between the two legs
// const double MGState::DISP_LEGDIST = .2;
// Width of the boxes representing the legs
const double MGState::DISP_LEGWIDTH = .05;
// Width of slide
const double MGState::DISP_SLIDEWIDTH = 1.0;
// Half-length of slide
const int MGState::DISP_SLIDELEN2 = 3;

// Angle between thigh and slide surface normal
double LegState::thighAng() const
{
    Vector3 d = fTPos - fHPos;
    return atan2(-d.x(), -d.z()) - McGeer::GAMMA - McGeer::ALPHA_T;
}

Vector3 LegState::kneePos() const
{
    return Vector3(-McGeer::L_T * sin(thighAng() + McGeer::GAMMA),
                    fTPos.y(),
                   -McGeer::L_T * cos(thighAng() + McGeer::GAMMA)) + fHPos;
}

// Angle between shank and slide surface normal
double LegState::shankAng() const
{
    Vector3 d = fSPos - kneePos();
    return atan2(-d.x(), -d.z()) - McGeer::GAMMA - McGeer::ALPHA_S + McGeer::EPS_K;
}

Vector3 LegState::footCtr() const
{
    return Vector::Null;
}

double LegState::footClearance() const
{
    return 0.0;
}


void MGState::Draw() const
{
    /* glColor3f(1., 1., 0.);
    GLWidget::drawSphere(.3, fBPos);
    
    glColor3f(0., 1., 0.);
    GLWidget::drawODEBox(gSlideG, 3., 1., .2); */
    
    // GLWidget::drawCheckerboardFloor();
    // GLWidget::drawDiscSegment(1., 1., 1.);
    
    glMatrixMode(GL_MODELVIEW);
    
    DrawSlide();
    /* Vector3 footCtr = fLSPos + fLSRot * Vector3(dGeomGetOffsetPosition(fParent->fLeftFootG));
    GLWidget::drawSphere(McGeer::R, footCtr); */
    
    glColor3f(1., 1., 0.);
    DrawLeg(fLLeg.fTPos, fLLeg.fTRot, fLLeg.fSPos, fLLeg.fSRot);

    glColor3f(1., 0., 1.);
    DrawLeg(fRLeg.fTPos, fRLeg.fTRot, fRLeg.fSPos, fRLeg.fSRot);
}

void MGState::DrawSlide() const
{
    float x1, z1, x2, z2;
    const float xn = sin(McGeer::GAMMA);
    const float zn = cos(McGeer::GAMMA);
    
    glNormal3f(xn, 0., zn);
    
    glBegin(GL_QUADS);
    for(int k=-DISP_SLIDELEN2; k<DISP_SLIDELEN2; k++) {
        x1 = -McGeer::FLOOR_DIST * xn + k * zn;
        z1 = -McGeer::FLOOR_DIST * zn - k * xn;
        x2 = x1 + zn;
        z2 = z1 - xn;
        
        if(k % 2)
            glColor3f(1., 0., 0.);
        else
            glColor3f(0., 0., 1.);
        glVertex3f(x1, DISP_SLIDEWIDTH/2., z1);
        glVertex3f(x1, 0, z1);
        glVertex3f(x2, 0, z2);
        glVertex3f(x2, DISP_SLIDEWIDTH/2., z2);
        
        if(k % 2)
            glColor3f(0., 0., 1.);
        else
            glColor3f(1., 0., 0.);
        glVertex3f(x1, 0, z1);
        glVertex3f(x1, -DISP_SLIDEWIDTH/2., z1);
        glVertex3f(x2, -DISP_SLIDEWIDTH/2., z2);
        glVertex3f(x2, 0, z2);
    }
    glEnd();
}

void MGState::DrawLeg(Vector3 thighPos, Rotation thighRot, Vector3 shankPos, Rotation shankRot) const
{
    // Draw thigh
    glPushMatrix();
    glTranslatef(thighPos.x(), thighPos.y(), thighPos.z());
    GL::Rotate(thighRot.conj());
    glTranslatef(0., 0.,
            sqrt(McGeer::W_T*McGeer::W_T + McGeer::C_T*McGeer::C_T));
    
    glRotatef(McGeer::ALPHA_T*GLWidget::RAD_TO_DEG, 0., 1., 0.);
    glScalef(DISP_LEGWIDTH, DISP_LEGWIDTH, McGeer::L_T);
    glTranslatef(0., 0., -0.5);
    GLWidget::drawUnitCube();
    glPopMatrix();
    
    // Draw shank
    glPushMatrix();
    glTranslatef(shankPos.x(), shankPos.y(), shankPos.z());
    GL::Rotate(shankRot.conj());
    
    glTranslatef(0., 0.,
        sqrt(McGeer::W_S*McGeer::W_S + McGeer::C_S*McGeer::C_S));
    glRotatef((McGeer::EPS_K - McGeer::ALPHA_S)*GLWidget::RAD_TO_DEG,
                0., 1., 0.);
    
    // Foot segment opening angle
    const double alpha = 2. * asin(((McGeer::L_S-McGeer::R)*sin(McGeer::EPS_K)+DISP_LEGWIDTH/2.)/McGeer::R);
    // Shank length (to beginning of foot)
    // (Note: shank length to bottom of foot is McGeer::L_S)
    const double h = (McGeer::L_S-McGeer::R)*cos(McGeer::EPS_K) + McGeer::R * cos(alpha/2.);

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
    dMass mass;
    
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -1.); //-9.81);
    
    fFloorG = dCreatePlane(0, sin(GAMMA), 0., cos(GAMMA), -FLOOR_DIST);
    // fFloorG = dCreatePlane(0, 0., 0., 1., -FLOOR_DIST);
    
    fHip = dBodyCreate(fWorld);
    dBodySetPosition(fHip, 0., 0., 0.);
    dMassSetSphereTotal(&mass, 1.-2*M_T*2.*M_S, .01);
    dBodySetMass(fHip, &mass);
    
    InitLeg(fLeftThigh, fLeftShank, fLeftFootG, .1, INI_PHI_LT, INI_PHI_LS);
    InitLeg(fRightThigh, fRightShank, fRightFootG, -.1, INI_PHI_RT, INI_PHI_RS);
    
    const double v = 0.44;
    dBodySetLinearVel(fHip, v, 0., 0.);
    
    /* dBodySetLinearVel(fLeftThigh,  v*cos(GAMMA), 0., -v*sin(GAMMA));
    dBodySetLinearVel(fLeftShank,  v*cos(GAMMA), 0., -v*sin(GAMMA));
    dBodySetLinearVel(fRightThigh, v*cos(GAMMA), 0., -v*sin(GAMMA));
    dBodySetLinearVel(fRightShank, v*cos(GAMMA), 0., -v*sin(GAMMA)); */
    
    
    dJointID j1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j1, fLeftThigh, fHip);
    dJointSetHingeAnchor(j1, 0., 0., 0.);
    dJointSetHingeAxis(j1, 0., 1., 0.);
    
    dJointID j2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j2, fRightThigh, fHip);
    dJointSetHingeAnchor(j2, 0., 0., 0.);
    dJointSetHingeAxis(j2, 0., 1., 0.);
    
    fContactGroup = dJointGroupCreate(0);
}

McGeer::~McGeer()
{
    dWorldDestroy(fWorld);
    dJointGroupDestroy(fContactGroup);
}

void McGeer::InitLeg(dBodyID& thigh, dBodyID& shank, dGeomID& footG, double y, double iniPhiT, double iniPhiS)
{
    dMass mass;

    // NOTE: we intend the motion to be confined to a two-dimensional plane, so
    // the inertia tensor effectively reduces to a single number.
    thigh = dBodyCreate(fWorld);
    const double R_T = sqrt(C_T*C_T + W_T*W_T);
    dBodySetPosition(thigh, -R_T * sin(iniPhiT),
                             y,
                            -R_T * cos(iniPhiT));
    dBodySetQuaternion(thigh, Rotation(iniPhiT, Vector::eY).quatarray());
    dMassSetParameters(&mass, M_T, 0., 0., 0., 1., M_T * R_GYR_T, 1., 0., 0., 0.);
    dBodySetMass(thigh, &mass);
    
    shank = dBodyCreate(fWorld);
    const double R_S = sqrt(C_S*C_S + W_S*W_S);
    Vector3 kneePos = Vector3(-L_T * sin(iniPhiT),
                               y,
                              -L_T * cos(iniPhiT));
    Vector3 shankCoGOffset = Vector3(-R_S * sin(iniPhiS),
                                      0,
                                     -R_S * cos(iniPhiS));
    Vector3 shankCoG = kneePos + shankCoGOffset;

    dBodySetPosition(shank, shankCoG.x(), shankCoG.y(), shankCoG.z());
    Rotation shankRot(iniPhiS, Vector::eY);
    dBodySetQuaternion(shank, shankRot.quatarray());
    dMassSetParameters(&mass, M_S, 0., 0., 0., 1., M_S * R_GYR_S, 1., 0., 0., 0.);
    dBodySetMass(shank, &mass);
    
    dJointID j2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j2, thigh, shank);
    dJointSetHingeAnchor(j2, kneePos.x(), kneePos.y(), kneePos.z());
    dJointSetHingeAxis(j2, 0., 1., 0.);
    dJointSetHingeParam(j2, dParamHiStop, iniPhiS-iniPhiT+EPS_K-ALPHA_S);
    
    Vector3 footCtrOffset = Vector3(-(L_S-R) * sin(iniPhiS-ALPHA_S),
                                     0,
                                    -(L_S-R) * cos(iniPhiS-ALPHA_S));
    Vector3 footCtr = shankRot.conj() * (footCtrOffset - shankCoGOffset);
    
    footG = dCreateCapsule(0, R, 10.);
    dGeomSetBody(footG, shank);
    dGeomSetOffsetPosition(footG, footCtr.x(), footCtr.y(), footCtr.z());
    dGeomSetOffsetQuaternion(footG, Rotation(M_PI/2., Vector::eX).quatarray());
}

void McGeer::Collide(dGeomID g1, dGeomID g2)
{
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactBounce;
        contact[i].surface.bounce = 0.0;
        contact[i].surface.mu = 100.0;
        dJointID c = dJointCreateContact(fWorld, fContactGroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
    }
}


void McGeer::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        Collide(fFloorG, fLeftFootG);
        Collide(fFloorG, fRightFootG);
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        dJointGroupEmpty(fContactGroup);
    }
}

MGState McGeer::GetCurrentState()
{
    MGState state;
    
    state.fParent = this;
    
    state.fLLeg.fHPos = Vector3(dBodyGetPosition(fHip));
    state.fRLeg.fHPos = Vector3(dBodyGetPosition(fHip));
    
    state.fLLeg.fTPos = Vector3(dBodyGetPosition(fLeftThigh));
    state.fLLeg.fTRot = Rotation::FromQuatArray(dBodyGetQuaternion(fLeftThigh));
    state.fLLeg.fSPos = Vector3(dBodyGetPosition(fLeftShank));
    state.fLLeg.fSRot = Rotation::FromQuatArray(dBodyGetQuaternion(fLeftShank));
    
    state.fRLeg.fTPos = Vector3(dBodyGetPosition(fRightThigh));
    state.fRLeg.fTRot = Rotation::FromQuatArray(dBodyGetQuaternion(fRightThigh));
    state.fRLeg.fSPos = Vector3(dBodyGetPosition(fRightShank));
    state.fRLeg.fSRot = Rotation::FromQuatArray(dBodyGetQuaternion(fRightShank));
    
    /* const dReal *pt, *ps;
    
    state.fPhiRT = 0.;
    state.fPhiRS = 0.;
    
    // left leg
    pt = dBodyGetPosition(fLeftThigh);
    state.fPhiLT = atan2(-pt[0], -pt[2]) - ALPHA_T;
    
    ps = dBodyGetPosition(fLeftShank);
    state.fPhiLS = atan2(-ps[0] - L_T * sin(state.fPhiLT),
                         -ps[2] - L_T * cos(state.fPhiLT)) + EPS_K - ALPHA_S;
    
    state.pos = Vector3(ps[0], ps[1], ps[2]);
    
    // right leg
    pt = dBodyGetPosition(fRightThigh);
    state.fPhiRT = atan2(-pt[0], -pt[2]) - ALPHA_T;
    
    ps = dBodyGetPosition(fRightShank);
    state.fPhiRS = atan2(-ps[0] - L_T * sin(state.fPhiRT),
                         -ps[2] - L_T * cos(state.fPhiRT)) + EPS_K - ALPHA_S;
    */
    return state;
}
