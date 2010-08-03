#include "Hobbelen.h"
#include "RotMotion.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include <GL/gl.h>
#include <cmath>

/* Reference:
    Daan Hobbelen, Limit Cycle Walking
    PhD thesis, TU Delft, 2008
 */

// ** Simulation parameters **
const int Hobbelen::STEP_PER_SEC = 100;
const int Hobbelen::INT_PER_STEP = 16;   // Integration intervals per timestep
const char Hobbelen::TITLE[] = "2D Walker";

// ** Display parameters (these do not enter into the simulation) **
// Width of the boxes representing the legs
const double HobState::DISP_LEGWIDTH = .05;
// Width of slide
const double HobState::DISP_SLIDEWIDTH = 1.0;
// Half-length of slide
const int HobState::DISP_SLIDELEN2 = 30;

BodyQ BodyQ::FromODE(dBodyID id)
{
    return BodyQ(Vector3(dBodyGetPosition(id)),
                 Rotation::FromQuatArray(dBodyGetQuaternion(id)),
                 Vector3(dBodyGetLinearVel(id)),
                 Vector3(dBodyGetAngularVel(id)));
}

void BodyQ::TransformGL() const
{
    glPushMatrix();
    GL::Translate(fPos);
    GL::Rotate(fRot.conj());
}

void HobState::Draw() const
{
    using namespace HobbelenConst;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    DrawSlide();
    
    glColor3f(1., 1., 1.);
    fBodyQ.TransformGL();
    GL::Translate(Body.cb());
    glScalef(.1, OUTER_LEG_DIST + DISP_LEGWIDTH, Body.l());
    GL::drawUnitCube();
    glPopMatrix();
    
    glColor3f(1., 1., 0.);
    glTranslatef(0., -INNER_LEG_DIST/2., 0.);
    DrawLeg(fIULegQ, fILLegQ, fIFootQ);
    glTranslatef(0., INNER_LEG_DIST, 0.);
    DrawLeg(fIULegQ, fILLegQ, fIFootQ);
    glTranslatef(0., -INNER_LEG_DIST/2., 0.);
    
    glColor3f(1., 0., 1.);
    glTranslatef(0., -OUTER_LEG_DIST/2., 0.);
    DrawLeg(fOULegQ, fOLLegQ, fOFootQ);
    glTranslatef(0., OUTER_LEG_DIST, 0.);
    DrawLeg(fOULegQ, fOLLegQ, fOFootQ);
    glTranslatef(0., -OUTER_LEG_DIST/2., 0.);
    
    glPopMatrix();
}

void HobState::DrawLeg(const BodyQ& upperLegQ, const BodyQ& lowerLegQ,
                       const BodyQ& footQ) const
{
    using namespace HobbelenConst;
    
    upperLegQ.TransformGL();
    GL::Translate(UpperLeg.cb());
    glScalef(DISP_LEGWIDTH, DISP_LEGWIDTH, HobbelenConst::UpperLeg.l());
    GL::drawUnitCube();
    glPopMatrix();
    
    lowerLegQ.TransformGL();
    GL::Translate(LowerLeg.cb());
    glScalef(DISP_LEGWIDTH, DISP_LEGWIDTH, HobbelenConst::LowerLeg.l());
    GL::drawUnitCube();
    glPopMatrix();
    
    footQ.TransformGL();
    
    GL::drawTube(Foot.r(), HobbelenConst::Foot.pfb() - .05 * Vector3::eY,
                           HobbelenConst::Foot.pfb() + .05 * Vector3::eY);
    GL::drawTube(Foot.r(), HobbelenConst::Foot.pbb() - .05 * Vector3::eY,
                           HobbelenConst::Foot.pbb() + .05 * Vector3::eY);
    glPopMatrix();
}

void HobState::DrawSlide() const
{
    float x1, z1, x2, z2;
    const float xn = sin(HobbelenConst::GAMMA);
    const float zn = cos(HobbelenConst::GAMMA);
    
    glNormal3f(xn, 0., zn);
    
    glBegin(GL_QUADS);
    for(int k=-DISP_SLIDELEN2; k<DISP_SLIDELEN2; k++) {
        x1 = -HobbelenConst::FLOOR_DIST * xn + k * zn;
        z1 = -HobbelenConst::FLOOR_DIST * zn - k * xn;
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

Hobbelen::Hobbelen()
{
    using namespace HobbelenConst;
    
    const double THETA_C = 0.27;
    const double OMEGA_C = 1.48;
    // const double OMEGA_FT = -4.5;
    // const double OMEGA_FS = 4.5;
    
    dMass mass;
    
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    Vector3 floorNormal(sin(GAMMA), 0., cos(GAMMA));
    fFloorG = dCreatePlane(0, floorNormal.x(), floorNormal.y(), floorNormal.z(),
                           -FLOOR_DIST);
    
    ChainSegment BodyC(HobbelenConst::Body);
    ChainSegment OULegC(HobbelenConst::UpperLeg);
    ChainSegment OLLegC(HobbelenConst::LowerLeg);
    FootSegment  OFootC(HobbelenConst::Foot);
    ChainSegment IULegC(HobbelenConst::UpperLeg);
    ChainSegment ILLegC(HobbelenConst::LowerLeg);
    FootSegment  IFootC(HobbelenConst::Foot);
    
    OFootC.SetPB((-FLOOR_DIST + OFootC.r()) * floorNormal,
                 GAMMA);
    OLLegC.SetP2(OFootC.p1(), -THETA_C + GAMMA);
    OULegC.SetP2(OLLegC.p1(), -THETA_C + GAMMA);
    BodyC.SetP2(OULegC.p1(), 0.);
    IULegC.SetP1(OULegC.p1(), THETA_C + GAMMA);
    ILLegC.SetP1(IULegC.p2(), THETA_C + GAMMA);
    IFootC.SetP1(ILLegC.p2(), GAMMA);
    
    InitLeg(fILeg, IULegC, ILLegC, IFootC);
    InitLeg(fOLeg, OULegC, OLLegC, OFootC);
    
    fBodyB = BodyFromConfig(BodyC);
    fILeg.HipJ.id = dJointCreateHinge(fWorld, 0);
    dJointAttach(fILeg.HipJ.id, fILeg.ULegB, fBodyB);
    ODE::JointSetHingeAnchor(fILeg.HipJ.id, IULegC.p1());
    ODE::JointSetHingeAxis(fILeg.HipJ.id, Vector3::eY);
    
    fOLeg.HipJ.id = dJointCreateHinge(fWorld, 0);
    dJointAttach(fOLeg.HipJ.id, fOLeg.ULegB, fBodyB);
    ODE::JointSetHingeAnchor(fOLeg.HipJ.id, OULegC.p1());
    ODE::JointSetHingeAxis(fOLeg.HipJ.id, Vector3::eY);
    
    dJointID jm = dJointCreateAMotor(fWorld, 0);
    dJointAttach(jm, fBodyB, 0);
    dJointSetAMotorNumAxes(jm, 1);
    dJointSetAMotorAxis(jm, 0, 0, 0., 1., 0.);
    dJointSetAMotorParam(jm, dParamFMax, 100.);
    dJointSetAMotorParam(jm, dParamVel, 0.);
    
    RotMotion mot = RotMotion::Rotation(OFootC.p1(), OMEGA_C * Vector3::eY);
    
    ODE::BodySetLinearVel(fOLeg.LLegB, mot.v(OLLegC.CoG()));
    ODE::BodySetAngularVel(fOLeg.LLegB, mot.omega());
    ODE::BodySetLinearVel(fOLeg.ULegB, mot.v(OULegC.CoG()));
    ODE::BodySetAngularVel(fOLeg.ULegB, mot.omega());
    
    ODE::BodySetLinearVel(fBodyB, mot.v(BodyC.p2()));
    ODE::BodySetAngularVel(fBodyB, Vector3::Null);
    
    /* mot = combine(mot, RotMotion::Rotation(IULegC.p1(),
        (OMEGA_FT - OMEGA_C)*Vector3::eY));
    ODE::BodySetLinearVel(fILeg.ULegB, mot.v(IULegC.CoG()));
    ODE::BodySetAngularVel(fILeg.ULegB, mot.omega());
    
    mot = combine(mot, RotMotion::Rotation(ILLegC.p1(),
        (OMEGA_FS - OMEGA_FT)*Vector3::eY));
    ODE::BodySetLinearVel(fILeg.LLegB, mot.v(ILLegC.CoG()));
    ODE::BodySetAngularVel(fILeg.LLegB, mot.omega());
    ODE::BodySetLinearVel(fILeg.FootB, mot.v(IFootC.CoG()));
    ODE::BodySetAngularVel(fILeg.FootB, mot.omega()); */
    
    // Lock stance knee
    dJointSetHingeParam(fOLeg.KneeJ.id, dParamLoStop, -fOLeg.KneeJ.off);
    
    // Prevent swing foot from oscillation
    dJointSetHingeParam(fILeg.AnkleJ.id, dParamLoStop, -fILeg.AnkleJ.off);
    
    fContactGroup = dJointGroupCreate(0);
}

dBodyID Hobbelen::BodyFromConfig(const BodyConf& conf)
{
    dBodyID body;
    dMass mass;
    
    body = dBodyCreate(fWorld);
    dBodySetPosition(body, conf.CoG().x(), conf.CoG().y(), conf.CoG().z());
    dBodySetQuaternion(body, conf.Rot().quatarray());
    dMassSetParameters(&mass, conf.m(), 0., 0., 0.,
                       conf.I(), conf.I(), conf.I(),
                       0., 0., 0.);
    dBodySetMass(body, &mass);
    
    return body;
}

void Hobbelen::InitLeg(HobLeg& leg, ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC)
{
    leg.ULegB = BodyFromConfig(upperLegC);
    
    leg.LLegB = BodyFromConfig(lowerLegC);
    
    leg.KneeJ.id = dJointCreateHinge(fWorld, 0);
    leg.KneeJ.off = upperLegC.theta() - lowerLegC.theta();
    dJointAttach(leg.KneeJ.id, leg.ULegB, leg.LLegB);
    ODE::JointSetHingeAnchor(leg.KneeJ.id, upperLegC.p2());
    ODE::JointSetHingeAxis(leg.KneeJ.id, Vector3::eY);
    dJointSetHingeParam(leg.KneeJ.id, dParamHiStop, -leg.KneeJ.off);
    
    leg.FootB = BodyFromConfig(footC);
    
    leg.FFootG = dCreateCapsule(0, footC.r(), .1);
    dGeomSetBody(leg.FFootG, leg.FootB);
    ODE::GeomSetOffsetPosition(leg.FFootG, footC.pfb());
    dGeomSetOffsetQuaternion(leg.FFootG, Rotation(M_PI/2., Vector3::eX).quatarray());
    
    leg.BFootG = dCreateCapsule(0, footC.r(), .1);
    dGeomSetBody(leg.BFootG, leg.FootB);
    ODE::GeomSetOffsetPosition(leg.BFootG, footC.pbb());
    dGeomSetOffsetQuaternion(leg.BFootG, Rotation(M_PI/2., Vector3::eX).quatarray());
    
    leg.AnkleJ.id = dJointCreateHinge(fWorld, 0);
    dJointAttach(leg.AnkleJ.id, leg.LLegB, leg.FootB);
    ODE::JointSetHingeAnchor(leg.AnkleJ.id, lowerLegC.p2());
    ODE::JointSetHingeAxis(leg.AnkleJ.id, Vector3::eY);
    
    leg.AnkleJ.off = lowerLegC.theta() - footC.theta();
}

Hobbelen::~Hobbelen()
{
    dWorldDestroy(fWorld);
    dJointGroupDestroy(fContactGroup);
}

void Hobbelen::KneeLockControl()
{
    double ang = dJointGetHingeAngle(fILeg.KneeJ.id) + fILeg.KneeJ.off;
    
    if(ang > 0) {
        dJointSetHingeParam(fILeg.KneeJ.id, dParamLoStop, -fILeg.KneeJ.off);
        dJointSetHingeParam(fOLeg.KneeJ.id, dParamLoStop, -dInfinity);
    }
}

/*** UNUSED ***
 In the simulation, this is much better achieved with ODEs angular motor
void Hobbelen::HipTorqueControl()
{
    Rotation rot = Rotation::FromQuatArray(dBodyGetQuaternion(fBodyB));
    Vector3 zp = rot * Vector3::eZ;
    double ang = atan2(zp.x(), zp.z());
    double avel = Vector3(dBodyGetAngularVel(fBodyB)).y();
    
    //double torque = 200.*ang + 30.*avel;
    double torque = 50.*ang + 10.*avel;
    
    dJointAddHingeTorque(fOLeg.HipJ.id, torque);
} */

void Hobbelen::AnkleTorqueControl(const struct HobJoint& ankleJ)
{
    double ang = dJointGetHingeAngle(ankleJ.id) + ankleJ.off;
    if(ang < 0)
        dJointAddHingeTorque(ankleJ.id, -HobbelenConst::K_AL * ang);
    else
        dJointAddHingeTorque(ankleJ.id, -HobbelenConst::K_A * ang);
}

void Hobbelen::Collide(dGeomID g1, dGeomID g2)
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

void Hobbelen::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        Collide(fFloorG, fOLeg.FFootG);
        Collide(fFloorG, fOLeg.BFootG);
        //Collide(fFloorG, fILeg.FFootG);
        //Collide(fFloorG, fILeg.BFootG);
        
        //KneeLockControl();
        AnkleTorqueControl(fOLeg.AnkleJ);
        //AnkleTorqueControl(fILeg.AnkleJ);
        
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        dJointGroupEmpty(fContactGroup);
    }
}

HobState Hobbelen::GetCurrentState()
{
    HobState state;
    
    state.fParent = this;
    
    state.fBodyQ = BodyQ::FromODE(fBodyB);
    
    state.fIULegQ = BodyQ::FromODE(fILeg.ULegB);
    state.fILLegQ = BodyQ::FromODE(fILeg.LLegB);
    state.fIFootQ = BodyQ::FromODE(fILeg.FootB);
    
    state.fOULegQ = BodyQ::FromODE(fOLeg.ULegB);
    state.fOLLegQ = BodyQ::FromODE(fOLeg.LLegB);
    state.fOFootQ = BodyQ::FromODE(fOLeg.FootB);
    
    state.dbg1 = dJointGetHingeAngle(fILeg.KneeJ.id) + fILeg.KneeJ.off;
    
    return state;
}
