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
const int Hobbelen::STEP_PER_SEC = 50;
const int Hobbelen::INT_PER_STEP = 32;   // Integration intervals per timestep
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
    : fSwingSpl(HobbelenConst::SWING_SPL_N_POINTS,
                HobbelenConst::SWING_SPL_X, HobbelenConst::SWING_SPL_Y)
{
    using namespace HobbelenConst;
    
    const double THETA_C = 0.27;
    const double OMEGA_C = 1.0; //1.48;
    // const double OMEGA_FT = -4.5;
    // const double OMEGA_FS = 4.5;
    
    fT = 0.;
    fSwingT = 0.;
    
    dMass mass;
    
    dInitODE();
    
    // Initially, inner leg pair in swing leg
    fILegState = Pushoff;
    fOLegState = Stance;
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fFloorG = dCreatePlane(0, FloorNormal.x(), FloorNormal.y(), FloorNormal.z(),
                           -FLOOR_DIST);
    
    ChainSegment BodyC(HobbelenConst::Body);
    ChainSegment OULegC(HobbelenConst::UpperLeg);
    ChainSegment OLLegC(HobbelenConst::LowerLeg);
    FootSegment  OFootC(HobbelenConst::Foot);
    ChainSegment IULegC(HobbelenConst::UpperLeg);
    ChainSegment ILLegC(HobbelenConst::LowerLeg);
    FootSegment  IFootC(HobbelenConst::Foot);
    
    OFootC.SetPB((-FLOOR_DIST + OFootC.r()) * FloorNormal,
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
    fILeg.HipJ.off = THETA_C - GAMMA;
    dJointAttach(fILeg.HipJ.id, fILeg.ULegB, fBodyB);
    ODE::JointSetHingeAnchor(fILeg.HipJ.id, IULegC.p1());
    ODE::JointSetHingeAxis(fILeg.HipJ.id, Vector3::eY);
    
    fOLeg.HipJ.id = dJointCreateHinge(fWorld, 0);
    fOLeg.HipJ.off = -THETA_C + GAMMA;
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
    SetKneeLock(fOLeg.KneeJ, true);
    
    // Prevent swing foot from oscillation
    // dJointSetHingeParam(fILeg.AnkleJ.id, dParamLoStop, -fILeg.AnkleJ.off);
    
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

double Hobbelen::GetHeelClearance(dBodyID footB)
{
    Rotation rot = Rotation::FromQuatArray(dBodyGetQuaternion(footB));
    Vector3 heelpos = Vector3(dBodyGetPosition(footB))
        + (rot.mat() * HobbelenConst::Foot.pbb());
    return HobbelenConst::FLOOR_DIST - HobbelenConst::Foot.r() +
        Vector::dot(heelpos, HobbelenConst::FloorNormal);
}

double Hobbelen::GetTipClearance(dBodyID footB)
{
    Rotation rot = Rotation::FromQuatArray(dBodyGetQuaternion(footB));
    Vector3 tippos = Vector3(dBodyGetPosition(footB))
        + (rot.mat() * HobbelenConst::Foot.pfb());
    return HobbelenConst::FLOOR_DIST - HobbelenConst::Foot.r() +
        Vector::dot(tippos, HobbelenConst::FloorNormal);
}

void Hobbelen::SetKneeLock(const HobJoint& kneeJ, bool lock)
{
    if(lock)
        dJointSetHingeParam(kneeJ.id, dParamLoStop, -kneeJ.off);
    else
        dJointSetHingeParam(kneeJ.id, dParamLoStop, -dInfinity);
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

void Hobbelen::AnkleTorqueControl(const HobJoint& ankleJ, StepState state)
{
    double ang = dJointGetHingeAngle(ankleJ.id) + ankleJ.off;
    double rate = dJointGetHingeAngleRate(ankleJ.id);
    double torque;
    
    if(state == Pushoff) ang += 0.25;
    
    if(ang < 0)
        torque = -HobbelenConst::K_AL * ang;
    else
        torque = -HobbelenConst::K_A * ang;
    
    if(state == Swing) {
        torque -= 0.01 * HobbelenConst::K_AL * rate;
    }
    
    if(state == Stance) {
        const double norm_time = fSwingT / HobbelenConst::STEP_T;
        const double nominal_gain = 30;
        double gain;
        if(norm_time < .2) {
            gain = norm_time / .2 * nominal_gain;
        } else if(norm_time < .8) {
            gain = nominal_gain;
        } else if(norm_time < 1.) {
            gain = (1. - (norm_time - .8))/.2 * nominal_gain;
        } else {
            gain = 0.;
        }
        
        const double ang_ref = .6 * norm_time - .3;
        torque += gain * (ang_ref - ang);
    }
    
    dJointAddHingeTorque(ankleJ.id, torque);
}

void Hobbelen::SwingHipTorqueControl(LegType leg, double t)
{
    // Inter-leg angle
    double phi = dJointGetHingeAngle(fOLeg.HipJ.id) + fOLeg.HipJ.off
        - dJointGetHingeAngle(fILeg.HipJ.id) - fILeg.HipJ.off;
    double phidot = dJointGetHingeAngleRate(fOLeg.HipJ.id)
        - dJointGetHingeAngleRate(fILeg.HipJ.id);
    
    if(leg == OuterLeg) {
        phi = -phi; phidot = -phidot;
    }
    
    // PD constants
    const double K_P = 3.0;
    const double K_D = 5.0;
    
    const double torque = K_P * (fSwingSpl.eval(t) - phi)
        + K_D * (fSwingSpl.eval_deriv(t) - phidot);
    
    if(leg == InnerLeg)
        dJointAddHingeTorque(fILeg.HipJ.id, -torque);
    else
        dJointAddHingeTorque(fOLeg.HipJ.id, -torque);
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
        Collide(fFloorG, fILeg.FFootG);
        Collide(fFloorG, fILeg.BFootG);
        
        // State machine
        if(fILegState == Pushoff) {
            if(GetTipClearance(fILeg.FootB) > 1e-3) {
                fILegState = Swing;
            }
        } else if(fILegState == Swing) {
            if(GetHeelClearance(fILeg.FootB) < 1e-3) {
                fILegState = Touchdown;
                SetKneeLock(fILeg.KneeJ, true);
                fOLegState = Pushoff;
                SetKneeLock(fOLeg.KneeJ, false);
                fSwingT = 0.;
            }
        } else if(fILegState == Touchdown) {
            if(GetTipClearance(fILeg.FootB) < 1e-3) {
                fILegState = Stance;
            }
        } else if(fILegState == Stance) {
            // Remain in stance mode until touchdown of swing leg
        }
        
        if(fOLegState == Pushoff) {
            if(GetTipClearance(fOLeg.FootB) > 1e-3) {
                fOLegState = Swing;
            }
        } else if(fOLegState == Swing) {
            if(GetHeelClearance(fOLeg.FootB) < 1e-3) {
                fOLegState = Touchdown;
                SetKneeLock(fOLeg.KneeJ, true);
                fILegState = Pushoff;
                SetKneeLock(fILeg.KneeJ, false);
                fSwingT = 0.;
            }
        } else if(fOLegState == Touchdown) {
            if(GetTipClearance(fOLeg.FootB) < 1e-3) {
                fOLegState = Stance;
            }
        } else if(fOLegState == Stance) {
            // Remain in stance mode until touchdown of swing leg
        }
        
        if(fILegState == Swing)
            SwingHipTorqueControl(InnerLeg, fSwingT);
        if(fOLegState == Swing)
            SwingHipTorqueControl(OuterLeg, fSwingT);
        
        AnkleTorqueControl(fILeg.AnkleJ, fILegState);
        AnkleTorqueControl(fOLeg.AnkleJ, fOLegState);
        
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        dJointGroupEmpty(fContactGroup);
        
        fT += 1./(STEP_PER_SEC * INT_PER_STEP);
        fSwingT += 1./(STEP_PER_SEC * INT_PER_STEP);
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
    
    state.fIHipAngle = dJointGetHingeAngle(fILeg.HipJ.id) + fILeg.HipJ.off;
    state.fOHipAngle = dJointGetHingeAngle(fOLeg.HipJ.id) + fOLeg.HipJ.off;
    
    if(fILegState == Swing || fILegState == Pushoff)
        state.fDesiredInterLeg = fSwingSpl.eval(fSwingT);
    else
        state.fDesiredInterLeg = -fSwingSpl.eval(fSwingT);
    
    state.fIKneeAngle = dJointGetHingeAngle(fILeg.KneeJ.id) + fILeg.KneeJ.off;
    state.fOKneeAngle = dJointGetHingeAngle(fOLeg.KneeJ.id) + fOLeg.KneeJ.off;
    
    state.fIAnkleAngle = dJointGetHingeAngle(fILeg.AnkleJ.id) + fILeg.AnkleJ.off;
    state.fOAnkleAngle = dJointGetHingeAngle(fOLeg.AnkleJ.id) + fOLeg.AnkleJ.off;
    
    state.fITipClear = GetTipClearance(fILeg.FootB);
    state.fIHeelClear = GetHeelClearance(fILeg.FootB);
    state.fOTipClear = GetTipClearance(fOLeg.FootB);
    state.fOHeelClear = GetHeelClearance(fOLeg.FootB);
    
    state.fILegState = fILegState;
    state.fOLegState = fOLegState;
    
    return state;
}
