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
const int Hobbelen::STEP_PER_SEC = 25;
const int Hobbelen::INT_PER_STEP = 64;   // Integration intervals per timestep
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
    return BodyQ(ODE::BodyGetPosition(id),
                 ODE::BodyGetQuaternion(id),
                 ODE::BodyGetLinearVel(id),
                 ODE::BodyGetAngularVel(id));
}

void BodyQ::TransformGL() const
{
    glPushMatrix();
    GL::Translate(fPos);
    GL::Rotate(fRot);
}

void HobState::Draw(int) const
{
    using namespace HobbelenConst;
    
    glMatrixMode(GL_MODELVIEW);
    
    DrawRobot(false);
    
    GL::shadowsBeginFloor();
    DrawSlide();
    GL::shadowsBeginObjects(Eigen::Vector3d(sin(GAMMA), 0., cos(GAMMA)), -FLOOR_DIST);
    DrawRobot(true);
    GL::shadowsEnd();
}

void HobState::DrawRobot(bool shadowmode) const
{
    using namespace HobbelenConst;
    
    glPushMatrix();
    
    if(!shadowmode) glColor3f(1., 1., 1.);
    fBodyQ.TransformGL();
    GL::Translate(Body.cb());
    glScalef(.1, DISP_BODYWIDTH, Body.l());
    GL::drawUnitCube();
    glPopMatrix();
    
    if(!shadowmode) glColor3f(1., 1., 0.);
    DrawLeg(fLULegQ, fLLLegQ, fLFootQ);
    
    if(!shadowmode) glColor3f(1., 0., 1.);
    DrawLeg(fRULegQ, fRLLegQ, fRFootQ);
    
    glPopMatrix();
}

void HobState::DrawLeg(const BodyQ& upperLegQ, const BodyQ& lowerLegQ,
                       const BodyQ& footQ) const
{
    using namespace HobbelenConst;
    const Eigen::Vector3d eY = Eigen::Vector3d::UnitY();
    
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
    
    GL::drawTube(Foot.r(), HobbelenConst::Foot.pfb() - HobbelenConst::FOOT_WIDTH/2.*eY,
                           HobbelenConst::Foot.pfb() + HobbelenConst::FOOT_WIDTH/2.*eY);
    GL::drawTube(Foot.r(), HobbelenConst::Foot.pbb() - HobbelenConst::FOOT_WIDTH/2.*eY,
                           HobbelenConst::Foot.pbb() + HobbelenConst::FOOT_WIDTH/2.*eY);
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
    : fSwingHipSpl(HobbelenConst::SWING_HIP_N_POINTS,
                   HobbelenConst::SWING_HIP_X, HobbelenConst::SWING_HIP_Y),
      fSwingKneeSpl(HobbelenConst::SWING_KNEE_N_POINTS,
                    HobbelenConst::SWING_KNEE_X, HobbelenConst::SWING_KNEE_Y)
{
    using namespace HobbelenConst;
    const Eigen::Vector3d eX = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d eY = Eigen::Vector3d::UnitY();
    
    const double THETA_C = 0.27;
    const double OMEGA_C = 1.0; //1.48;
    // const double OMEGA_FT = -4.5;
    // const double OMEGA_FS = 4.5;
    
    fT = 0.;
    fSwingT = 0.;
    
    dMass mass;
    
    dInitODE();
    
    fDesiredLRoll = 0.;
    fDesiredRRoll = 0.;
    
    // Initially, left leg is swing leg
    fLLegState = Pushoff;
    fRLegState = Stance;
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fFloorG = dCreatePlane(0, FloorNormal.x(), FloorNormal.y(), FloorNormal.z(),
                           -FLOOR_DIST);
    
    ChainSegment BodyC(HobbelenConst::Body);
    ChainSegment RULegC(HobbelenConst::UpperLeg);
    ChainSegment RLLegC(HobbelenConst::LowerLeg);
    FootSegment  RFootC(HobbelenConst::Foot);
    ChainSegment LULegC(HobbelenConst::UpperLeg);
    ChainSegment LLLegC(HobbelenConst::LowerLeg);
    FootSegment  LFootC(HobbelenConst::Foot);
    
    RFootC.SetPB((-FLOOR_DIST + RFootC.r()) * FloorNormal - LEG_DIST/2.*eY,
                 GAMMA);
    RLLegC.SetP2(RFootC.p1(), -THETA_C + GAMMA);
    RULegC.SetP2(RLLegC.p1(), -THETA_C + GAMMA);
    BodyC.SetP2(RULegC.p1() + LEG_DIST/2.*eY, 0.);
    LULegC.SetP1(BodyC.p2() + LEG_DIST/2.*eY, THETA_C + GAMMA);
    LLLegC.SetP1(LULegC.p2(), THETA_C + GAMMA);
    LFootC.SetP1(LLLegC.p2(), GAMMA);
    
    InitLeg(fLLeg, LULegC, LLLegC, LFootC);
    InitLeg(fRLeg, RULegC, RLLegC, RFootC);
    
    fBodyB = BodyFromConfig(BodyC);
    fLLeg.HipJ.id = dJointCreateUniversal(fWorld, 0);
    fLLeg.HipJ.off = THETA_C - GAMMA;
    dJointAttach(fLLeg.HipJ.id, fLLeg.ULegB, fBodyB);
    ODE::JointSetUniversalAnchor(fLLeg.HipJ.id, LULegC.p1());
    ODE::JointSetUniversalAxis1(fLLeg.HipJ.id, eX);
    ODE::JointSetUniversalAxis2(fLLeg.HipJ.id, eY);
    
    #ifndef WALKER_3D
    dJointSetUniversalParam(fLLeg.HipJ.id, dParamVel, 0.);
    dJointSetUniversalParam(fLLeg.HipJ.id, dParamFMax, 100.);
    #endif
    
    fRLeg.HipJ.id = dJointCreateUniversal(fWorld, 0);
    fRLeg.HipJ.off = -THETA_C + GAMMA;
    dJointAttach(fRLeg.HipJ.id, fRLeg.ULegB, fBodyB);
    ODE::JointSetUniversalAnchor(fRLeg.HipJ.id, RULegC.p1());
    ODE::JointSetUniversalAxis1(fRLeg.HipJ.id, eX);
    ODE::JointSetUniversalAxis2(fRLeg.HipJ.id, eY);
    dJointSetUniversalParam(fRLeg.HipJ.id, dParamVel, 0.);
    dJointSetUniversalParam(fRLeg.HipJ.id, dParamFMax, 100.);
    
    dJointID jm = dJointCreateAMotor(fWorld, 0);
    dJointAttach(jm, fBodyB, 0);
    dJointSetAMotorNumAxes(jm, 1);
    dJointSetAMotorAxis(jm, 0, 0, 0., 1., 0.);
    dJointSetAMotorParam(jm, dParamFMax, 100.);
    dJointSetAMotorParam(jm, dParamVel, 0.);
    
    RotMotion mot = RotMotion::Rotation(RFootC.p1(), OMEGA_C * eY);
    
    ODE::BodySetLinearVel(fRLeg.LLegB, mot.v(RLLegC.CoG()));
    ODE::BodySetAngularVel(fRLeg.LLegB, mot.omega());
    ODE::BodySetLinearVel(fRLeg.ULegB, mot.v(RULegC.CoG()));
    ODE::BodySetAngularVel(fRLeg.ULegB, mot.omega());
    
    ODE::BodySetLinearVel(fBodyB, mot.v(BodyC.p2()));
    ODE::BodySetAngularVel(fBodyB, Eigen::Vector3d::Zero());
    
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
    SetKneeLock(fRLeg.KneeJ, true);
    
    // Prevent swing foot from oscillation
    // dJointSetHingeParam(fILeg.AnkleJ.id, dParamLoStop, -fILeg.AnkleJ.off);
    
    fContactGroup = dJointGroupCreate(0);
}

dBodyID Hobbelen::BodyFromConfig(const BodyConf& conf)
{
    dBodyID body;
    dMass mass;
    
    body = dBodyCreate(fWorld);
    ODE::BodySetPosition(body, conf.CoG());
    ODE::BodySetQuaternion(body, conf.Rot());
    dMassSetParameters(&mass, conf.m(), 0., 0., 0.,
                       conf.I(), conf.I(), conf.I(),
                       0., 0., 0.);
    dBodySetMass(body, &mass);
    
    return body;
}

void Hobbelen::InitLeg(HobLeg& leg, ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC)
{
    const Eigen::Vector3d eX = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d eY = Eigen::Vector3d::UnitY();
    
    leg.ULegB = BodyFromConfig(upperLegC);
    
    leg.LLegB = BodyFromConfig(lowerLegC);
    
    leg.KneeJ.id = dJointCreateHinge(fWorld, 0);
    leg.KneeJ.off = upperLegC.theta() - lowerLegC.theta();
    dJointAttach(leg.KneeJ.id, leg.ULegB, leg.LLegB);
    ODE::JointSetHingeAnchor(leg.KneeJ.id, upperLegC.p2());
    ODE::JointSetHingeAxis(leg.KneeJ.id, eY);
    dJointSetHingeParam(leg.KneeJ.id, dParamHiStop, -leg.KneeJ.off);
    
    leg.FootB = BodyFromConfig(footC);
    
    leg.FFootG = dCreateCapsule(0, footC.r(), HobbelenConst::FOOT_WIDTH);
    dGeomSetBody(leg.FFootG, leg.FootB);
    ODE::GeomSetOffsetPosition(leg.FFootG, footC.pfb());
    Eigen::Quaterniond FFootGOffset(Eigen::AngleAxis<double>(M_PI/2., eX));
    ODE::GeomSetOffsetQuaternion(leg.FFootG, FFootGOffset);
    
    leg.BFootG = dCreateCapsule(0, footC.r(), HobbelenConst::FOOT_WIDTH);
    dGeomSetBody(leg.BFootG, leg.FootB);
    ODE::GeomSetOffsetPosition(leg.BFootG, footC.pbb());
    Eigen::Quaterniond BFootGOffset(Eigen::AngleAxis<double>(M_PI/2., eX));
    ODE::GeomSetOffsetQuaternion(leg.BFootG, BFootGOffset);
    
    leg.AnkleJ.id = dJointCreateUniversal(fWorld, 0);
    dJointAttach(leg.AnkleJ.id, leg.LLegB, leg.FootB);
    ODE::JointSetUniversalAnchor(leg.AnkleJ.id, lowerLegC.p2());
    ODE::JointSetUniversalAxis1(leg.AnkleJ.id, eX);
    ODE::JointSetUniversalAxis2(leg.AnkleJ.id, eY);
    
    dJointSetUniversalParam(leg.AnkleJ.id, dParamVel, 0.);
    dJointSetUniversalParam(leg.AnkleJ.id, dParamFMax, 100.);
    
    leg.AnkleJ.off = lowerLegC.theta() - footC.theta();
}

Hobbelen::~Hobbelen()
{
    dWorldDestroy(fWorld);
    dJointGroupDestroy(fContactGroup);
}

double Hobbelen::GetHeelClearance(dBodyID footB)
{
    Eigen::Quaterniond rot = ODE::BodyGetQuaternion(footB);
    Eigen::Vector3d heelpos = ODE::BodyGetPosition(footB)
        + (rot * HobbelenConst::Foot.pbb());
    return HobbelenConst::FLOOR_DIST - HobbelenConst::Foot.r() +
        heelpos.dot(HobbelenConst::FloorNormal);
}

double Hobbelen::GetTipClearance(dBodyID footB)
{
    Eigen::Quaterniond rot = ODE::BodyGetQuaternion(footB);
    Eigen::Vector3d tippos = ODE::BodyGetPosition(footB)
        + (rot * HobbelenConst::Foot.pfb());
    return HobbelenConst::FLOOR_DIST - HobbelenConst::Foot.r() +
        tippos.dot(HobbelenConst::FloorNormal);
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

void Hobbelen::AnklePitchTorqueControl(const HobJoint& ankleJ, StepState state)
{
    double ang = dJointGetUniversalAngle2(ankleJ.id) + ankleJ.off;
    double rate = dJointGetUniversalAngle2Rate(ankleJ.id);
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
    
    dJointAddUniversalTorques(ankleJ.id, 0., torque);
}

void Hobbelen::AnkleRollTorqueControl(const HobJoint& ankleJ)
{
    double ang = dJointGetUniversalAngle1(ankleJ.id);
    double rate = dJointGetUniversalAngle1Rate(ankleJ.id);
    double torque = -.01*ang - .005*rate;
    dJointAddUniversalTorques(ankleJ.id, torque, 0.);
}

void Hobbelen::SwingHipTorqueControl(LegType leg, double t)
{
    // Inter-leg angle
    double phi = dJointGetUniversalAngle2(fRLeg.HipJ.id) + fRLeg.HipJ.off
        - dJointGetUniversalAngle2(fLLeg.HipJ.id) - fLLeg.HipJ.off;
    double phidot = dJointGetUniversalAngle2Rate(fRLeg.HipJ.id)
        - dJointGetUniversalAngle2Rate(fLLeg.HipJ.id);
    
    if(leg == RightLeg) {
        phi = -phi; phidot = -phidot;
    }
    
    // PD constants
    const double K_P = 3.0;
    const double K_D = 5.0;
    
    const double torque = K_P * (fSwingHipSpl.eval(t) - phi)
        + K_D * (fSwingHipSpl.eval_deriv(t) - phidot);
    
    if(leg == LeftLeg)
        dJointAddUniversalTorques(fLLeg.HipJ.id, 0., -torque);
    else
        dJointAddUniversalTorques(fRLeg.HipJ.id, 0., -torque);
}

void Hobbelen::SwingKneeTorqueControl(const HobJoint& kneeJ, double t)
{
    // Inter-leg angle
    double phi = dJointGetHingeAngle(kneeJ.id) + kneeJ.off;
    double phidot = dJointGetHingeAngleRate(kneeJ.id);
    
    // PD constants
    const double K_P = 0.3;
    const double K_D = 0.5;
    
    const double torque = K_P * (fSwingKneeSpl.eval(t) - phi)
        + K_D * (fSwingKneeSpl.eval_deriv(t) - phidot);
    
    if(t > .2 && t < .8) {
        dJointAddHingeTorque(kneeJ.id, torque);
    }
}


void Hobbelen::Servo(const HobJoint& joint, double desiredAngle)
{
    const double maxV = 1.;
    double vel = dJointGetUniversalAngle1(joint.id) - desiredAngle;
    if(vel < -maxV) vel = -maxV;
    if(vel > maxV) vel = maxV;
    dJointSetUniversalParam(joint.id, dParamVel, vel);
}

void Hobbelen::LegRollControl()
{
    Servo(fLLeg.HipJ, fDesiredLRoll);
    //Servo(fLLeg.AnkleJ, fDesiredLRoll);
    Servo(fRLeg.HipJ, fDesiredRRoll);
    //Servo(fRLeg.AnkleJ, fDesiredRRoll);
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
        Collide(fFloorG, fRLeg.FFootG);
        Collide(fFloorG, fRLeg.BFootG);
        Collide(fFloorG, fLLeg.FFootG);
        Collide(fFloorG, fLLeg.BFootG);
        
        // State machine
        if(fLLegState == Pushoff) {
            if(GetTipClearance(fLLeg.FootB) > 1e-3) {
                fLLegState = Swing;
            }
        } else if(fLLegState == Swing) {
            if(GetHeelClearance(fLLeg.FootB) < 1e-3) {
                fLLegState = Touchdown;
                SetKneeLock(fLLeg.KneeJ, true);
                fRLegState = Pushoff;
                SetKneeLock(fRLeg.KneeJ, false);
                fSwingT = 0.;
            }
        } else if(fLLegState == Touchdown) {
            if(GetTipClearance(fLLeg.FootB) < 1e-3) {
                fLLegState = Stance;
            }
        } else if(fLLegState == Stance) {
            // Remain in stance mode until touchdown of swing leg
        }
        
        if(fRLegState == Pushoff) {
            if(GetTipClearance(fRLeg.FootB) > 1e-3) {
                fRLegState = Swing;
            }
        } else if(fRLegState == Swing) {
            if(GetHeelClearance(fRLeg.FootB) < 1e-3) {
                fRLegState = Touchdown;
                SetKneeLock(fRLeg.KneeJ, true);
                fLLegState = Pushoff;
                SetKneeLock(fLLeg.KneeJ, false);
                fSwingT = 0.;
            }
        } else if(fRLegState == Touchdown) {
            if(GetTipClearance(fRLeg.FootB) < 1e-3) {
                fRLegState = Stance;
            }
        } else if(fRLegState == Stance) {
            // Remain in stance mode until touchdown of swing leg
        }
        
        if(fLLegState == Swing) {
            SwingHipTorqueControl(LeftLeg, fSwingT);
            #ifdef ACTIVE_KNEE
            SwingKneeTorqueControl(fLLeg.KneeJ, fSwingT);
            #endif
        }
        if(fRLegState == Swing) {
            SwingHipTorqueControl(RightLeg, fSwingT);
            #ifdef ACTIVE_KNEE
            SwingKneeTorqueControl(fRLeg.KneeJ, fSwingT);
            #endif
        }
        
        AnklePitchTorqueControl(fLLeg.AnkleJ, fLLegState);
        AnklePitchTorqueControl(fRLeg.AnkleJ, fRLegState);
        
        #ifdef WALKER_3D
        //Servo(fLLeg.HipJ, fDesiredLRoll);
        //Servo(fRLeg.HipJ, fDesiredRRoll);
        
        AnkleRollTorqueControl(fLLeg.AnkleJ);
        AnkleRollTorqueControl(fRLeg.AnkleJ);
        
        if(fT > .5) {
            //fDesiredLRoll = .1;
        }
        #endif
        
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
    
    state.fLULegQ = BodyQ::FromODE(fLLeg.ULegB);
    state.fLLLegQ = BodyQ::FromODE(fLLeg.LLegB);
    state.fLFootQ = BodyQ::FromODE(fLLeg.FootB);
    
    state.fRULegQ = BodyQ::FromODE(fRLeg.ULegB);
    state.fRLLegQ = BodyQ::FromODE(fRLeg.LLegB);
    state.fRFootQ = BodyQ::FromODE(fRLeg.FootB);
    
    state.fLHipAngle = dJointGetUniversalAngle2(fLLeg.HipJ.id) + fLLeg.HipJ.off;
    state.fRHipAngle = dJointGetUniversalAngle2(fRLeg.HipJ.id) + fRLeg.HipJ.off;
    
    if(fLLegState == Swing || fLLegState == Pushoff)
        state.fDesiredInterLeg = fSwingHipSpl.eval(fSwingT);
    else
        state.fDesiredInterLeg = -fSwingHipSpl.eval(fSwingT);
    
    state.fDesiredKneeAngle = fSwingKneeSpl.eval(fSwingT);
    
    state.fLKneeAngle = dJointGetHingeAngle(fLLeg.KneeJ.id) + fLLeg.KneeJ.off;
    state.fRKneeAngle = dJointGetHingeAngle(fRLeg.KneeJ.id) + fRLeg.KneeJ.off;
    
    state.fLAnkleAngle = dJointGetUniversalAngle2(fLLeg.AnkleJ.id) + fLLeg.AnkleJ.off;
    state.fRAnkleAngle = dJointGetUniversalAngle2(fRLeg.AnkleJ.id) + fRLeg.AnkleJ.off;
    
    state.fLTipClear = GetTipClearance(fLLeg.FootB);
    state.fLHeelClear = GetHeelClearance(fLLeg.FootB);
    state.fRTipClear = GetTipClearance(fRLeg.FootB);
    state.fRHeelClear = GetHeelClearance(fRLeg.FootB);
    
    state.fLLegState = fLLegState;
    state.fRLegState = fRLegState;
    
    return state;
}
