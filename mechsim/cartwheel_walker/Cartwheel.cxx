#include "Cartwheel.h"
#include "RotMotion.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include "DynTransform.h"
#include "DynInfo.h"
#include <GL/gl.h>
#include <cmath>

#include <Physics/ContactPoint.h>
#include <Core/TurnController.h>
#include <Core/WorldOracle.h>

// ** Simulation parameters **
// Generate contact joints between the feet and the ground
//#define USE_FOOT_COLLISION

// Lock stance foot once it reaches the ground
#define USE_STANCE_FOOT_LOCKING

// ** Title **
const char Cartwheel::TITLE[] = "Cartwheel Walker";

// ** Display parameters (these do not enter into the simulation) **
// Width of the boxes representing the legs
const double CartState::DISP_LEGWIDTH = .05;
// Width of slide
const double CartState::DISP_SLIDEWIDTH = 1.0;
// Half-length of slide
const int CartState::DISP_SLIDELEN2 = 30;

BodyQ QFromODE(dBodyID id)
{
    return BodyQ(ODE::BodyGetPosition(id),
                 ODE::BodyGetQuaternion(id),
                 ODE::BodyGetLinearVel(id),
                 ODE::BodyGetAngularVel(id));
}

void TransformGL(const BodyQ& q)
{
    glPushMatrix();
    GL::Translate(q.pos());
    GL::Rotate(q.rot());
}

void CartState::Draw(int mode) const
{
    glMatrixMode(GL_MODELVIEW);
    
    if(mode == 0) {
        DrawRobot(false);
        
        GL::shadowsBeginFloor();
        glPushMatrix();
        glScalef(.1, .1, .1);
        GL::drawCheckerboardFloor();
        glPopMatrix();
        GL::shadowsBeginObjects(Eigen::Vector3d::UnitZ(), 0.);
        DrawRobot(true);
        GL::shadowsEnd();
    } else {
        DrawRobotOutline();
    }
    
    glColor3f(1., 1., 0.);
    GL::drawSphere(0.1, fDbg.desSwingPos);
}

void CartState::DrawRobot(bool shadowmode) const
{
    if(!shadowmode) glColor3f(.5, .5, 0.);
    
    /* fTorsoQ.TransformGL();
    glScalef(CharacterConst::torsoSizeX, CharacterConst::torsoSizeY,
        CharacterConst::torsoSizeZ);
    GL::drawUnitCube();
    glPopMatrix();
    
    fLowerBackQ.TransformGL();
    glScalef(CharacterConst::lowerBackSizeX, CharacterConst::lowerBackSizeY,
        CharacterConst::lowerBackSizeZ);
    GL::drawUnitCube();
    glPopMatrix(); */
    
    TransformGL(fFState.q(B_PELVIS));
    glScalef(CharacterConst::pelvisSizeX, CharacterConst::pelvisSizeY, CharacterConst::pelvisSizeZ);
    GL::drawUnitCube();
    glPopMatrix();
    
    if(!shadowmode) glColor3f(.5, 0., .5);
    
    TransformGL(fFState.q(B_L_THIGH));
    GL::drawTube(CharacterConst::legDiameter/2.,
                 -CharacterConst::thighSizeZ / 2. * Eigen::Vector3d::UnitZ(),
                 CharacterConst::thighSizeZ / 2. * Eigen::Vector3d::UnitZ());
    glPopMatrix();
    
    TransformGL(fFState.q(B_L_SHANK));
    GL::drawTube(CharacterConst::legDiameter/2.,
                 -CharacterConst::shankSizeZ / 2. * Eigen::Vector3d::UnitZ(),
                 CharacterConst::shankSizeZ / 2. * Eigen::Vector3d::UnitZ());
    glPopMatrix();
    
    TransformGL(fFState.q(B_R_THIGH));
    GL::drawTube(CharacterConst::legDiameter/2.,
                 -CharacterConst::thighSizeZ / 2. * Eigen::Vector3d::UnitZ(),
                 CharacterConst::thighSizeZ / 2. * Eigen::Vector3d::UnitZ());
    glPopMatrix();
    
    TransformGL(fFState.q(B_R_SHANK));
    GL::drawTube(CharacterConst::legDiameter/2.,
                 -CharacterConst::shankSizeZ / 2. * Eigen::Vector3d::UnitZ(),
                 CharacterConst::shankSizeZ / 2. * Eigen::Vector3d::UnitZ());
    glPopMatrix();
    
    if(!shadowmode) glColor3f(.5, .5, .5);
    
    TransformGL(fFState.q(B_L_FOOT));
    glScalef(CharacterConst::footSizeX, CharacterConst::footSizeY,
        CharacterConst::footSizeZ);
    GL::drawUnitCube();
    /* glBegin(GL_LINE_LOOP);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);
    glVertex3f(-.5, -.5,  .5);
    glEnd(); */
    glPopMatrix();
    
    TransformGL(fFState.q(B_R_FOOT));
    glScalef(CharacterConst::footSizeX, CharacterConst::footSizeY,
        CharacterConst::footSizeZ);
    GL::drawUnitCube();
    glPopMatrix();
}

void CartState::DrawRobotOutline() const
{
    glDisable(GL_LIGHTING);
    
    glColor3f(0., .2, 0.);
    glPushMatrix();
    glScalef(.1, .1, .1);
    GL::drawCheckerboardFloorOutline();
    glPopMatrix();
    
    glPushMatrix();
    GL::Rotate(Eigen::AngleAxis<double>(M_PI/2., Eigen::Vector3d::UnitX()));
    
    glColor3f(1., 1., 1.);
    
    TransformGL(fFState.q(B_L_FOOT));
    glScalef(CharacterConst::footSizeX, CharacterConst::footSizeY,
        CharacterConst::footSizeZ);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);
    glVertex3f(-.5, -.5,  .5);
    glEnd();
    glPopMatrix();
    
    TransformGL(fFState.q(B_R_FOOT));
    glScalef(CharacterConst::footSizeX, CharacterConst::footSizeY,
        CharacterConst::footSizeZ);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);
    glVertex3f(-.5, -.5,  .5);
    glEnd();
    glPopMatrix();
    
    glBegin(GL_LINE_STRIP);
    GL::Vertex3(fJPos[J_L_ANKLE]);
    GL::Vertex3(fJPos[J_L_KNEE]);
    GL::Vertex3(fJPos[J_L_HIP]);
    GL::Vertex3(fJPos[J_R_HIP]);
    GL::Vertex3(fJPos[J_R_KNEE]);
    GL::Vertex3(fJPos[J_R_ANKLE]);
    glEnd();
    
    /* const double scale = 0.01;
    glColor3f(1., 1., 0.);
    glBegin(GL_LINES);
    for(int jid=0; jid<J_MAX; jid++) {
        GL::Vertex3(fJPos[jid]);
        GL::Vertex3(fJPos[jid]+scale*fJTorques[jid]);
    } */
    glEnd();
    
    glPopMatrix();
}

Cartwheel::Cartwheel(unsigned int stepPerSec, unsigned int intPerStep)
    : fStepPerSec(stepPerSec),
      fIntPerStep(intPerStep)   // Integration intervals per timestep
{
    fT = 0.;
    fPint = 0.;
    
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    //fFloorG = dCreatePlane(0, FloorNormal.x(), FloorNormal.y(), FloorNormal.z(),
    //                       -FLOOR_DIST);
    fFloorG = dCreatePlane(0, 0., 0., 1., 0.);
    
    fRobot = new CWRobot();
    fRobot->create(fWorld);
    
    WorldOracle* worldOracle = new WorldOracle();
    
    fController = new CWController(worldOracle);
    
    fContactGroup = dJointGroupCreate(0);
    
    #ifdef USE_STANCE_FOOT_LOCKING
    fStance = LEFT_STANCE;
    fLastStanceSwitchTime = 0.;
    fLFootStickyJ = fRFootStickyJ = 0;
    
    LockStanceFoot(LEFT_STANCE);
    #endif
}

Cartwheel::~Cartwheel()
{
    dJointGroupDestroy(fContactGroup);
    dWorldDestroy(fWorld);
}

unsigned int Cartwheel::Collide(dGeomID g1, dGeomID g2, std::vector<ContactPoint>& cps,
    dJointFeedback* feedback)
{
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    
    cps.clear();
    cps.reserve(num_contacts);
    
    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactBounce | dContactApprox1;
        contact[i].surface.bounce = 0.0;
        contact[i].surface.bounce_vel = 1000.0;
        contact[i].surface.mu = 1000.0;
        dJointID c = dJointCreateContact(fWorld, fContactGroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
        
        cps.push_back(ContactPoint());
        cps.at(i).cp = Point3d(contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2]);
        dJointSetFeedback(c,&(feedback[i]));
    }
    
    return num_contacts;
}

void Cartwheel::LockStanceFoot(int stance)
{
    if(fLFootStickyJ != 0) {
        dJointDestroy(fLFootStickyJ);
        fLFootStickyJ = 0;
    }
    if(fRFootStickyJ != 0) {
        dJointDestroy(fRFootStickyJ);
        fRFootStickyJ = 0;
    }
    
    if(stance == LEFT_STANCE) {
        fLFootStickyJ = dJointCreateFixed(fWorld, 0);
        dJointAttach(fLFootStickyJ, 0, fRobot->fBodies[B_L_FOOT]);
        dJointSetFixed(fLFootStickyJ);
        dJointSetFeedback(fLFootStickyJ, &fLeftFeedback[0]);
    } else {
        fRFootStickyJ = dJointCreateFixed(fWorld, 0);
        dJointAttach(fRFootStickyJ, 0, fRobot->fBodies[B_R_FOOT]);
        dJointSetFixed(fRFootStickyJ);
        dJointSetFeedback(fRFootStickyJ, &fRightFeedback[0]);
    }
}

void Cartwheel::SetFakeContactDataForFoot(std::vector<ContactPoint>& cpts, const Vector3d& pos)
{
    cpts.push_back(ContactPoint());
    cpts.at(0).f = Vector3d(0., 0., 100.);
    cpts.at(0).cp = pos + Vector3d(1., 1., 0.);
    
    cpts.push_back(ContactPoint());
    cpts.at(1).f = Vector3d(0., 0., 100.);
    cpts.at(1).cp = pos + Vector3d(1., -1., 0.);
    
    cpts.push_back(ContactPoint());
    cpts.at(2).f = Vector3d(0., 0., 100.);
    cpts.at(2).cp = pos + Vector3d(-1., 1., 0.);
    
    cpts.push_back(ContactPoint());
    cpts.at(3).f = Vector3d(0., 0., 100.);
    cpts.at(3).cp = Vector3d(-1., -1., 0.);
}

void Cartwheel::SetFakeContactData(int stance)
{
    fCData.pLeft.clear();
    fCData.pRight.clear();
    
    if(stance == LEFT_STANCE) {
        fCData.lFtot = Eigen::Vector3d(fLeftFeedback[0].f1[0], fLeftFeedback[0].f1[1], fLeftFeedback[0].f1[2]);
        fCData.lTtot = Eigen::Vector3d(fLeftFeedback[0].t1[0], fLeftFeedback[0].t1[1], fLeftFeedback[0].t1[2]);
        fCData.rFtot = Eigen::Vector3d(0., 0., 0.);
        fCData.rTtot = Eigen::Vector3d(0., 0., 0.);
        
        SetFakeContactDataForFoot(fCData.pLeft, ODE::BodyGetPosition(fRobot->fBodies[B_L_FOOT]));
    } else {
        fCData.lFtot = Eigen::Vector3d(0., 0., 0.);
        fCData.lTtot = Eigen::Vector3d(0., 0., 0.);
        fCData.rFtot = Eigen::Vector3d(fRightFeedback[0].f1[0], fRightFeedback[0].f1[1], fRightFeedback[0].f1[2]);
        fCData.rTtot = Eigen::Vector3d(fRightFeedback[0].t1[0], fRightFeedback[0].t1[1], fRightFeedback[0].t1[2]);
        
        SetFakeContactDataForFoot(fCData.pRight, ODE::BodyGetPosition(fRobot->fBodies[B_R_FOOT]));
    }
}

void Cartwheel::ApplyTorques(const JSpTorques& jt)
{
    Eigen::Quaterniond lHipRot = ODE::BodyGetQuaternion(fRobot->fBodies[B_PELVIS]).conjugate() *
        ODE::BodyGetQuaternion(fRobot->fBodies[B_L_THIGH]);
    double lhz, lhy, lhx;
    decompZYXRot(lHipRot, lhz, lhy, lhx);
    Eigen::Vector3d lHipTorque = transformHipTorque(lhz, lhy, lhx,
        jt.t(LEFT, HZ), jt.t(LEFT, HY), jt.t(LEFT, HX));
    lHipTorque = ODE::BodyGetQuaternion(fRobot->fBodies[B_PELVIS])._transformVector(lHipTorque);
    ODE::BodyAddTorque(fRobot->fBodies[B_PELVIS], -lHipTorque);
    ODE::BodyAddTorque(fRobot->fBodies[B_L_THIGH], lHipTorque);
    
    dJointAddHingeTorque(fRobot->fLKneeJ, jt.t(LEFT, KY));
    dJointAddUniversalTorques(fRobot->fLAnkleJ, jt.t(LEFT, AX), jt.t(LEFT, AY));
    
    Eigen::Quaterniond rHipRot = ODE::BodyGetQuaternion(fRobot->fBodies[B_PELVIS]).conjugate() *
        ODE::BodyGetQuaternion(fRobot->fBodies[B_R_THIGH]);
    double rhz, rhy, rhx;
    decompZYXRot(rHipRot, rhz, rhy, rhx);
    Eigen::Vector3d rHipTorque = transformHipTorque(rhz, rhy, rhx,
        jt.t(RIGHT, HZ), jt.t(RIGHT, HY), jt.t(RIGHT, HX));
    rHipTorque = ODE::BodyGetQuaternion(fRobot->fBodies[B_PELVIS])._transformVector(rHipTorque);
    ODE::BodyAddTorque(fRobot->fBodies[B_PELVIS], -rHipTorque);
    ODE::BodyAddTorque(fRobot->fBodies[B_R_THIGH], rHipTorque);
    
    dJointAddHingeTorque(fRobot->fRKneeJ, jt.t(RIGHT, KY));
    dJointAddUniversalTorques(fRobot->fRAnkleJ, jt.t(RIGHT, AX), jt.t(RIGHT, AY));
}

void Cartwheel::AdvanceInTime(double dt, const JSpTorques& torques)
{
    #ifdef USE_STANCE_FOOT_LOCKING
    if((fT - fLastStanceSwitchTime) > 0.5) {
        dBodyID swingFootB = (fStance == LEFT_STANCE ? fRobot->fBodies[B_R_FOOT] : fRobot->fBodies[B_L_FOOT]);
        if(ODE::BodyGetPosition(swingFootB).z() < CharacterConst::footPosZ) {
            fStance = (fStance == LEFT_STANCE ? RIGHT_STANCE : LEFT_STANCE);
            fLastStanceSwitchTime = fT;
            LockStanceFoot(fStance);
        }
    }
    #endif
    
    //go through all the joints in the world, and apply their torques to the parent and child rb's
    ApplyTorques(torques);
    
    //we need to determine the contact points first - delete the previous contacts
    dJointGroupEmpty(fContactGroup);
    //initiate the collision detection
    #ifdef USE_FOOT_COLLISION
    Collide(fRobot->fLFootG, fFloorG, fCData.pLeft, fLeftFeedback);
    Collide(fRobot->fRFootG, fFloorG, fCData.pRight, fRightFeedback);
    #endif
    
    fCData.lPos = ODE::BodyGetPosition(fRobot->fBodies[B_L_FOOT]);
    fCData.rPos = ODE::BodyGetPosition(fRobot->fBodies[B_R_FOOT]);
    
    //advance the simulation
    dWorldStep(fWorld, dt);
    
    #ifdef USE_FOOT_COLLISION
    //copy over the force information for the contact forces
    assert(fCData.pLeft.size() < MAX_CONTACTS);
    assert(fCData.pRight.size() < MAX_CONTACTS);
    
    fCData.lFtot = Eigen::Vector3d::Zero();
    fCData.lTtot = Eigen::Vector3d::Zero();
    fCData.rFtot = Eigen::Vector3d::Zero();
    fCData.rTtot = Eigen::Vector3d::Zero();
    
    for (unsigned int i=0; i<fCData.pLeft.size(); i++) {
        fCData.pLeft[i].f = Vector3d(fLeftFeedback[i].f1[0], fLeftFeedback[i].f1[1], fLeftFeedback[i].f1[2]);
        
        Eigen::Vector3d f(fLeftFeedback[i].f1[0], fLeftFeedback[i].f1[1], fLeftFeedback[i].f1[2]);
        Eigen::Vector3d t(fLeftFeedback[i].t1[0], fLeftFeedback[i].t1[1], fLeftFeedback[i].t1[2]);
        /* BEGIN test */
        Eigen::Vector3d r(fCData.pLeft[i].cp.x(), fCData.pLeft[i].cp.y(), fCData.pLeft[i].cp.z());
        Eigen::Vector3d tp = (r - fCData.lPos).cross(f);
        assert((tp - t).squaredNorm() < 1e-12);
        assert(std::abs(r.z()) < 1e-3);
        /* END test */
        
        fCData.lFtot += f;
        fCData.lTtot += t;
    }
    for (unsigned int i=0; i<fCData.pRight.size(); i++) {
        fCData.pRight[i].f = Vector3d(fRightFeedback[i].f1[0], fRightFeedback[i].f1[1], fRightFeedback[i].f1[2]);
        
        Eigen::Vector3d f(fRightFeedback[i].f1[0], fRightFeedback[i].f1[1], fRightFeedback[i].f1[2]);
        Eigen::Vector3d t(fRightFeedback[i].t1[0], fRightFeedback[i].t1[1], fRightFeedback[i].t1[2]);
        
        fCData.rFtot += f;
        fCData.rTtot += t;
    }
    #endif
    
    #ifdef USE_STANCE_FOOT_LOCKING
    SetFakeContactData(fStance);
    #endif
}

JSpTorques getTestJSpTorques(double t)
{
    JSpTorques torques = JSpTorques::Zero();
    torques.t(LAX) = 2.3;
    torques.t(LAY) = -1.5;
    torques.t(LKY) = -0.9;
    torques.t(LHZ) = 1.0 * cos(6*t);
    torques.t(LHY) = 1.4 * cos(4*t);
    torques.t(LHX) = 2.5 * cos(9*t);
    
    torques.t(RHZ) = 0.02 * sin(4*t);
    torques.t(RHY) = -0.5 * sin(5*t);
    torques.t(RHX) = 0.1 * sin(7*t);
    torques.t(RKY) = -0.02;
    torques.t(RAY) = 0.01;
    torques.t(RAX) = 0.006;
    
    return torques;
}

void Cartwheel::Advance()
{
    const double dt = 1./(fStepPerSec * fIntPerStep);
    
    for(int i=0; i<fIntPerStep; ++i) {
        //dBodyAddForce(fRobot->fPelvisB, 40.*sin(fT), 40.*cos(fT), 0.);
        
        FullState fstate;
        for(unsigned int b=0; b<B_MAX; b++)
            fstate.q(b) = QFromODE(fRobot->fBodies[b]);
        JSpState jstate = jointFromFull(fstate);
        
        const double desiredHeading = fT/4.;
        JSpTorques torques = fController->Run(dt, fstate, jstate, fCData, desiredHeading);
        
        fPint += Psys(torques, jstate) * dt;
        
        AdvanceInTime(dt, torques);
        fT += dt;
        
        fDebugJTorques = torques;
    }
}

CartState Cartwheel::GetCurrentState()
{
    CartState state;
    
    state.fParent = this;
    for(unsigned int b=0; b<B_MAX; b++)
        state.fFState.q(b) = QFromODE(fRobot->fBodies[b]);
    
    state.fJState = jointFromFull(state.fFState);
    
    double tmp[4];
    dJointGetUniversalAnchor(fRobot->fLAnkleJ, tmp);
    state.fJPos[J_L_ANKLE] = Eigen::Vector3d(tmp);
    dJointGetHingeAnchor(fRobot->fLKneeJ, tmp);
    state.fJPos[J_L_KNEE] = Eigen::Vector3d(tmp);
    dJointGetBallAnchor(fRobot->fLHipJ, tmp);
    state.fJPos[J_L_HIP] = Eigen::Vector3d(tmp);
    dJointGetUniversalAnchor(fRobot->fRAnkleJ, tmp);
    state.fJPos[J_R_ANKLE] = Eigen::Vector3d(tmp);
    dJointGetHingeAnchor(fRobot->fRKneeJ, tmp);
    state.fJPos[J_R_KNEE] = Eigen::Vector3d(tmp);
    dJointGetBallAnchor(fRobot->fRHipJ, tmp); 
    state.fJPos[J_R_HIP] = Eigen::Vector3d(tmp);
    
    state.fTorques = fDebugJTorques;
    
    state.fPint = fPint;
    
    state.fDbg = fController->fDbg;
    
    return state;
}
