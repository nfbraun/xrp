#include "Cartwheel.h"
#include "RotMotion.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include "DynTransform.h"
#include "DynInfo.h"
#include "Reaction.h"
#include <GL/gl.h>
#include <cmath>

#include <Physics/ContactPoint.h>
#include <Core/TurnController.h>
#include <Core/WorldOracle.h>

// ** Simulation parameters **
// Generate contact joints between the feet and the ground
#define USE_FOOT_COLLISION

// Lock stance foot once it reaches the ground
//#define USE_STANCE_FOOT_LOCKING

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
    
    const double dt = 1./(fStepPerSec * fIntPerStep);
    fMotorModel[LHZ].setTau(0., dt);
    fMotorModel[LHY].setTau(0., dt);
    fMotorModel[LHX].setTau(0., dt);
    fMotorModel[LKY].setTau(0., dt);
    fMotorModel[LAY].setTau(0., dt);
    fMotorModel[LAX].setTau(0., dt);
    
    fMotorModel[RHZ].setTau(0., dt);
    fMotorModel[RHY].setTau(0., dt);
    fMotorModel[RHX].setTau(0., dt);
    fMotorModel[RKY].setTau(0., dt);
    fMotorModel[RAY].setTau(0., dt);
    fMotorModel[RAX].setTau(0., dt);
}

Cartwheel::~Cartwheel()
{
    dJointGroupDestroy(fContactGroup);
    dWorldDestroy(fWorld);
}

unsigned int Cartwheel::Collide(dGeomID g1, dGeomID g2,
    Eigen::Matrix<ContactPoint, Eigen::Dynamic, 1>& cps,
    dJointFeedback* feedback)
{
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    
    cps.resize(num_contacts, 1);
    
    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = 1.0;
        dJointID c = dJointCreateContact(fWorld, fContactGroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
        
        cps(i).cp = Eigen::Vector3d(contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2]);
        dJointSetFeedback(c,&(feedback[i]));
    }
    
    return num_contacts;
}

void Cartwheel::GetContactMask(unsigned int& lContactMask, unsigned int& rContactMask, const FullState& fstate) const
{
    lContactMask = 0;
    rContactMask = 0;
    
    for (unsigned int i=0; i<fCData.pLeft.size(); i++) {
        Eigen::Vector3d tmpP = fstate.trToLocal(B_L_FOOT).onPoint(fCData.pLeft[i].cp);
        
        if(std::abs(std::abs(tmpP.x()) - CharacterConst::footSizeX/2.) > 1e-3
          || std::abs(std::abs(tmpP.y()) - CharacterConst::footSizeY/2.) > 1e-3) {
            std::cerr << "WARNING: strange contact encountered.";
            std::cerr << "   x=" << tmpP.x() << " y=" << tmpP.y() << std::endl;
        }
        
        if (tmpP.x() > 0 && tmpP.y() > 0) lContactMask |= 0x08;
        if (tmpP.x() > 0 && tmpP.y() < 0) lContactMask |= 0x04;
        if (tmpP.x() < 0 && tmpP.y() > 0) lContactMask |= 0x02;
        if (tmpP.x() < 0 && tmpP.y() < 0) lContactMask |= 0x01;
    }
    
    for (unsigned int i=0; i<fCData.pRight.size(); i++) {
        Eigen::Vector3d tmpP = fstate.trToLocal(B_R_FOOT).onPoint(fCData.pRight[i].cp);
        
        if(std::abs(std::abs(tmpP.x()) - CharacterConst::footSizeX/2.) > 1e-3
          || std::abs(std::abs(tmpP.y()) - CharacterConst::footSizeY/2.) > 1e-3) {
            std::cerr << "WARNING: strange contact encountered.";
            std::cerr << "   x=" << tmpP.x() << " y=" << tmpP.y() << std::endl;
        }
        
        if (tmpP.x() > 0 && tmpP.y() > 0) rContactMask |= 0x08;
        if (tmpP.x() > 0 && tmpP.y() < 0) rContactMask |= 0x04;
        if (tmpP.x() < 0 && tmpP.y() > 0) rContactMask |= 0x02;
        if (tmpP.x() < 0 && tmpP.y() < 0) rContactMask |= 0x01;
    }
}

unsigned int Cartwheel::TransformContactMask(unsigned int mask) const
{
    /*
        0            - 0 contacts active
        1,2,3,4      - 1 contact active
        5,6,7,8,9,10 - 2 contacts active
        11,12,13,14  - 3 contacts active
        15           - 4 contacts active
    */

    unsigned int lookup[16] =
        { 0, 1, 2, 5, 3, 6, 7, 11, 4, 8, 9, 12, 10, 13, 14, 15 };
    return lookup[mask & 0xF];
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

void Cartwheel::SetFakeContactDataForFoot(Eigen::Matrix<ContactPoint, Eigen::Dynamic, 1>& cpts, const Eigen::Vector3d& pos)
{
    cpts.resize(4, 1);
    cpts(0).f = Eigen::Vector3d(0., 0., 100.);
    cpts(0).cp = pos + Eigen::Vector3d(1., 1., 0.);
    
    cpts(1).f = Eigen::Vector3d(0., 0., 100.);
    cpts(1).cp = pos + Eigen::Vector3d(1., -1., 0.);
    
    cpts(2).f = Eigen::Vector3d(0., 0., 100.);
    cpts(2).cp = pos + Eigen::Vector3d(-1., 1., 0.);
    
    cpts(3).f = Eigen::Vector3d(0., 0., 100.);
    cpts(3).cp = pos + Eigen::Vector3d(-1., -1., 0.);
}

void Cartwheel::SetFakeContactData(int stance)
{
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
    
    JSpTorques filtTorques;
    for(unsigned int i=0; i<DOF_MAX; i++)
        filtTorques.t(i) = fMotorModel[i].outTorque(torques.t(i));
    
    //go through all the joints in the world, and apply their torques to the parent and child rb's
    ApplyTorques(filtTorques);
    
    fCtrlTorques = torques;
    fFiltTorques = filtTorques;
    
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
        Eigen::Vector3d f(fLeftFeedback[i].f1[0], fLeftFeedback[i].f1[1], fLeftFeedback[i].f1[2]);
        Eigen::Vector3d t(fLeftFeedback[i].t1[0], fLeftFeedback[i].t1[1], fLeftFeedback[i].t1[2]);
        
        fCData.pLeft[i].f = f;
        
        fCData.lFtot += f;
        fCData.lTtot += t;
    }
    for (unsigned int i=0; i<fCData.pRight.size(); i++) {
        Eigen::Vector3d f(fRightFeedback[i].f1[0], fRightFeedback[i].f1[1], fRightFeedback[i].f1[2]);
        Eigen::Vector3d t(fRightFeedback[i].t1[0], fRightFeedback[i].t1[1], fRightFeedback[i].t1[2]);
        
        fCData.pRight[i].f = f;
        
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
    }
}

CartState Cartwheel::GetCurrentState()
{
    CartState state;
    
    state.fParent = this;
    for(unsigned int b=0; b<B_MAX; b++)
        state.fFState.q(b) = QFromODE(fRobot->fBodies[b]);
    
    state.fJState = jointFromFull(state.fFState);
    
    state.fLF = fCData.lFtot;
    state.fLT = fCData.lTtot;
    state.fRF = fCData.rFtot;
    state.fRT = fCData.rTtot;
    
    state.fStance = fController->fDbg.stance;
    
    calcReaction(state.fStF_pred, state.fStT_pred, state.fFState, state.fJState, fCtrlTorques, fController->fDbg.stance);
    
    #ifdef USE_FOOT_COLLISION
    GetContactMask(state.fLContactMask, state.fRContactMask, state.fFState);
    state.fLContacts = TransformContactMask(state.fLContactMask);
    state.fRContacts = TransformContactMask(state.fRContactMask);
    #else
    state.fLContactMask = (fStance == LEFT_STANCE) ? 0xF : 0x0;
    state.fRContactMask = (fStance == LEFT_STANCE) ? 0x0 : 0xF;
    state.fLContacts = (fStance == LEFT_STANCE) ? 15 : 0;
    state.fRContacts = (fStance == LEFT_STANCE) ? 0 : 15;
    #endif
    
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
    
    state.fCtrlTorques = fCtrlTorques;
    state.fFiltTorques = fFiltTorques;
    
    state.fPint = fPint;
    
    state.fDbg = fController->fDbg;
    
    return state;
}
