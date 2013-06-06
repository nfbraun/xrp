#include "Cartwheel.h"
#include "RotMotion.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include "DynTransform.h"
#include <GL/gl.h>
#include <cmath>

#include <Physics/ContactPoint.h>
#include <Physics/RigidBody.h>
#include <Core/TurnController.h>
#include <Core/WorldOracle.h>

const char* PhiNames[] = { "LH0", "LH1", "LH2", "LK", "LA0", "LA1",
                           "RH0", "RH1", "RH2", "RK", "RA0", "RA1" };

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
    GL::drawSphere(0.1, fDbg.desSwingPos.toEigen());
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
    
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    //fFloorG = dCreatePlane(0, FloorNormal.x(), FloorNormal.y(), FloorNormal.z(),
    //                       -FLOOR_DIST);
    fFloorG = dCreatePlane(0, 0., 0., 1., 0.);
    
    fRobot = new CWRobot();
    fRobot->create(fWorld);
    fCharacter = fRobot->fCharacter;
    fCharacter->computeMass();
    
    fFloorRB = new RigidBody();
    //fFloorRB->lockBody(true);
    //fFloorRB->setFrictionCoefficient(2.5);
    //fFloorRB->setRestitutionCoefficient(0.35);
    
    WorldOracle* worldOracle = new WorldOracle();
    
    fController = new CWController(fCharacter, worldOracle);
    
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

Eigen::Vector3d invTransformTorque(const dReal* R, double t0, double t1, double t2)
{
    Eigen::Vector3d v;
    
    v(0) = R[0] * t0 + R[1] * t1 + R[2] * t2;
    v(1) = R[4] * t0 + R[5] * t1 + R[6] * t2;
    v(2) = R[8] * t0 + R[9] * t1 + R[10] * t2;
    
    return v;
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
        fCData.lFtot = Vector3d(fLeftFeedback[0].f1[0], fLeftFeedback[0].f1[1], fLeftFeedback[0].f1[2]);
        fCData.lTtot = Vector3d(fLeftFeedback[0].t1[0], fLeftFeedback[0].t1[1], fLeftFeedback[0].t1[2]);
        fCData.rFtot = Vector3d(0., 0., 0.);
        fCData.rTtot = Vector3d(0., 0., 0.);
        
        SetFakeContactDataForFoot(fCData.pLeft, ODE::BodyGetPosition(fRobot->fBodies[B_L_FOOT]));
    } else {
        fCData.lFtot = Vector3d(0., 0., 0.);
        fCData.lTtot = Vector3d(0., 0., 0.);
        fCData.rFtot = Vector3d(fRightFeedback[0].f1[0], fRightFeedback[0].f1[1], fRightFeedback[0].f1[2]);
        fCData.rTtot = Vector3d(fRightFeedback[0].t1[0], fRightFeedback[0].t1[1], fRightFeedback[0].t1[2]);
        
        SetFakeContactDataForFoot(fCData.pRight, ODE::BodyGetPosition(fRobot->fBodies[B_R_FOOT]));
    }
}

void Cartwheel::ApplyTorques(const JointSpTorques& jt)
{
    Eigen::Vector3d lHipTorque = invTransformTorque(dBodyGetRotation(fRobot->fBodies[B_PELVIS]),
        jt.fLeftLeg[0], jt.fLeftLeg[1], jt.fLeftLeg[2]);
    ODE::BodyAddTorque(fRobot->fBodies[B_PELVIS], lHipTorque);
    ODE::BodyAddTorque(fRobot->fBodies[B_L_THIGH], -lHipTorque);
    
    dJointAddHingeTorque(fRobot->fLKneeJ, -jt.fLeftLeg[3]);
    dJointAddUniversalTorques(fRobot->fLAnkleJ, -jt.fLeftLeg[4], -jt.fLeftLeg[5]);
    
    Eigen::Vector3d rHipTorque = invTransformTorque(dBodyGetRotation(fRobot->fBodies[B_PELVIS]),
        jt.fRightLeg[0], jt.fRightLeg[1], jt.fRightLeg[2]);
    ODE::BodyAddTorque(fRobot->fBodies[B_PELVIS], rHipTorque);
    ODE::BodyAddTorque(fRobot->fBodies[B_R_THIGH], -rHipTorque);
    
    dJointAddHingeTorque(fRobot->fRKneeJ, -jt.fRightLeg[3]);
    dJointAddUniversalTorques(fRobot->fRAnkleJ, -jt.fRightLeg[4], -jt.fRightLeg[5]);
}

void Cartwheel::AdvanceInTime(double dt, const JointSpTorques& torques)
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
    
    //copy over the state of the ODE bodies to the rigid bodies...
    for(int rid=0; rid<B_MAX; rid++)
    {
        ArticulatedRigidBody* rb = fCharacter->getARBs()[rid];
        setRBState(rb, QFromODE(fRobot->fBodies[rid]));
    }
    
    #ifdef USE_FOOT_COLLISION
    //copy over the force information for the contact forces
    assert(fCData.pLeft.size() < MAX_CONTACTS);
    assert(fCData.pRight.size() < MAX_CONTACTS);
    
    fCData.lFtot = Vector3d(0., 0., 0.);
    fCData.lTtot = Vector3d(0., 0., 0.);
    fCData.rFtot = Vector3d(0., 0., 0.);
    fCData.rTtot = Vector3d(0., 0., 0.);
    
    for (unsigned int i=0; i<fCData.pLeft.size(); i++) {
        fCData.pLeft[i].f = Vector3d(fLeftFeedback[i].f1[0], fLeftFeedback[i].f1[1], fLeftFeedback[i].f1[2]);
        
        Vector3d f(fLeftFeedback[i].f1[0], fLeftFeedback[i].f1[1], fLeftFeedback[i].f1[2]);
        Vector3d t(fLeftFeedback[i].t1[0], fLeftFeedback[i].t1[1], fLeftFeedback[i].t1[2]);
        /* BEGIN test */
        Vector3d r = Eigen::Vector3d(fCData.pLeft[i].cp.x(), fCData.pLeft[i].cp.y(), fCData.pLeft[i].cp.z());
        Vector3d tp = (r - fCData.lPos).cross(f);
        assert((tp - t).squaredNorm() < 1e-12);
        assert(std::abs(r.z()) < 1e-3);
        /* END test */
        
        fCData.lFtot += f;
        fCData.lTtot += t;
    }
    for (unsigned int i=0; i<fCData.pRight.size(); i++) {
        fCData.pRight[i].f = Vector3d(fRightFeedback[i].f1[0], fRightFeedback[i].f1[1], fRightFeedback[i].f1[2]);
        
        Vector3d f(fRightFeedback[i].f1[0], fRightFeedback[i].f1[1], fRightFeedback[i].f1[2]);
        Vector3d t(fRightFeedback[i].t1[0], fRightFeedback[i].t1[1], fRightFeedback[i].t1[2]);
        
        fCData.rFtot += f;
        fCData.rTtot += t;
    }
    #endif
    
    #ifdef USE_STANCE_FOOT_LOCKING
    SetFakeContactData(fStance);
    #endif
}

void Cartwheel::setRBState(RigidBody* rb, const BodyQ& q)
{
    rb->state.position = q.pos();
    
    rb->state.orientation.s = q.rot().w();
    rb->state.orientation.v.x() = q.rot().x();
    rb->state.orientation.v.y() = q.rot().y();
    rb->state.orientation.v.z() = q.rot().z();
    
    rb->state.velocity = q.vel();
    
    rb->state.angularVelocity = q.avel();
}

void Cartwheel::Advance()
{
    const double dt = 1./(fStepPerSec * fIntPerStep);
    
    for(int i=0; i<fIntPerStep; ++i) {
        //dBodyAddForce(fRobot->fPelvisB, 40.*sin(fT), 40.*cos(fT), 0.);
        
        const double desiredHeading = fT/4.;
        JointSpTorques torques = fController->Run(dt, fCData, desiredHeading);
        
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
    
    /*** left leg ***/
    state.fTorques[LH0] = fDebugJTorques.fLeftLeg[0];
    state.fTorques[LH1] = fDebugJTorques.fLeftLeg[1];
    state.fTorques[LH2] = fDebugJTorques.fLeftLeg[2];
    state.fTorques[LK]  = fDebugJTorques.fLeftLeg[3];
    state.fTorques[LA0] = fDebugJTorques.fLeftLeg[4];
    state.fTorques[LA1] = fDebugJTorques.fLeftLeg[5];
    
    /*** right leg ***/
    state.fTorques[RH0] = fDebugJTorques.fRightLeg[0];
    state.fTorques[RH1] = fDebugJTorques.fRightLeg[1];
    state.fTorques[RH2] = fDebugJTorques.fRightLeg[2];
    state.fTorques[RK]  = fDebugJTorques.fRightLeg[3];
    state.fTorques[RA0] = fDebugJTorques.fRightLeg[4];
    state.fTorques[RA1] = fDebugJTorques.fRightLeg[5];
    
    state.fDbg = fController->fDbg;
    
    return state;
}
