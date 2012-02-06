#include "Cartwheel.h"
#include "RotMotion.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include <GL/gl.h>
#include <cmath>

#include <Physics/ContactPoint.h>
#include <Physics/RigidBody.h>
#include <Core/TurnController.h>
#include <Core/WorldOracle.h>

//#define DEBUG_FIXED_TORSO

// ** Simulation parameters **
const int Cartwheel::STEP_PER_SEC = 25;
const int Cartwheel::INT_PER_STEP = 100;   // Integration intervals per timestep
const char Cartwheel::TITLE[] = "Cartwheel Walker";

// ** Display parameters (these do not enter into the simulation) **
// Width of the boxes representing the legs
const double CartState::DISP_LEGWIDTH = .05;
// Width of slide
const double CartState::DISP_SLIDEWIDTH = 1.0;
// Half-length of slide
const int CartState::DISP_SLIDELEN2 = 30;

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
}

void CartState::DrawRobot(bool shadowmode) const
{
    glPushMatrix();
    GL::Rotate(Eigen::AngleAxis<double>(M_PI/2., Eigen::Vector3d::UnitX()));
    
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
    
    fPelvisQ.TransformGL();
    glScalef(CharacterConst::pelvisSizeX, CharacterConst::pelvisSizeY, CharacterConst::pelvisSizeZ);
    GL::drawUnitCube();
    glPopMatrix();
    
    if(!shadowmode) glColor3f(.5, 0., .5);
    
    fLUpperLegQ.TransformGL();
    GL::drawTube(CharacterConst::legDiameter/2.,
                 CharacterConst::upperLegSizeY / 2. * Eigen::Vector3d::UnitY(),
                 -CharacterConst::upperLegSizeY / 2. * Eigen::Vector3d::UnitY());
    glPopMatrix();
    
    fLLowerLegQ.TransformGL();
    GL::drawTube(CharacterConst::legDiameter/2.,
                 CharacterConst::lowerLegSizeY / 2. * Eigen::Vector3d::UnitY(),
                 -CharacterConst::lowerLegSizeY / 2. * Eigen::Vector3d::UnitY());
    glPopMatrix();
    
    fRUpperLegQ.TransformGL();
    GL::drawTube(CharacterConst::legDiameter/2.,
                 CharacterConst::upperLegSizeY / 2. * Eigen::Vector3d::UnitY(),
                 -CharacterConst::upperLegSizeY / 2. * Eigen::Vector3d::UnitY());
    glPopMatrix();
    
    fRLowerLegQ.TransformGL();
    GL::drawTube(CharacterConst::legDiameter/2.,
                 CharacterConst::lowerLegSizeY / 2. * Eigen::Vector3d::UnitY(),
                 -CharacterConst::lowerLegSizeY / 2. * Eigen::Vector3d::UnitY());
    glPopMatrix();
    
    if(!shadowmode) glColor3f(.5, .5, .5);
    
    fLFootQ.TransformGL();
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
    
    fRFootQ.TransformGL();
    glScalef(CharacterConst::footSizeX, CharacterConst::footSizeY,
        CharacterConst::footSizeZ);
    GL::drawUnitCube();
    glPopMatrix();
    
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
    
    fLFootQ.TransformGL();
    glScalef(CharacterConst::footSizeX, CharacterConst::footSizeY,
        CharacterConst::footSizeZ);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);
    glVertex3f(-.5, -.5,  .5);
    glEnd();
    glPopMatrix();
    
    fRFootQ.TransformGL();
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
    
    const double scale = 0.01;
    glColor3f(1., 1., 0.);
    glBegin(GL_LINES);
    for(int jid=0; jid<J_MAX; jid++) {
        GL::Vertex3(fJPos[jid]);
        GL::Vertex3(fJPos[jid]+scale*fJTorques[jid]);
    }
    glEnd();
    
    glPopMatrix();
}

Cartwheel::Cartwheel()
{
    fT = 0.;
    
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., -9.81, 0.);
    
    //fFloorG = dCreatePlane(0, FloorNormal.x(), FloorNormal.y(), FloorNormal.z(),
    //                       -FLOOR_DIST);
    fFloorG = dCreatePlane(0, 0., 1., 0., 0.);
    
    fRobot = new CWRobot();
    fRobot->create(fWorld);
    fCharacter = fRobot->fCharacter;
    fCharacter->computeMass();
    
    fFloorRB = new RigidBody();
    //fFloorRB->lockBody(true);
    //fFloorRB->setFrictionCoefficient(2.5);
    //fFloorRB->setRestitutionCoefficient(0.35);
    
    fLowController = createController( fCharacter );
    fLowController->setStance( LEFT_STANCE );
    
    WorldOracle* worldOracle = new WorldOracle();
    
    fHighController = new TurnController(fCharacter, fLowController, worldOracle);
    fHighController->initializeDefaultParameters();
    
    fHighController->requestHeading(0.);
    fHighController->conTransitionPlan();
    fHighController->requestCoronalStepWidth(.3);
    fHighController->requestVelocities(.5, 0.);
    
    fContactGroup = dJointGroupCreate(0);
}

Cartwheel::~Cartwheel()
{
    dJointGroupDestroy(fContactGroup);
    dWorldDestroy(fWorld);
}

void Cartwheel::Collide(dGeomID g1, dGeomID g2, RigidBody* rb1, RigidBody* rb2)
{
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactBounce | dContactApprox1;
        contact[i].surface.bounce = 0.0;
        contact[i].surface.mu = 1000.0;
        dJointID c = dJointCreateContact(fWorld, fContactGroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
        
        if (fJointFeedbackCount >= MAX_CONTACT_FEEDBACK) {
            printf("Warning: too many contacts are established. Some of them will not be reported.\n");
        } else {
            fContactPoints.push_back(ContactPoint());
            //now we'll set up the feedback for this contact joint
            fContactPoints[fJointFeedbackCount].rb1 = rb1;
            fContactPoints[fJointFeedbackCount].rb2 = rb2;
            fContactPoints[fJointFeedbackCount].cp = Point3d(contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2]);
            dJointSetFeedback(c,&(fJointFeedback[fJointFeedbackCount]));
            fJointFeedbackCount++;
        }
    }
}

void Cartwheel::BodyAddTorque(dBodyID body, Vector3d torque)
{
    dBodyAddTorque(body, torque.x, torque.y, torque.z);
}

void Cartwheel::AdvanceInTime(double dt, const JointTorques& torques)
{
    //restart the counter for the joint feedback terms
    fJointFeedbackCount = 0;
    
    //go through all the joints in the world, and apply their torques to the parent and child rb's
    /* for(int jid=0; jid<J_MAX; jid++) {
        KTJoint* joint = fCharacter->getJoints()[jid];
        Vector3d t = joint->torque;
        //we will apply to the parent a positive torque, and to the child a negative torque
        dBodyAddTorque(fRobot->getODEBody(joint->parent), t.x, t.y, t.z);
        dBodyAddTorque(fRobot->getODEBody(joint->child), -t.x, -t.y, -t.z);
    } */
    
    BodyAddTorque(fRobot->fPelvisB, torques.get(J_L_HIP));
    BodyAddTorque(fRobot->fLUpperLegB, -torques.get(J_L_HIP));
    
    BodyAddTorque(fRobot->fLUpperLegB, torques.get(J_L_KNEE));
    BodyAddTorque(fRobot->fLLowerLegB, -torques.get(J_L_KNEE));
    
    BodyAddTorque(fRobot->fLLowerLegB, torques.get(J_L_ANKLE));
    BodyAddTorque(fRobot->fLFootB, -torques.get(J_L_ANKLE));
    
    BodyAddTorque(fRobot->fPelvisB, torques.get(J_R_HIP));
    BodyAddTorque(fRobot->fRUpperLegB, -torques.get(J_R_HIP));
    
    BodyAddTorque(fRobot->fRUpperLegB, torques.get(J_R_KNEE));
    BodyAddTorque(fRobot->fRLowerLegB, -torques.get(J_R_KNEE));
    
    BodyAddTorque(fRobot->fRLowerLegB, torques.get(J_R_ANKLE));
    BodyAddTorque(fRobot->fRFootB, -torques.get(J_R_ANKLE));
    
    
    //clear the previous list of contact forces
    fContactPoints.clear();
    
    //we need to determine the contact points first - delete the previous contacts
    dJointGroupEmpty(fContactGroup);
    //initiate the collision detection
    #ifndef DEBUG_FIXED_TORSO
    Collide(fFloorG, fRobot->fLFootG, fFloorRB, fRobot->fLFootRB);
    Collide(fFloorG, fRobot->fRFootG, fFloorRB, fRobot->fRFootRB);
    #endif
    
    //advance the simulation
    dWorldStep(fWorld, dt);
    
    //copy over the state of the ODE bodies to the rigid bodies...
    for(int rid=0; rid<R_MAX; rid++)
    {
        ArticulatedRigidBody* rb = fCharacter->getARBs()[rid];
        //setRBStateFromODE((*rb)->ODEBodyID, *rb);
        setRBStateFromODE(rb);
    }
    
    //copy over the force information for the contact forces
    for (int i=0;i<fJointFeedbackCount;i++){
        fContactPoints[i].f = Vector3d(fJointFeedback[i].f1[0], fJointFeedback[i].f1[1], fJointFeedback[i].f1[2]);
        //make sure that the force always points away from the static objects
        //if (fContactPoints[i].rb1->isLocked() && !fContactPoints[i].rb2->isLocked()){
            fContactPoints[i].f = fContactPoints[i].f * (-1);
            RigidBody* tmpBdy = fContactPoints[i].rb1;
            fContactPoints[i].rb1 = fContactPoints[i].rb2;
            fContactPoints[i].rb2 = tmpBdy;
        //}
    }
}

void Cartwheel::setRBStateFromODE(RigidBody* rb)
{
    dBodyID body = fRobot->getODEBody(rb);
    const dReal *tempData;
    
    tempData = dBodyGetPosition(body);
    rb->state.position.x = tempData[0];
    rb->state.position.y = tempData[1];
    rb->state.position.z = tempData[2];
    
    tempData = dBodyGetQuaternion(body);
    rb->state.orientation.s = tempData[0];
    rb->state.orientation.v.x = tempData[1];
    rb->state.orientation.v.y = tempData[2];
    rb->state.orientation.v.z = tempData[3];
    
    tempData = dBodyGetLinearVel(body);
    rb->state.velocity.x = tempData[0];
    rb->state.velocity.y = tempData[1];
    rb->state.velocity.z = tempData[2];
    
    tempData = dBodyGetAngularVel(body);
    rb->state.angularVelocity.x = tempData[0];
    rb->state.angularVelocity.y = tempData[1];
    rb->state.angularVelocity.z = tempData[2];
}

void Cartwheel::Advance()
{
    const double dt = 1./(STEP_PER_SEC * INT_PER_STEP);
    
    for(int i=0; i<INT_PER_STEP; ++i) {
        //dBodyAddForce(fRobot->fPelvisB, 40.*sin(fT), 40.*cos(fT), 0.);
        
        JointTorques torques = fHighController->performPreTasks(dt, &fContactPoints);
        //fLowController->performPreTasks(dt, &fContactPoints);
        AdvanceInTime(dt, torques);
        fT += dt;
        //fLowController->performPostTasks(dt, &fContactPoints);
        fHighController->performPostTasks(dt, &fContactPoints);
        
        fHighController->requestHeading(fT/4.);
        
        fDebugJTorques = torques;
    }
}

CartState Cartwheel::GetCurrentState()
{
    double tmp[4];
    CartState state;
    
    state.fParent = this;
    //state.fTorsoQ = BodyQ::FromODE(fRobot->fTorsoB);
    //state.fLowerBackQ = BodyQ::FromODE(fRobot->fLowerBackB);
    state.fPelvisQ = BodyQ::FromODE(fRobot->fPelvisB);
    
    state.fLUpperLegQ = BodyQ::FromODE(fRobot->fLUpperLegB);
    state.fLLowerLegQ = BodyQ::FromODE(fRobot->fLLowerLegB);
    state.fRUpperLegQ = BodyQ::FromODE(fRobot->fRUpperLegB);
    state.fRLowerLegQ = BodyQ::FromODE(fRobot->fRLowerLegB);
    
    state.fLFootQ = BodyQ::FromODE(fRobot->fLFootB);
    state.fRFootQ = BodyQ::FromODE(fRobot->fRFootB);
    
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
    
    for(int jid=0; jid<J_MAX; jid++) {
        Vector3d t = fDebugJTorques.get(jid);
        state.fJTorques[jid] = Eigen::Vector3d(t.x, t.y, t.z);
    }
    
    Vector3d com = fRobot->fCharacter->getCOM();
    state.fCoM = Eigen::Vector3d(com.x, com.y, com.z);
    
    return state;
}
