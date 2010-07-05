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
const int Hobbelen::STEP_PER_SEC = 16;
const int Hobbelen::INT_PER_STEP = 16;   // Integration intervals per timestep
const char Hobbelen::TITLE[] = "2D Walker";

// ** System parameters **
namespace HobbelenConst {
const double GAMMA      = 0.0;   // Floor slope
const double FLOOR_DIST = 1.0;   // shortest distance floor to origin

//                          m,   I,      l,   c,         w   [SI units]
const ChainSegment Body    (8.5, 0.11,   0.4, 0.4 - 0.2, 0.);
const ChainSegment UpperLeg(0.9, 0.0068, 0.3, 0.15,      0.);
const ChainSegment LowerLeg(0.9, 0.0068, 0.3, 0.15,      0.);

//                     m,   I,      l,     w,      r,    h  [SI units]
const FootSegment Foot(0.1, 0.0001, 0.085, 0.0175, 0.02, 0.025);

// These do not matter much, as long as they are large enough
const double INNER_LEG_DIST = 0.2;
const double OUTER_LEG_DIST = 0.4;
}; // end namespace HobbelenConst

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
                 Rotation::FromQuatArray(dBodyGetQuaternion(id)));
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
    dMass mass;
    
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fFloorG = dCreatePlane(0, sin(HobbelenConst::GAMMA), 0., cos(HobbelenConst::GAMMA),
                           -HobbelenConst::FLOOR_DIST);
    
    ChainSegment BodyC(HobbelenConst::Body);
    ChainSegment OULegC(HobbelenConst::UpperLeg);
    ChainSegment OLLegC(HobbelenConst::LowerLeg);
    FootSegment  OFootC(HobbelenConst::Foot);
    ChainSegment IULegC(HobbelenConst::UpperLeg);
    ChainSegment ILLegC(HobbelenConst::LowerLeg);
    FootSegment  IFootC(HobbelenConst::Foot);
    
    OFootC.SetPF(Vector3(0., 0., -HobbelenConst::FLOOR_DIST + OFootC.r()), 0.);
    OLLegC.SetP2(OFootC.p1(), -.2);
    OULegC.SetP2(OLLegC.p1(), -.2);
    BodyC.SetP2(OULegC.p1(), 0.);
    IULegC.SetP1(OULegC.p1(), .2);
    ILLegC.SetP1(IULegC.p2(), .5);
    IFootC.SetP1(ILLegC.p2(), .3);
    
    InitLeg(fIULegB, fILLegB, fIFootB, fIFFootG, fIBFootG, IULegC, ILLegC, IFootC);
    InitLeg(fOULegB, fOLLegB, fOFootB, fOFFootG, fOBFootG, OULegC, OLLegC, OFootC);
    
    fBodyB = BodyFromConfig(BodyC);
    dJointID j1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j1, fIULegB, fBodyB);
    ODE::JointSetHingeAnchor(j1, IULegC.p1());
    ODE::JointSetHingeAxis(j1, Vector3::eY);
    
    dJointID j2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j2, fOULegB, fBodyB);
    ODE::JointSetHingeAnchor(j2, OULegC.p1());
    ODE::JointSetHingeAxis(j2, Vector3::eY);
    
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

void Hobbelen::InitLeg(dBodyID& upperLegB, dBodyID& lowerLegB,
             dBodyID& footB, dGeomID& fFootG, dGeomID& bFootG,
             ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC)
{
    upperLegB = BodyFromConfig(upperLegC);
    
    lowerLegB = BodyFromConfig(lowerLegC);
    
    dJointID j1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j1, upperLegB, lowerLegB);
    ODE::JointSetHingeAnchor(j1, upperLegC.p2());
    ODE::JointSetHingeAxis(j1, Vector3::eY);
    
    footB = BodyFromConfig(footC);
    
    fFootG = dCreateCapsule(0, footC.r(), .1);
    dGeomSetBody(fFootG, footB);
    ODE::GeomSetOffsetPosition(fFootG, footC.pfb());
    dGeomSetOffsetQuaternion(fFootG, Rotation(M_PI/2., Vector3::eX).quatarray());
    
    bFootG = dCreateCapsule(0, footC.r(), .1);
    dGeomSetBody(bFootG, footB);
    ODE::GeomSetOffsetPosition(bFootG, footC.pbb());
    dGeomSetOffsetQuaternion(bFootG, Rotation(M_PI/2., Vector3::eX).quatarray());
    
    dJointID j2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j2, lowerLegB, footB);
    ODE::JointSetHingeAnchor(j2, lowerLegC.p2());
    ODE::JointSetHingeAxis(j2, Vector3::eY);
}

Hobbelen::~Hobbelen()
{
    dWorldDestroy(fWorld);
    dJointGroupDestroy(fContactGroup);
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
        Collide(fFloorG, fIFFootG);
        Collide(fFloorG, fIBFootG);
        Collide(fFloorG, fOFFootG);
        Collide(fFloorG, fOBFootG);
        
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        dJointGroupEmpty(fContactGroup);
    }
}

HobState Hobbelen::GetCurrentState()
{
    HobState state;
    
    state.fParent = this;
    
    state.fBodyQ = BodyQ::FromODE(fBodyB);
    
    state.fIULegQ = BodyQ::FromODE(fIULegB);
    state.fILLegQ = BodyQ::FromODE(fILLegB);
    state.fIFootQ = BodyQ::FromODE(fIFootB);
    
    state.fOULegQ = BodyQ::FromODE(fOULegB);
    state.fOLLegQ = BodyQ::FromODE(fOLLegB);
    state.fOFootQ = BodyQ::FromODE(fOFootB);
    
    return state;
}
