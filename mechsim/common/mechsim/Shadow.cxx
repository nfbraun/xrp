/* Simple stencil shadows 
   Taken from: http://nehe.gamedev.net/tutorial/shadows/16010/  */

#include "Shadow.h"

Object::Object()
{
    // Set sane defaults for parameters
    fC = Eigen::Vector3f(0., 0., 0.);
    fQ = Eigen::Quaternionf(1., 0., 0., 0.);
    
    fColor = Eigen::Vector4f(1., 1., 1., 1.);
}

// connectivity procedure - based on Gamasutra's article
// hard to explain here
void Object::updateConnectivity()
{
    long p1i, p2i, p1j, p2j;
    long P1i, P2i, P1j, P2j;
    long i,j,ki,kj;
    
    for ( i=0 ; i<fPlanes.size()-1 ; i++ ) {
        for ( j=i+1 ; j<fPlanes.size() ; j++ ) {
            for ( ki=0 ; ki<3 ; ki++ ) {
                if ( !fPlanes[i].neigh[ki] ) {
                    for ( kj=0 ; kj<3 ; kj++ ) {
                        p1i=ki;
                        p1j=kj;
                        p2i=(ki+1)%3;
                        p2j=(kj+1)%3;
                        
                        p1i=fPlanes[i].p[p1i];
                        p2i=fPlanes[i].p[p2i];
                        p1j=fPlanes[j].p[p1j];
                        p2j=fPlanes[j].p[p2j];
                        
                        P1i=((p1i+p2i)-abs(p1i-p2i))/2;
                        P2i=((p1i+p2i)+abs(p1i-p2i))/2;
                        P1j=((p1j+p2j)-abs(p1j-p2j))/2;
                        P2j=((p1j+p2j)+abs(p1j-p2j))/2;
                        
                        if ( (P1i==P1j) && (P2i==P2j) ) {  //they are neighbours
                            fPlanes[i].neigh[ki] = j+1;
                            fPlanes[j].neigh[kj] = i+1;
                        }
                    }
                }
            }
        }
    }
}

void Object::updatePlaneEq()
{
    for (unsigned int i=0 ; i < fPlanes.size() ; i++ ) {
        Eigen::Vector3f v0, v1, v2;
        
        v0 = fPoints[fPlanes[i].p[0]];
        v1 = fPoints[fPlanes[i].p[1]];
        v2 = fPoints[fPlanes[i].p[2]];
        
        Eigen::Vector3f n = (v0 - v1).cross(v0 - v2);
        
        fPlanes[i].fPlaneEq.n = n;
        fPlanes[i].fPlaneEq.d = -v0.dot(n);
    }
}

void Object::setCenter(const Eigen::Vector3f& c)
{
    fC = c;
    updatePoints();
    updatePlaneEq();
}

void Object::setRotation(const Eigen::Quaternionf& q)
{
    fQ = q;
    updatePoints();
    updateNormals();
    updatePlaneEq();
}

Box::Box(float lx, float ly, float lz)
    : Object()
{
    fPoints.resize(8);
    
    /* Set topology */
    fPlanes.resize(12);
    fPlanes[0] = sPlane(0,2,1);
    fPlanes[1] = sPlane(0,3,2);
    fPlanes[2] = sPlane(4,5,6);
    fPlanes[3] = sPlane(4,6,7);
    fPlanes[4] = sPlane(4,3,0);
    fPlanes[5] = sPlane(4,7,3);
    fPlanes[6] = sPlane(2,5,1);
    fPlanes[7] = sPlane(2,6,5);
    fPlanes[8] = sPlane(4,0,1);
    fPlanes[9] = sPlane(4,1,5);
    fPlanes[10] = sPlane(2,3,7);
    fPlanes[11] = sPlane(2,7,6);
    
    updateConnectivity();
    
    fsx = lx/2.;
    fsy = ly/2.;
    fsz = lz/2.;
    
    updatePoints();
    updateNormals();
    
    updatePlaneEq();
}

void Box::updatePoints()
{
    fPoints[0] = fQ*Eigen::Vector3f(-fsx,  fsy, -fsz) + fC;
    fPoints[1] = fQ*Eigen::Vector3f( fsx,  fsy, -fsz) + fC;
    fPoints[2] = fQ*Eigen::Vector3f( fsx,  fsy,  fsz) + fC;
    fPoints[3] = fQ*Eigen::Vector3f(-fsx,  fsy,  fsz) + fC;
    fPoints[4] = fQ*Eigen::Vector3f(-fsx, -fsy, -fsz) + fC;
    fPoints[5] = fQ*Eigen::Vector3f( fsx, -fsy, -fsz) + fC;
    fPoints[6] = fQ*Eigen::Vector3f( fsx, -fsy,  fsz) + fC;
    fPoints[7] = fQ*Eigen::Vector3f(-fsx, -fsy,  fsz) + fC;
}

void Box::updateNormals()
{
    fPlanes[0].setNormals(fQ * Eigen::Vector3f(0,1,0));
    fPlanes[1].setNormals(fQ * Eigen::Vector3f(0,1,0));
    fPlanes[2].setNormals(fQ * Eigen::Vector3f(0,-1,0));
    fPlanes[3].setNormals(fQ * Eigen::Vector3f(0,-1,0));
    fPlanes[4].setNormals(fQ * Eigen::Vector3f(-1,0,0));
    fPlanes[5].setNormals(fQ * Eigen::Vector3f(-1,0,0));
    fPlanes[6].setNormals(fQ * Eigen::Vector3f(1,0,0));
    fPlanes[7].setNormals(fQ * Eigen::Vector3f(1,0,0));
    fPlanes[8].setNormals(fQ * Eigen::Vector3f(0,0,-1));
    fPlanes[9].setNormals(fQ * Eigen::Vector3f(0,0,-1));
    fPlanes[10].setNormals(fQ * Eigen::Vector3f(0,0,1));
    fPlanes[11].setNormals(fQ * Eigen::Vector3f(0,0,1));
}

Cylinder::Cylinder(float r, float l, unsigned int nseg)
    : Object()
{
    fNSeg = nseg;
    
    fPoints.resize(2*fNSeg+2);
    
    /* Set topology */
    fPlanes.resize(4*fNSeg);
    /* Side planes */
    for(unsigned int i=0; i<(fNSeg-1); i++) {
        fPlanes[i] = sPlane(i,i+fNSeg,i+1);
        fPlanes[i+fNSeg] = sPlane(i+fNSeg, i+fNSeg+1, i+1);
    }
    fPlanes[fNSeg-1] = sPlane(fNSeg-1, fNSeg-1+fNSeg, 0);
    fPlanes[fNSeg-1+fNSeg] = sPlane(fNSeg-1+fNSeg, fNSeg, 0);
    
    /* Top planes */
    for(unsigned int i=0; i<(fNSeg-1); i++) {
        fPlanes[i+2*fNSeg] = sPlane(i,i+1,2*fNSeg);
    }
    fPlanes[fNSeg-1+2*fNSeg] = sPlane(fNSeg-1, 0, 2*fNSeg);
    
    /* Bottom planes */
    for(unsigned int i=0; i<(fNSeg-1); i++) {
        fPlanes[i+3*fNSeg] = sPlane(i+fNSeg,2*fNSeg+1,i+fNSeg+1);
    }
    fPlanes[fNSeg-1+3*fNSeg] = sPlane(fNSeg-1+fNSeg, 2*fNSeg+1, fNSeg);
    
    updateConnectivity();
    
    fr = r;
    fs = l/2.;
    
    updatePoints();
    updateNormals();
    
    updatePlaneEq();
}

void Cylinder::updatePoints()
{
    
    for(unsigned int i=0; i<fNSeg; i++) {
        float phi = (((float)i)/fNSeg) * 2.*M_PI;
        fPoints[i] = fQ*Eigen::Vector3f(fr*cos(phi), fr*sin(phi), fs) + fC;
        fPoints[i+fNSeg] = fQ*Eigen::Vector3f(fr*cos(phi), fr*sin(phi), -fs) + fC;
    }
    fPoints[2*fNSeg] = fQ*Eigen::Vector3f(0., 0., fs) + fC;
    fPoints[2*fNSeg+1] = fQ*Eigen::Vector3f(0., 0., -fs) + fC;
}

void Cylinder::updateNormals()
{
    for(unsigned int i=0; i<fNSeg; i++) {
        float phi0 = (((float)i)/fNSeg) * 2.*M_PI;
        float phi1 = (((float)i+1.)/fNSeg) * 2.*M_PI;
        Eigen::Vector3f n0 = fQ*Eigen::Vector3f(cos(phi0), sin(phi0), 0.);
        Eigen::Vector3f n1 = fQ*Eigen::Vector3f(cos(phi1), sin(phi1), 0.);
        fPlanes[i].setNormals(n0, n0, n1);
        fPlanes[i+fNSeg].setNormals(n0, n1, n1);
    }
    for(unsigned int i=0; i<fNSeg; i++) {
        fPlanes[i+2*fNSeg].setNormals(fQ*Eigen::Vector3f(0., 0., 1.));
        fPlanes[i+3*fNSeg].setNormals(fQ*Eigen::Vector3f(0., 0., -1.));
    }
}

void Scene::castShadowPass(const std::vector< std::vector<bool> >& visible)
{
    unsigned int i, j, k, jj;
    unsigned int p1, p2;
    Eigen::Vector3f v1, v2;
    
    std::vector< std::vector<bool> >::const_iterator vis_it = visible.begin();
    
    for (std::vector<const Object*>::const_iterator oit = fObj.begin(); oit != fObj.end(); ++oit) {
        const Object* o = *oit;
        const std::vector<bool>& vis = *vis_it;
        vis_it++;
        
        for ( i=0 ; i < o->fPlanes.size() ; i++ ) {
            if ( vis[i] ) {
                for ( j=0 ; j < 3 ; j++ ) {
                    k = o->fPlanes[i].neigh[j];
                    if ((!k) || (!vis[k-1])){
                        // here we have an edge, we must draw a polygon
                        p1 = o->fPlanes[i].p[j];
                        jj = (j+1)%3;
                        p2 = o->fPlanes[i].p[jj];
                        
                        //calculate the length of the vector
                        v1 = (o->fPoints[p1] - lightPos())*100;
                        v2 = (o->fPoints[p2] - lightPos())*100;
                        
                        //draw the polygon
                        glBegin(GL_TRIANGLE_STRIP);
                            glVertex3f(o->fPoints[p1].x(),
                                        o->fPoints[p1].y(),
                                        o->fPoints[p1].z());
                            glVertex3f(o->fPoints[p1].x() + v1.x(),
                                        o->fPoints[p1].y() + v1.y(),
                                        o->fPoints[p1].z() + v1.z());
                            
                            glVertex3f(o->fPoints[p2].x(),
                                        o->fPoints[p2].y(),
                                        o->fPoints[p2].z());
                            glVertex3f(o->fPoints[p2].x() + v2.x(),
                                        o->fPoints[p2].y() + v2.y(),
                                        o->fPoints[p2].z() + v2.z());
                        glEnd();
                    }
                }
            }
        }
    }
}

void Scene::addObject(Object* obj)
{
    fObj.push_back(obj);
}

void Scene::castShadow()
{
    std::vector< std::vector<bool> > visible(fObj.size());
    
    //set visual parameter
    for(unsigned int obj_id=0; obj_id<fObj.size(); obj_id++) {
        visible[obj_id].resize(fObj[obj_id]->fPlanes.size());
        for (unsigned int i=0; i<fObj[obj_id]->fPlanes.size(); i++){
            // check to see if light is in front or behind the plane (face plane)
            float side = fObj[obj_id]->fPlanes[i].fPlaneEq.n.dot(lightPos()) + fObj[obj_id]->fPlanes[i].fPlaneEq.d;
            visible[obj_id][i] = side > 0;
        }
    }
    
    glDisable(GL_LIGHTING);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
    
    glEnable(GL_STENCIL_TEST);
    glColorMask(0, 0, 0, 0);
    glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
    
    // first pass, stencil operation increases stencil value
    glFrontFace(GL_CCW);
    glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
    castShadowPass(visible);
    
    // second pass, stencil operation decreases stencil value
    glFrontFace(GL_CW);
    glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
    castShadowPass(visible);

    glFrontFace( GL_CCW );
    glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );

    //draw a shadowing rectangle covering the entire screen
    glColor4f( 0.0f, 0.0f, 0.0f, 0.4f );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glStencilFunc( GL_NOTEQUAL, 0, 0xffffffff );
    glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
    glPushMatrix();
    glLoadIdentity();
    glBegin( GL_TRIANGLE_STRIP );
        glVertex3f( -2.1f,  1.1f, -2.5f );
        glVertex3f( -2.1f, -1.1f, -2.5f );
        glVertex3f(  2.1f,  1.1f, -2.5f );
        glVertex3f(  2.1f, -1.1f, -2.5f );
    glEnd();
    glPopMatrix();
    glDisable(GL_BLEND);
    
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_LIGHTING);
    glDisable(GL_STENCIL_TEST);
    glShadeModel(GL_SMOOTH);
}

void Scene::drawObjects()
{
    unsigned int i, j;
    
    glBegin(GL_TRIANGLES);
    for (std::vector<const Object*>::const_iterator oit = fObj.begin(); oit != fObj.end(); ++oit) {
        const Object* o = *oit;
        glColor4fv(o->color().data());
        for (i=0; i<o->fPlanes.size(); i++){
            for (j=0; j<3; j++){
                glNormal3f(o->fPlanes[i].normal(j).x(),
                           o->fPlanes[i].normal(j).y(),
                           o->fPlanes[i].normal(j).z());
                glVertex3f(o->fPoints[o->fPlanes[i].p[j]].x(),
                           o->fPoints[o->fPlanes[i].p[j]].y(),
                           o->fPoints[o->fPlanes[i].p[j]].z());
            }
        }
    }
    glEnd();
    
    /* Show normals (debug only) */
    #if 0
    glColor3f(1., 0., 1.);
    glBegin(GL_LINES);
    for (std::vector<const Object*>::const_iterator oit = fObj.begin(); oit != fObj.end(); ++oit) {
        const Object* o = *oit;
        for (i=0; i<o->planes.size(); i++){
            for (j=0; j<3; j++){
                Eigen::Vector3f p1 = o->points[o->planes[i].p[j]];
                Eigen::Vector3f p2 = p1 + 0.1*o->planes[i].normals[j];
                
                glVertex3f(p1.x(), p1.y(), p1.z());
                glVertex3f(p2.x(), p2.y(), p2.z());
            }
        }
    }
    glEnd();
    #endif
}

