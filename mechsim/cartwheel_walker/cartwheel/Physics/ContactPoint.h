#pragma once

#include <MathLib/Point3d.h>
#include <MathLib/Vector3d.h>
#include <vector>

/**
    This class is mainly a container for a Contact Point. It holds information such as the world coordinates of the contact point, the normal at the contact, the rigid bodies that generated it, etc.
*/
class ContactPoint {
  public:
    // world coordinate of the origin of the contact force
    Point3d cp;
    
    // force applied (with f being applied to rb1, and -f to rb2)
    Vector3d f;
};

class ContactData {
  public:
    // static const unsigned int MAX_CONTACTS = 4;  // FIXME
    
    std::vector<ContactPoint> pLeft;
    std::vector<ContactPoint> pRight;
};
