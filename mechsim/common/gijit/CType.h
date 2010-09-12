#ifndef _CTYPE_H_
#define _CTYPE_H_

namespace GiJIT {

class CType_Any {
  virtual void dummy() {}
};

template<typename T>
class CType: public CType_Any {
  virtual void dummy() {}
};

} // end namespace GiJIT

#endif
