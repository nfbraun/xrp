#ifndef MSIM_HEIGHTFIELD_H
#define MSIM_HEIGHTFIELD_H

#include <vector>
#include <stdexcept>

class Heightfield {
  public:
    Heightfield(unsigned int xsize, unsigned int ysize, double x1, double x2, double y1, double y2)
        : fXSize(xsize), fYSize(ysize), fX1(x1), fX2(x2), fY1(y1), fY2(y2),
          fXScale((double)xsize/(x2-x1)), fYScale((double)ysize/(y2-y1)),
          fData(xsize*ysize, 0.) {}
    
    unsigned int xsize() const { return fXSize; }
    unsigned int ysize() const { return fYSize; }
    
    double x1() const { return fX1; }
    double x2() const { return fX2; }
    double y1() const { return fY1; }
    double y2() const { return fY2; }
    
    double xcoord(double x) const { return x / fXScale + fX1; }
    double ycoord(double y) const { return y / fYScale + fY1; }
    
    double at(unsigned int x, unsigned int y) const {
        if(x >= xsize() || y >= ysize()) return 0.;
        else return fData[y*xsize()+x];
    }
    
    double& at(unsigned int x, unsigned int y) {
        if(x >= xsize() || y >= ysize()) {
            throw std::runtime_error("Access to region outside heightfield range");
        }
        return fData[y*xsize()+x];
    }
    
    const double* raw_data() const { return fData.data(); }
    
  private:
    unsigned int fXSize, fYSize;
    double fX1, fX2, fY1, fY2;
    double fXScale, fYScale;
    
    std::vector<double> fData;
};

#endif
