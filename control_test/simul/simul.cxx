#include <iostream>
#include <limits>
#include <vector>

class Ringbuf {
  public:
    Ringbuf(int size, double ival = std::numeric_limits<double>::quiet_NaN())
         : fBuf(size, ival), fIndex(size-1) { }
    void push(double x);
    double get(int t);
    
  private:
    std::vector<double> fBuf;
    int fIndex;
};

void Ringbuf::push(double x)
{
    ++fIndex;
    fIndex %= fBuf.size();
    
    fBuf[fIndex] = x;
}

double Ringbuf::get(int i)
{
    if(i > 0 || i <= -fBuf.size())
        return std::numeric_limits<double>::quiet_NaN();
    
    // Avoid negative operands for modulo operator
    // (implementation-defined behavior for negative operands)
    int pos = (fBuf.size() + fIndex + i) % fBuf.size();
    return fBuf[pos];
}

double controller(double x, double v)
{
    return -2. * x - 2. * v;
}

int main()
{
    Ringbuf xBuf(1000, 1.0), vBuf(1000, 0.0);

    const double TIMESTEP = 0.001;
    const int NSTEPS = 100.0 / TIMESTEP;
    
    const double k = 1.0;
    
    double t;
    double x = 1.0;
    double v = 0.0;
    double a;
    
    for(int n=0; n<=NSTEPS; ++n) {
        a = k * x + controller(xBuf.get(-100), vBuf.get(-100));
        
        t = n * TIMESTEP;
        x += v * TIMESTEP;
        v += a * TIMESTEP;
        xBuf.push(x);
        vBuf.push(v);
        
        std::cout << t << " " << x << " " << v << std::endl;
    }
    
    return 0;
}
