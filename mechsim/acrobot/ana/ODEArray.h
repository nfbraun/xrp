#ifndef _ODEARRAY_H_
#define _ODEARRAY_H_

template<typename T>
class ODEArray
{
  public:
    ODEArray(size_t _n) : n(_n), data(new T[n]) {}
    ODEArray(size_t _n, const T* init)
        : n(_n), data(new T[n])
        { for(size_t i=0; i<n; i++) data[i] = init[i]; }
    ODEArray(const ODEArray<T>& ar)
        : n(ar.n), data(new T[n])
        { for(size_t i=0; i<n; i++) data[i] = ar.data[i]; }
    ODEArray<T>& operator=(const ODEArray<T>& ar)
        {
            // This function should handle self-assignment implicitly
            if(ar.n != n) {
                delete[] data;
                data = new T[ar.n];
                n = ar.n;
            }
            for(size_t i=0; i<n; i++)
                data[i] = ar.data[i];
            return *this;
        }
    ~ODEArray() { delete[] data; }
    
    operator const T*() const { return data; }
    operator T*() { return data; }
    
  private:
    size_t n;
    T* data;
};

#endif
