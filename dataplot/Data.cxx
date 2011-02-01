#include "Data.h"
#include <cmath>
#include <limits>
#include <algorithm>

_Data::_Data()
{
    fStorage = new DataStorage;
    fStorage->fRefcount++;
}

_Data::_Data(const _Data& src)
{
    fStorage = src.fStorage;
    fStorage->fRefcount++;
}

_Data& _Data::operator=(const _Data& src)
{
    if(&src == this) return (*this);
    
    fStorage->fRefcount--;
    if(fStorage->fRefcount == 0)
        delete fStorage;
    
    fStorage = src.fStorage;
    fStorage->fRefcount++;
    
    return (*this);
}

_Data::~_Data()
{
    fStorage->fRefcount--;
    if(fStorage->fRefcount==0)
        delete fStorage;
}

void _Data::adopt(MutableData& src)
{
    fStorage = src.fStorage;
    src.fStorage = new DataStorage;
    src.fStorage->fRefcount++;
}

double Data::minX() const
{
    if(data().begin() == data().end()) return 0.;
    else return data().begin()->first;
}

double Data::maxX() const
{
    if(data().rbegin() == data().rend()) return 0.;
    else return data().rbegin()->first;
}

double Data::getMinY() const
{
    double minY = std::numeric_limits<double>::infinity();
    for(Data::const_iterator_t it = data().begin();
        it != data().end(); it++) {
        minY = std::min(minY, it->second);
    }
    return minY;
}

double Data::getMaxY() const
{
    double maxY = -std::numeric_limits<double>::infinity();
    for(Data::const_iterator_t it = data().begin();
        it != data().end(); it++) {
        maxY = std::max(maxY, it->second);
    }
    return maxY;
}

Data::const_iterator_t Data::getFirst(double x) const
{
    Data::const_iterator_t it = data().lower_bound(x);
    if(it != data().begin())
        it--;
    return it;
}

Data::const_iterator_t Data::getLast(double x) const
{
    Data::const_iterator_t it = data().lower_bound(x);
    if(it != data().end())
        it++;
    return it;
}
