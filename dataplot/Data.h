#ifndef VC_DATA_H
#define VC_DATA_H

#include <map>
#include <iostream>

typedef std::map<double, double> data_t;

class MutableData;
class _Data;

class DataStorage
{
  friend class MutableData;
  friend class _Data;
  public:
    DataStorage() : fRefcount(0) { }
    ~DataStorage() { }
  private:
    // Copying is not supported
    DataStorage(const DataStorage&) { }
    DataStorage& operator=(const DataStorage&) { return *this; }
    
    data_t fRawData;
    int fRefcount;
};

class MutableData
{
  friend class _Data;
  public:
    MutableData()  { fStorage = new DataStorage; fStorage->fRefcount++; }
    ~MutableData() { delete fStorage; }
    
    data_t& data()             { return fStorage->fRawData; }
    const data_t& data() const { return fStorage->fRawData; }
  private:
    // Copying is not supported
    MutableData(const MutableData&) { }
    MutableData& operator=(const MutableData&) { return *this; }
    
    DataStorage* fStorage;
};

class _Data
{
  public:
    _Data();
    _Data(const _Data& src);
    _Data& operator=(const _Data& src);
    ~_Data();
    
    void adopt(MutableData& src);
    
    const data_t& data() const { return fStorage->fRawData; }
    
  private:
    DataStorage *fStorage;
};

class Data: public _Data
{
  public:
    typedef data_t::const_iterator const_iterator_t;
    
    Data() : _Data() {}
    
    double minX() const;
    double maxX() const;
    
    double getMinY() const;
    double getMaxY() const;
    
    const_iterator_t getFirst(double x) const;
    const_iterator_t getLast(double x) const;
};

#endif
