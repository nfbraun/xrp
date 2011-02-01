#ifndef VC_CSVFILE_H
#define VC_CSVFILE_H

#include <string>
#include <vector>
#include <map>
#include <limits>

class CSVFile {
  public:
    CSVFile()
     : fFail(true),
       fData(),
       fFName()
     { }
    
    typedef double value_t;
    typedef std::vector<value_t> values_t;
    typedef std::vector<values_t> data_t;
    
    CSVFile(const char* fname,
        bool requireAscendingIdx = false,
        unsigned int nval = std::numeric_limits<unsigned int>::max());
    inline bool fail() const { return fFail; }
    inline unsigned int nval() const { return fNVal; }
    inline const data_t& data() const { return fData; }
    inline const std::string& valName(int i) { return fValNames[i]; }
    inline const std::string& errorstr() const { return fErrorStr; }
    
  private:
    unsigned int fNVal;
    bool fFail;
    void ParseLine(const std::string& line, value_t& lastT,
        bool requireAscensingIdx, const int nline, std::ostringstream& err);
    void ParseValueName(const std::string& line, const int nline,
        std::ostringstream& err);
    data_t fData;
    std::map<int, std::string> fValNames;
    std::string fFName;
    std::string fErrorStr;
};

#endif
