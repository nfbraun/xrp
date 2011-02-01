#include "CSVFile.h"

#include <fstream>
#include <sstream>

CSVFile::CSVFile(const char* fname, bool requireAscendingIdx, unsigned int nval)
    : fNVal(nval)
{
    // Remember filename for better error reporting
    fFName.assign(fname);
    
    std::ostringstream err;
    
    // Attempt to open the file
    std::ifstream file(fname, std::ios::in);
    
    // Start optimistically
    fFail = false;
    
    // Check if file could be opened
    if(!file) {
        err << "Error: failed to open file " << fname << "\n";
        fFail = true;
        return;
    }
    
    // Read all lines
    std::string line;
    value_t lastT = -std::numeric_limits<value_t>::infinity();
    int nline = 1;
    while(getline(file, line) && !fFail) {
        if(line.compare(0, 2, "#:") == 0)
            ParseValueName(line, nline++, err);
        else
            ParseLine(line, lastT, requireAscendingIdx, nline++, err);
    }
    
    if(!fFail && fData.empty()) {
        err << fFName << ": No data found\n";
        fFail = true;
    }
    
    fErrorStr = err.str();
}

void CSVFile::ParseValueName(const std::string& line, const int nline,
    std::ostringstream& err)
{
    std::istringstream input(line);
    int i;
    std::string name;
    if(!(input.get() == '#' && input.get() == ':')) {
        err << fFName << ":" << nline << ": Warning: parse error\n";
        return;
    }
    
    input >> i;
    if(input.fail() || input.get() != ':') {
        err << fFName << ":" << nline << ": Warning: parse error\n";
        return;
    }
    
    getline(input, name);
    if(name.empty() || (input.fail() && !input.eof())) {
        err << fFName << ":" << nline << ": Warning: parse error\n";
        return;
    }
    
    fValNames[i] = name;
}

void CSVFile::ParseLine(const std::string& line, value_t& lastT,
    bool requireAscendingIdx, const int nline, std::ostringstream& err)
{
    // Create input string, ignoring everything after a potential comment
    std::istringstream input(line.substr(0, line.find_first_of('#')));
    
    // Get values
    value_t x;
    values_t values;
    unsigned int nv = 0;
    while((input >> x) && nv <= fNVal) {
        values.push_back(x);
        nv++;
    }
    
    // If a parsing error occured, we return an error
    if(input.fail() && !input.eof()) {
        err << fFName << ":" << nline << ": Parse error\n";
        fFail = true;
        return;
    }
    
    // Empty lines are silently ignored
    if(nv == 0)
        return;
    
    // Test for strictly ascending index (column 0)
    if(requireAscendingIdx) {
        if(values[0] <= lastT) {
            err << fFName << ":" << nline << ": "
                << "Index values must be strictly ascending\n";
            fFail = true;
            return;
        }
        lastT = values[0];
    }
    
    if(fNVal == std::numeric_limits<unsigned int>::max())
        fNVal = nv;
    
    if(nv != fNVal) {
        err << fFName << ":" << nline << ": Unexpected number of values\n";
        fFail = true;
        return;
    }
    
    // Add values to data store
    fData.push_back(values);
    
    return;
}
