#include "File.h"
#include "Data.h"
#include "CSVFile.h"
#include <sstream>

File::File(const std::string& fname)
    : fFName(fname)
{
    reload();
}

File::~File()
{
    deleteAllChannels();
}
void File::deleteAllChannels()
{
    for(std::vector<Channel*>::iterator ch = fChannels.begin();
        ch != fChannels.end(); ch++) {
        delete *ch;
    }
    
    fChannels.clear();
}

File::reload_result_t File::reload()
{
    CSVFile file(fFName.c_str(), true);
    fErrorStr = file.errorstr();
    if(file.fail()) {
        fFail = true;
        return RELOAD_FAIL;
    }
    
    reload_result_t retval = RELOAD_SUCCESS;
    if((fChannels.size()+1) != file.nval()) {
        deleteAllChannels();
        for(unsigned int n=0; n<(file.nval()-1); n++) {
            fChannels.push_back(new Channel());
        }
        retval = RELOAD_LAYOUT_CHANGED;
    }
    
    for(unsigned int n=0; n<(file.nval()-1); n++) {
        MutableData data;
        for(CSVFile::data_t::const_iterator i = file.data().begin();
            i != file.data().end(); i++) {
            data.data()[(*i)[0]] = (*i)[n+1];
        }
        
        Data tdata;
        tdata.adopt(data);
        
        std::string name = file.valName(n+1);
        if(name.empty()) {
            std::ostringstream str;
            str << "<Column " << (n+1) << ">";
            name = str.str();
        }
        
        fChannels[n]->setData(tdata);
        fChannels[n]->setName(name);
    }
    
    fFail = false;
    return retval;
}
