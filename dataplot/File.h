#ifndef VC_FILE_H
#define VC_FILE_H

#include <string>
#include <vector>
#include "Channel.h"

class File
{
  public:
    File(const std::string& fname);
    
    enum reload_result_t { RELOAD_FAIL, RELOAD_LAYOUT_CHANGED, RELOAD_SUCCESS };
    
    reload_result_t reload();
    void deleteAllChannels();
    
    inline bool fail() { return fFail; }
    
    const std::string& fname() const { return fFName; }
    std::string basename() const
        {
          std::string::size_type pos = fFName.find_last_of('/');
          return fFName.substr(pos == std::string::npos ? 0 : pos+1);
        }
    
    typedef std::vector<Channel>::const_iterator ch_iterator_t;
    const std::vector<Channel>& channels() const { return fChannels; }
    const std::string& errorstr() const { return fErrorStr; }
    
  private:
    std::string fFName;
    std::string fErrorStr;
    std::vector<Channel> fChannels;
    bool fFail;
};

#endif
