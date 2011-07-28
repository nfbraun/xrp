#ifndef MSIM_ASYNCSIMULATION_H
#define MSIM_ASYNCSIMULATION_H

#include "Simulation.h"
#include <vector>
#include <iostream>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#define DEBUG_ASYNCSIMULATION
// Interface for simulations that take a while to complete, and are computed
// asynchronously during playback
template<class State_T>
class AsyncSimulation: public Simulation
{
  public:
    AsyncSimulation() : Simulation(), fFinished(false) {
        f_pipefd[0] = -1;
        f_pipefd[1] = -1;
    }
    
    virtual const State_T* GetState(int t)  // t in units of TIMESTEPs
    {
        if(t < 0 || ((unsigned int) t) >= fData.size()) return 0;
        else return &fData[t];
    }
    
    virtual bool finished() { return fFinished; }
    virtual int GetDescriptor() { return f_pipefd[0]; }
    
    virtual void StartSimulation() {
        if(finished()) return;
        
        if(::pipe(f_pipefd) == -1) {
            fatal("pipe() failed");
            return;
        }
        
        f_chpid = ::fork();
        if(f_chpid < 0) {
            ::close(f_pipefd[0]);
            ::close(f_pipefd[1]);
            fatal("fork() failed");
            return;
        }
        
        if(f_chpid == 0) {
            ::close(f_pipefd[0]); // close read end
            RunSimulation();
            ::close(f_pipefd[1]);
            ::_exit(0);
        }
        
        ::close(f_pipefd[1]); // close write end
        fBufIdx = 0;
        
        #ifdef DEBUG_ASYNCSIMULATION
        std::cout << "DEBUG: Started simulation thread with pid " << f_chpid
            << std::endl;
        #endif
    }
    
    virtual void ReadData() {
        ssize_t res = ::read(f_pipefd[0], &fBuf[fBufIdx], BUF_SIZE-fBufIdx);
        if(res < 0) {
            ::close(f_pipefd[0]);
            fatal("read() failed");
            return;
        } else if(res == 0) {
            // Other end of pipe has been closed
            ::close(f_pipefd[0]);
            int status;
            pid_t chldp = waitpid(f_chpid, &status, 0);
            if(chldp == -1) {
                fatal("wait() failed");
                return;
            }
            if(!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
                fatal("Simulation subprocess exited with an error");
                return;
            }
            #ifdef DEBUG_ASYNCSIMULATION
            std::cout << "DEBUG: Simulation thread completed successfully"
                << std::endl;
            #endif
            fFinished = true;
        } else {
            // got data
            for(unsigned int idx = fBufIdx / sizeof(State_T);
                idx < (fBufIdx + res) / sizeof(State_T); idx++)
            {
                fData.push_back(((State_T*)fBuf)[idx]);
            }
            
            fBufIdx += res;
            if(fBufIdx % sizeof(State_T) == 0)
                fBufIdx = 0;
        }
    }
    
  protected:
    virtual void Advance() = 0;
    virtual State_T GetCurrentState() = 0;
    
  private:
    void RunSimulation() {
        int tend = GetDefaultEndTime();
        for(int t=0; t<=tend; t++) {
            State_T state = GetCurrentState();
            if(write(f_pipefd[1], &state, sizeof(State_T)) != sizeof(State_T)) {
                ::_exit(-1);
            }
            Advance();
        }
    }
    
    void fatal(const char* msg) {
        std::cerr << msg << std::endl;
        fData.clear();
        fFinished = true;
    }
    
    pid_t f_chpid;
    int f_pipefd[2];
    
    static const int BUF_SIZE = 16 * sizeof(State_T);
    char fBuf[BUF_SIZE];
    unsigned int fBufIdx;
    bool fFinished;
    
    std::vector<State_T> fData;
};

#endif
