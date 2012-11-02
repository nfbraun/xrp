#ifndef MSIM_ASYNCSIMRUNNER_H
#define MSIM_ASYNCSIMRUNNER_H

#include "SimRunner.h"
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#define DEBUG_ASYNCSIMULATION
// Runner for simulations that take a while to complete, and are computed
// asynchronously during playback
template<class Sim_T, class State_T>
class AsyncSimRunner: public SimRunner
{
  public:
    AsyncSimRunner(Sim_T* sim) : fSim(sim), fFinished(false) {
        f_pipefd[0] = -1;
        f_pipefd[1] = -1;
    }
    
    virtual double GetTimestep()
        { return fSim->GetTimestep(); }    // in units of seconds
    virtual int GetDefaultEndTime()
        { return fSim->GetDefaultEndTime(); } // in units of TIMESTEPs

    virtual const char* GetTitle()
        { return fSim->GetTitle(); }
    
    virtual const SimulationState* GetState(int t)  // t in units of TIMESTEPs
    {
        if(t < 0 || ((unsigned int) t) >= fData.size()) return 0;
        else return &fData[t];
    }
    
    // Interface for draw modes
    virtual int GetNDrawModes() const { return fSim->GetNDrawModes(); }
    virtual const char* GetDrawModeName(int id) { return fSim->GetDrawModeName(id); }
    
    // Interface for data display
    virtual int GetNDataCh() const { return fSim->GetNDataCh(); }
    virtual const char* GetChName(int ch) const { return fSim->GetChName(ch); }
    
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
    
  private:
    // Instances of this class must not be copied
    AsyncSimRunner(const AsyncSimRunner& src) { assert(false); }
    AsyncSimRunner& operator=(const AsyncSimRunner& src) { assert(false); return *this; }
    
    void RunSimulation() {
        int tend = GetDefaultEndTime();
        for(int t=0; t<=tend; t++) {
            State_T state = fSim->GetCurrentState();
            if(write(f_pipefd[1], &state, sizeof(State_T)) != sizeof(State_T)) {
                ::_exit(-1);
            }
            fSim->Advance();
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
    char fBuf[BUF_SIZE] __attribute__((aligned(16)));
    unsigned int fBufIdx;
    bool fFinished;
    
    Sim_T* fSim;
    
    std::vector<State_T, Eigen::aligned_allocator<State_T> > fData;
};

#endif
