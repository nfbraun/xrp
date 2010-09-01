#ifndef _CODEGENIMP_H_
#define _CODEGENIMP_H_

#include <llvm/LLVMContext.h>
#include <llvm/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/Interpreter.h>
#include <llvm/ExecutionEngine/JIT.h>
#include <llvm/PassManager.h>
#include <vector>
#include <map>
#include <ginac/ginac.h>

#include "GiJIT.h"
#include "Visitor.h"

namespace GiJIT {

class CodeGenImp {
  public:
    static CodeGenImp* getCodeGenImp();
    ~CodeGenImp();
    
    void dump();
    void* compile(const ArgVector& args, bool returnNum,
                    GiNaC::ex ex = GiNaC::ex());
    
  private:
    CodeGenImp();
    
    const llvm::Type* getInputType(const Argument* arg);
    const llvm::Type* getInputType(const NumberArg* arg);
    const llvm::Type* getInputType(const VectorArg* arg);
    const llvm::Type* getInputType(const ResultArg* arg);
    
    void registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
        std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
        llvm::IRBuilder<>& builder, const Argument* arg);
    void registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
        std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
        llvm::IRBuilder<>& builder, const NumberArg* arg);
    void registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
        std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
        llvm::IRBuilder<>& builder, const VectorArg* arg);
    void registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
        std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
        llvm::IRBuilder<>& builder, const ResultArg* arg);
    
    llvm::Function* compileIR(const ArgVector& x, bool returnNum,
                              GiNaC::ex ex = GiNaC::ex());
    
    llvm::LLVMContext& fContext;
    llvm::Module* fModule;
    llvm::ExecutionEngine* fExEngine;
    
    llvm::FunctionPassManager* fFPM;
    
    static CodeGenImp* fGlobalCodeGenImp;
};

} // end namespace GiJIT

#endif
