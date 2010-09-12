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
#include "CType.h"
#include "Visitor.h"

namespace GiJIT {

class CodeGenImp {
  public:
    static CodeGenImp* getCodeGenImp();
    ~CodeGenImp();
    
    void dump();
    void* compile(const ArgVector& args, const Return* ret);
    
  private:
    CodeGenImp();
    
    const llvm::Type* getInputType(const Argument* arg);
    const llvm::Type* getReturnType(const Return* ret);
    
    const llvm::Type* convertCType(const CType_Any* type);
    
    void registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
        std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
        llvm::IRBuilder<>& builder, const Argument* arg);
    void createResultStore(Visitor& v,
        std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
        llvm::IRBuilder<>& builder, const ResultArg* result);
    void createRet(Visitor& v, llvm::IRBuilder<>& builder, const Return* ret);
    
    llvm::Function* compileIR(const ArgVector& x, const Return* ret);
    
    llvm::LLVMContext& fContext;
    llvm::Module* fModule;
    llvm::ExecutionEngine* fExEngine;
    
    llvm::FunctionPassManager* fFPM;
    
    static CodeGenImp* fGlobalCodeGenImp;
};

} // end namespace GiJIT

#endif
