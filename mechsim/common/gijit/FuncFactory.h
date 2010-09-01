#ifndef _FUNCFACTORY_H_
#define _FUNCFACTORY_H_

#include <string>
#include <map>
#include <llvm/LLVMContext.h>
#include <llvm/Module.h>

namespace GiJIT {

class FuncFactory {
  public:
    FuncFactory(llvm::LLVMContext& context, llvm::Module* module)
        : fContext(context), fModule(module) {}
    llvm::Function* get(const std::string& name);
    
  private:
    llvm::LLVMContext& fContext;
    llvm::Module* fModule;
    std::map<std::string, llvm::Function*> fFunctions;
};

} // end namespace GiJIT

#endif
