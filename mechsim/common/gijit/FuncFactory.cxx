#include <llvm/IntrinsicInst.h>

#include "FuncFactory.h"

namespace GiJIT {

llvm::Function* FuncFactory::get(const std::string& name)
{
    std::map<std::string, llvm::Function*>::iterator iter;
    iter = fFunctions.find(name);
    
    if(iter != fFunctions.end())
        return iter->second;
    
    const llvm::Type* doubleTy = llvm::Type::getDoubleTy(fContext);
    llvm::Function* func;
    
    // llvm intrinsics
    if(name == "sin") {
        func = llvm::Intrinsic::getDeclaration(fModule, llvm::Intrinsic::sin,
            &doubleTy, 1);
    } else if(name == "cos") {
        func = llvm::Intrinsic::getDeclaration(fModule, llvm::Intrinsic::cos,
            &doubleTy, 1);
    }
    // libm functions
    else if(name == "exp" || name == "log" || name == "tan" ||
            name == "asin"|| name == "acos" || name == "atan" ||
            name == "sinh" || name == "cosh" || name == "tanh" ||
            name == "asinh" || name == "acosh" || name == "atanh" ||
            name == "tgamma" || name == "lgamma") {
        llvm::FunctionType* FT = llvm::FunctionType::get(
            doubleTy, std::vector<const llvm::Type*>(1, doubleTy),
            false);
        func = llvm::Function::Create(FT, llvm::Function::ExternalLinkage,
            name, fModule);
    } else {
        return 0;
    }
    
    fFunctions.insert(std::make_pair(name, func));
    
    return func;
}

} // end namespace GiJIT
