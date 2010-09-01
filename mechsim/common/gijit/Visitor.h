#ifndef _VISITOR_H_
#define _VISITOR_H

#include <llvm/Module.h>
#include <llvm/LLVMContext.h>
#include <llvm/Support/IRBuilder.h>
#include <ginac/ginac.h>
#include "FuncFactory.h"

namespace GiJIT {

class Visitor
    : public GiNaC::visitor,
      public GiNaC::add::visitor,
      public GiNaC::mul::visitor,
      public GiNaC::ncmul::visitor,
      public GiNaC::power::visitor,
      public GiNaC::symbol::visitor,
      public GiNaC::numeric::visitor,
      public GiNaC::function::visitor,
      public GiNaC::matrix::visitor,
      public GiNaC::basic::visitor
{
    void visit(const GiNaC::add& ex);
    void visit(const GiNaC::mul& ex);
    void visit(const GiNaC::ncmul& ex);
    void visit(const GiNaC::power& ex);
    void visit(const GiNaC::symbol& ex);
    void visit(const GiNaC::numeric& ex);
    void visit(const GiNaC::function& ex);
    void visit(const GiNaC::matrix& ex);
    void visit(const GiNaC::basic& ex);
    
    void acceptCached(const GiNaC::ex& ex);
    
    GiNaC::exhashmap<llvm::Value*> fCache;
    
    llvm::Value* fLast;
    
    llvm::LLVMContext& fContext;
    llvm::Module* fModule;
    llvm::IRBuilder<>& fBuilder;
    
    FuncFactory fFuncFactory;
    
  public:
    Visitor(llvm::LLVMContext& context, llvm::Module* module,
            llvm::IRBuilder<>& builder)
        : fContext(context),
          fModule(module),
          fBuilder(builder),
          fFuncFactory(fContext, fModule)
        {}
    
    llvm::Value* getValue(const GiNaC::ex& ex)
        { acceptCached(ex); return fLast; }
    
    void registerSymbol(const GiNaC::symbol& sym, llvm::Value* value);
};

} // end namespace GiJIT

#endif
