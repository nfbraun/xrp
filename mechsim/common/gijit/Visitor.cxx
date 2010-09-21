#include "Visitor.h"
#include <stdexcept>
#include <sstream>
#include <llvm/IntrinsicInst.h>
#include <llvm/DerivedTypes.h>
#include <llvm/Support/IRBuilder.h>
#include <ginac/ginac.h>

namespace GiJIT {

void Visitor::registerSymbol(const GiNaC::symbol& sym, llvm::Value* value)
{
    if(!fCache.insert(std::make_pair(sym, value)).second)
        throw std::runtime_error("Symbol " + sym.get_name() + " registered twice");
}

void Visitor::acceptCached(const GiNaC::ex& ex)
{
    GiNaC::exhashmap<llvm::Value*>::iterator iter = fCache.find(ex);
    if(iter == fCache.end()) {
        ex.accept(*this);
        fCache.insert(std::make_pair(ex, fLast));
    } else {
        fLast = iter->second;
    }
}

void Visitor::visit(const GiNaC::add& ex)
{
    if(ex.nops() == 0)
        throw std::runtime_error("Internal error (add with no arguments)");
    
    acceptCached(ex.op(0));
    llvm::Value* retval = fLast;
    
    for(size_t i = 1; i < ex.nops(); i++) {
        acceptCached(ex.op(i));
        retval = fBuilder.CreateFAdd(retval, fLast);
    }
    
    fLast = retval;
}

void Visitor::visit(const GiNaC::mul& ex)
{
    if(ex.nops() == 0)
        throw std::runtime_error("Internal error (mul with no arguments)");
    
    acceptCached(ex.op(0));
    llvm::Value* retval = fLast;
    
    for(size_t i = 1; i < ex.nops(); i++) {
        acceptCached(ex.op(i));
        retval = fBuilder.CreateFMul(retval, fLast);
    }
    
    fLast = retval;
}

void Visitor::visit(const GiNaC::ncmul& ex)
{
    if(ex.nops() == 0)
        throw std::runtime_error("Internal error (ncmul with no arguments)");
    
    acceptCached(ex.op(0));
    llvm::Value* retval = fLast;
    
    for(size_t i = 1; i < ex.nops(); i++) {
        acceptCached(ex.op(i));
        retval = fBuilder.CreateFMul(retval, fLast);
    }
    
    fLast = retval;
}

void Visitor::visit(const GiNaC::power& ex)
{
    if(ex.nops() != 2)
        throw std::runtime_error("Internal error (power with more or less than two arguments)");
    
    acceptCached(ex.op(0));
    llvm::Value* x = fLast;
    llvm::Function* powfunc;
    llvm::Value* y;
    const llvm::Type* doubleTy = llvm::Type::getDoubleTy(fContext);
    
    if(GiNaC::is_a<GiNaC::numeric>(ex.op(1)) && 
       GiNaC::ex_to<GiNaC::numeric>(ex.op(1)).is_integer()) {
        powfunc = llvm::Intrinsic::getDeclaration(fModule, llvm::Intrinsic::powi,
            &doubleTy, 1);
        y = llvm::ConstantInt::get(llvm::Type::getInt32Ty(fContext),
            GiNaC::ex_to<GiNaC::numeric>(ex.op(1)).to_int());
        fLast = fBuilder.CreateCall2(powfunc, x, y);
    } else if(GiNaC::is_a<GiNaC::numeric>(ex.op(1)) && 
        GiNaC::ex_to<GiNaC::numeric>(ex.op(1)).to_double() == 0.5) {
        powfunc = llvm::Intrinsic::getDeclaration(fModule, llvm::Intrinsic::sqrt,
            &doubleTy, 1);
        fLast = fBuilder.CreateCall(powfunc, x);
    } else {
        powfunc = llvm::Intrinsic::getDeclaration(fModule, llvm::Intrinsic::pow,
            &doubleTy, 1);
        acceptCached(ex.op(1));
        y = fLast;
        fLast = fBuilder.CreateCall2(powfunc, x, y);
    }
}

void Visitor::visit(const GiNaC::symbol &ex)
{
    GiNaC::exhashmap<llvm::Value*>::iterator iter = fCache.find(ex);
    if(iter == fCache.end()) {
        throw std::runtime_error("Unknown symbol " + ex.get_name() + " in expression");
    }
    fLast = iter->second;
}

void Visitor::visit(const GiNaC::numeric& ex)
{
    fLast = llvm::ConstantFP::get(llvm::Type::getDoubleTy(fContext), ex.to_double());
}

void Visitor::visit(const GiNaC::function& ex)
{
    llvm::Function* func = fFuncFactory->get(ex.get_name());
    if(!func)
        throw std::runtime_error("Unknown function " + ex.get_name());
    
    if(func->getFunctionType()->getNumParams() != ex.nops())
        throw std::runtime_error("Internal error: number of function parameters do not match");
    
    std::vector<llvm::Value*> args;
    for(size_t i=0; i<ex.nops(); i++) {
        acceptCached(ex.op(i));
        args.push_back(fLast);
    }
    
    fLast = fBuilder.CreateCall(func, args.begin(), args.end());
}

void Visitor::visit(const GiNaC::matrix& ex)
{
    if(ex.rows() == 1 && ex.cols() == 1) {
        acceptCached(ex(0,0));
    } else {
        throw std::runtime_error("Got matrix where scalar expression was expected");
    }
}

void Visitor::visit(const GiNaC::basic& ex)
{
    std::ostringstream str;
    str << ex;
    throw std::runtime_error("Unhandled block: " + str.str());
}

} // end namespace GiJIT

