#include <vector>
#include <iostream>
#include <stdexcept>
#include <llvm/DerivedTypes.h>
#include <llvm/Support/IRBuilder.h>
#include <llvm/Target/TargetSelect.h>
#include <llvm/Analysis/Verifier.h>
#include <llvm/Target/TargetData.h>
#include <llvm/Transforms/Scalar.h>

#include "CodeGenImp.h"

namespace GiJIT {

CodeGenImp* CodeGenImp::fGlobalCodeGenImp = 0;

CodeGenImp* CodeGenImp::getCodeGenImp()
{
    if(!fGlobalCodeGenImp)
        fGlobalCodeGenImp = new CodeGenImp();
    return fGlobalCodeGenImp;
}

void dump()
{
    CodeGenImp::getCodeGenImp()->dump();
}

void* compileImp(const ArgVector& args, bool returnNum, GiNaC::ex ex)
{
    return CodeGenImp::getCodeGenImp()->compile(args, returnNum, ex);
}

VectorArg::VectorArg(const GiNaC::lst& slst)
{
    for(GiNaC::lst::const_iterator sym = slst.begin(); sym != slst.end(); sym++) {
        if(!GiNaC::is_a<GiNaC::symbol>(*sym)) {
            throw std::runtime_error("Found generic expression where symbol was expected");
        }
        syms.push_back(GiNaC::ex_to<GiNaC::symbol>(*sym));
    }
}

const llvm::Type* CodeGenImp::getInputType(const Argument* arg)
{
    if(dynamic_cast<const NumberArg*>(arg))
        return getInputType(dynamic_cast<const NumberArg*>(arg));
    else if(dynamic_cast<const VectorArg*>(arg))
        return getInputType(dynamic_cast<const VectorArg*>(arg));
    else if(dynamic_cast<const ResultArg*>(arg))
        return getInputType(dynamic_cast<const ResultArg*>(arg));
}

const llvm::Type* CodeGenImp::getInputType(const NumberArg* arg)
{
    return llvm::Type::getDoubleTy(fContext);
}

const llvm::Type* CodeGenImp::getInputType(const VectorArg* arg)
{
    return llvm::Type::getDoublePtrTy(fContext);
}

const llvm::Type* CodeGenImp::getInputType(const ResultArg* arg)
{
    return llvm::Type::getDoublePtrTy(fContext);
}

void CodeGenImp::registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
    std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
    llvm::IRBuilder<>& builder, const Argument* arg)
{
    if(dynamic_cast<const NumberArg*>(arg))
        registerArguments(v, pos, resultArgs, builder,
            dynamic_cast<const NumberArg*>(arg));
    else if(dynamic_cast<const VectorArg*>(arg))
        registerArguments(v, pos, resultArgs, builder,
            dynamic_cast<const VectorArg*>(arg));
    else if(dynamic_cast<const ResultArg*>(arg))
        registerArguments(v, pos, resultArgs, builder,
            dynamic_cast<const ResultArg*>(arg));
}

void CodeGenImp::registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
    std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
    llvm::IRBuilder<>& builder, const NumberArg* arg)
{
    v.registerSymbol(arg->getSym(), pos++);
}

void CodeGenImp::registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
    std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
    llvm::IRBuilder<>& builder, const VectorArg* arg)
{
    int off = 0;
    
    for(std::vector<GiNaC::symbol>::const_iterator s = arg->getSyms().begin();
        s != arg->getSyms().end(); s++) {
        llvm::Value* offV = llvm::ConstantInt::get(
           llvm::Type::getInt32Ty(fContext), off++);
        llvm::Value* ptr = builder.CreateGEP(pos, offV);
        llvm::Value* val = builder.CreateLoad(ptr);
        
        v.registerSymbol(*s, val);
    }
    
    pos++;
}

void CodeGenImp::registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
    std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
    llvm::IRBuilder<>& builder, const ResultArg* arg)
{
    resultArgs[arg] = pos++;
}

CodeGenImp::CodeGenImp()
    : fContext(llvm::getGlobalContext())
{
    llvm::InitializeNativeTarget();
    
    fModule = new llvm::Module("GiJIT", fContext);
    fFPM = new llvm::FunctionPassManager(fModule);
    
    std::string errstr;
    fExEngine = llvm::EngineBuilder(fModule).setErrorStr(&errstr).create();
    if(!fExEngine) {
        delete fModule;
        throw std::runtime_error("Could not create execution engine: " + errstr);
    }
    
    fFPM->add(new llvm::TargetData(*fExEngine->getTargetData()));
    //fFPM->add(llvm::createGVNPass());
    fFPM->doInitialization();
}

CodeGenImp::~CodeGenImp()
{
    fFPM = 0;
}

void CodeGenImp::dump()
{
    fModule->dump();
}

llvm::Function* CodeGenImp::compileIR(const ArgVector& args, bool returnNum,
                                   GiNaC::ex returnEx)
{
    std::vector<const llvm::Type*> argTypes;
    for(ArgVector::const_iterator arg = args.begin(); arg != args.end(); arg++) {
        argTypes.push_back(getInputType(*arg));
    }
    
    const llvm::Type* retType;
    if(returnNum)
        retType = llvm::Type::getDoubleTy(fContext);
    else
        retType = llvm::Type::getVoidTy(fContext);
    
    llvm::FunctionType* ft = llvm::FunctionType::get(
            retType, argTypes, false);
    
    llvm::Function* func = llvm::Function::Create(ft,
                  llvm::Function::ExternalLinkage, "", fModule);

    llvm::BasicBlock* bb = llvm::BasicBlock::Create(fContext,
                                                    "entry", func);
    llvm::IRBuilder<> builder(fContext);
    
    builder.SetInsertPoint(bb);
    
    Visitor v(fContext, fModule, builder);
    llvm::Function::arg_iterator ai = func->arg_begin();
    
    std::map<const Argument*, llvm::Function::arg_iterator> resultArgs;
    for(ArgVector::const_iterator arg = args.begin(); arg != args.end(); arg++) {
        registerArguments(v, ai, resultArgs, builder, *arg);
    }
    if(ai != func->arg_end())
        throw std::runtime_error("Internal error");
    
    for(ArgVector::const_iterator arg = args.begin(); arg != args.end(); arg++) {
        const ResultArg* result = dynamic_cast<const ResultArg*>(*arg);
        if(result) {
            int off = 0;
            GiNaC::ex ex = result->getEx();
            
            if(GiNaC::is_a<GiNaC::lst>(ex)) {
                GiNaC::lst lst = GiNaC::ex_to<GiNaC::lst>(ex);
                for(GiNaC::lst::const_iterator elem = lst.begin();
                    elem != lst.end(); elem++) {
                    llvm::Value* offV = llvm::ConstantInt::get(
                            llvm::Type::getInt32Ty(fContext), off++);
                    llvm::Value* ptr = builder.CreateGEP(resultArgs[result], offV);
                    llvm::Value* val = builder.CreateStore(v.getValue(*elem), ptr);
                }
            } else {
                llvm::Value* offV = llvm::ConstantInt::get(
                        llvm::Type::getInt32Ty(fContext), 0);
                llvm::Value* ptr = builder.CreateGEP(resultArgs[result], offV);
                llvm::Value* val = builder.CreateStore(v.getValue(ex), ptr);
            }
        }
    }
    
    if(returnNum) {
        builder.CreateRet(v.getValue(returnEx));
    } else {
        builder.CreateRetVoid();
    }
    
    return func;
}

void* CodeGenImp::compile(const ArgVector& args, bool returnNum, GiNaC::ex ex)
{
    llvm::Function* func = compileIR(args, returnNum, ex);
    
    llvm::verifyFunction(*func);
    fFPM->run(*func);
    
    return fExEngine->getPointerToFunction(func);
}

} // end namespace GiJIT
