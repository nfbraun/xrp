#include <vector>
#include <iostream>
#include <stdexcept>
#include <stdint.h>
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

void* compileImp(const ArgVector& args, const Return* ret)
{
    return CodeGenImp::getCodeGenImp()->compile(args, ret);
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

const llvm::Type* CodeGenImp::convertCType(const CType_Any* type)
{
    if(dynamic_cast< const CType<void>* >(type))
        return llvm::Type::getVoidTy(fContext);
    else if(dynamic_cast< const CType<int8_t>* >(type))
        return llvm::Type::getInt8Ty(fContext);
    else if(dynamic_cast< const CType<int16_t>* >(type))
        return llvm::Type::getInt16Ty(fContext);
    else if(dynamic_cast< const CType<int32_t>* >(type))
        return llvm::Type::getInt32Ty(fContext);
    else if(dynamic_cast< const CType<int64_t>* >(type))
        return llvm::Type::getInt64Ty(fContext);
    else if(dynamic_cast< const CType<float>* >(type))
        return llvm::Type::getFloatTy(fContext);
    else if(dynamic_cast< const CType<double>* >(type))
        return llvm::Type::getDoubleTy(fContext);
    else if(dynamic_cast< const CType<void*>* >(type))
        // void* is not available in llvm, manual suggests using i8* instead
        return llvm::Type::getInt8PtrTy(fContext);
    else if(dynamic_cast< const CType<int8_t*>* >(type))
        return llvm::Type::getInt8PtrTy(fContext);
    else if(dynamic_cast< const CType<int16_t*>* >(type))
        return llvm::Type::getInt16PtrTy(fContext);
    else if(dynamic_cast< const CType<int32_t*>* >(type))
        return llvm::Type::getInt32PtrTy(fContext);
    else if(dynamic_cast< const CType<int64_t*>* >(type))
        return llvm::Type::getInt64PtrTy(fContext);
    else if(dynamic_cast< const CType<float*>* >(type))
        return llvm::Type::getFloatPtrTy(fContext);
    else if(dynamic_cast< const CType<double*>* >(type))
        return llvm::Type::getDoublePtrTy(fContext);
    else
        throw std::runtime_error("Unknown C type");
}

const llvm::Type* CodeGenImp::getInputType(const Argument* arg)
{
    if(dynamic_cast<const NumberArg*>(arg))
        return llvm::Type::getDoubleTy(fContext);
    else if(dynamic_cast<const VectorArg*>(arg))
        return llvm::Type::getDoublePtrTy(fContext);
    else if(dynamic_cast<const ResultArg*>(arg))
        return llvm::Type::getDoublePtrTy(fContext);
    else if(dynamic_cast<const DummyArg_Any*>(arg))
        return convertCType((dynamic_cast<const DummyArg_Any*>(arg))->getCType());
    else
        throw std::runtime_error("Internal error: unknown argument type");
}

const llvm::Type* CodeGenImp::getReturnType(const Return* ret)
{
    if(dynamic_cast<const ReturnVoid*>(ret))
        return llvm::Type::getVoidTy(fContext);
    else if(dynamic_cast<const ReturnNumber*>(ret))
        return llvm::Type::getDoubleTy(fContext);
    else if(dynamic_cast<const ReturnDummyNumber*>(ret))
        return llvm::Type::getDoubleTy(fContext);
    else if(dynamic_cast<const ReturnDummyInt*>(ret))
        return llvm::Type::getInt32Ty(fContext);
    else
        throw std::runtime_error("Internal error: unknown return type");
}

void CodeGenImp::registerArguments(Visitor& v, llvm::Function::arg_iterator& pos,
    std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
    llvm::IRBuilder<>& builder, const Argument* arg)
{
    if(dynamic_cast<const NumberArg*>(arg)) {
        v.registerSymbol((dynamic_cast<const NumberArg*>(arg))->getSym(), pos++);
    } else if(dynamic_cast<const VectorArg*>(arg)) {
        int off = 0;
        const std::vector<GiNaC::symbol>& syms =
            (dynamic_cast<const VectorArg*>(arg))->getSyms();
        
        for(std::vector<GiNaC::symbol>::const_iterator s = syms.begin();
            s != syms.end(); s++) {
            llvm::Value* offV = llvm::ConstantInt::get(
               llvm::Type::getInt32Ty(fContext), off++);
            llvm::Value* ptr = builder.CreateGEP(pos, offV);
            llvm::Value* val = builder.CreateLoad(ptr);
            
            v.registerSymbol(*s, val);
        }
        
        pos++;
    } else if(dynamic_cast<const ResultArg*>(arg)) {
        resultArgs[dynamic_cast<const ResultArg*>(arg)] = pos++;
    } else if(dynamic_cast<const DummyArg_Any*>(arg)) {
        pos++;
    }
}

void CodeGenImp::createResultStore(Visitor& v,
    std::map<const Argument*, llvm::Function::arg_iterator>& resultArgs,
    llvm::IRBuilder<>& builder, const ResultArg* result)
{
    llvm::Function::arg_iterator apos = resultArgs[result];
    int off = 0;
    const GiNaC::ex& ex = result->getEx();
    
    if(GiNaC::is_a<GiNaC::lst>(ex)) {
        const GiNaC::lst& lst = GiNaC::ex_to<GiNaC::lst>(ex);
        for(GiNaC::lst::const_iterator elem = lst.begin();
            elem != lst.end(); elem++) {
            llvm::Value* offV = llvm::ConstantInt::get(
                    llvm::Type::getInt32Ty(fContext), off++);
            llvm::Value* ptr = builder.CreateGEP(apos, offV);
            builder.CreateStore(v.getValue(*elem), ptr);
        }
    } else if(GiNaC::is_a<GiNaC::matrix>(ex)) {
        const GiNaC::matrix& matrix = GiNaC::ex_to<GiNaC::matrix>(ex);
        for(size_t elem = 0; elem != matrix.nops(); elem++) {
            llvm::Value* offV = llvm::ConstantInt::get(
                    llvm::Type::getInt32Ty(fContext), off++);
            llvm::Value* ptr = builder.CreateGEP(apos, offV);
            builder.CreateStore(v.getValue(ex.op(elem)), ptr);
        }
    } else {
        llvm::Value* offV = llvm::ConstantInt::get(
                llvm::Type::getInt32Ty(fContext), 0);
        llvm::Value* ptr = builder.CreateGEP(apos, offV);
        builder.CreateStore(v.getValue(ex), ptr);
    }
}

void CodeGenImp::createRet(Visitor& v, llvm::IRBuilder<>& builder,
    const Return* ret)
{
    if(dynamic_cast<const ReturnVoid*>(ret)) {
        builder.CreateRetVoid();
    } else {
        llvm::Value* retval;
        if(dynamic_cast<const ReturnNumber*>(ret)) {
            retval = v.getValue((dynamic_cast<const ReturnNumber*>(ret))->getEx());
        } else if(dynamic_cast<const ReturnDummyNumber*>(ret)) {
            retval = llvm::ConstantFP::get(llvm::Type::getDoubleTy(fContext),
                (dynamic_cast<const ReturnDummyNumber*>(ret))->getX());
        } else if(dynamic_cast<const ReturnDummyInt*>(ret)) {
            retval = llvm::ConstantInt::get(llvm::Type::getInt32Ty(fContext),
                (dynamic_cast<const ReturnDummyInt*>(ret))->getX());
        } else {
            throw std::runtime_error("Internal error: unhandled return type");
        }
        builder.CreateRet(retval);
    }
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
    
    fFuncFactory = new FuncFactory(fContext, fModule);
}

CodeGenImp::~CodeGenImp()
{
    delete fFuncFactory;
    fFPM = 0;
}

void CodeGenImp::dump()
{
    fModule->dump();
}

llvm::Function* CodeGenImp::compileIR(const ArgVector& args, const Return* ret)
{
    std::vector<const llvm::Type*> argTypes;
    for(ArgVector::const_iterator arg = args.begin(); arg != args.end(); arg++) {
        argTypes.push_back(getInputType(*arg));
    }
    
    llvm::FunctionType* ft = llvm::FunctionType::get(
            getReturnType(ret), argTypes, false);
    
    llvm::Function* func = llvm::Function::Create(ft,
                  llvm::Function::ExternalLinkage, "", fModule);

    llvm::BasicBlock* bb = llvm::BasicBlock::Create(fContext,
                                                    "entry", func);
    llvm::IRBuilder<> builder(fContext);
    
    builder.SetInsertPoint(bb);
    
    Visitor v(fContext, fModule, builder, fFuncFactory);
    llvm::Function::arg_iterator ai = func->arg_begin();
    
    std::map<const Argument*, llvm::Function::arg_iterator> resultArgs;
    for(ArgVector::const_iterator arg = args.begin(); arg != args.end(); arg++) {
        registerArguments(v, ai, resultArgs, builder, *arg);
    }
    if(ai != func->arg_end())
        throw std::runtime_error("Internal error");
    
    for(ArgVector::const_iterator arg = args.begin(); arg != args.end(); arg++) {
        const ResultArg* result = dynamic_cast<const ResultArg*>(*arg);
        if(result)
            createResultStore(v, resultArgs, builder, result);
    }
    
    createRet(v, builder, ret);
    
    return func;
}

void* CodeGenImp::compile(const ArgVector& args, const Return* ret)
{
    llvm::Function* func = compileIR(args, ret);
    
    llvm::verifyFunction(*func);
    fFPM->run(*func);
    
    return fExEngine->getPointerToFunction(func);
}

} // end namespace GiJIT
