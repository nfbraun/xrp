#ifndef _GIJIT_H_
#define _GIJIT_H_

#include "CType.h"
#include <ginac/ginac.h>

namespace GiJIT {

class Argument {
  public:
    virtual void dummy() = 0;
};

class NumberArg: public Argument {
  public:
    typedef double func_arg_t;
    
    NumberArg(const GiNaC::symbol& _x): x(_x) {}
    virtual void dummy() {}
    inline const GiNaC::symbol& getSym() const { return x; }
  private:
    GiNaC::symbol x;
};

class VectorArg: public Argument {
  public:
    typedef const double* func_arg_t;
    
    VectorArg(const std::vector<GiNaC::symbol>& _syms): syms(_syms) {}
    VectorArg(const GiNaC::lst& slst);
    virtual void dummy() {}
    inline const std::vector<GiNaC::symbol>& getSyms() const { return syms; }
  private:
    std::vector<GiNaC::symbol> syms;
};

class ResultArg: public Argument {
  public:
    typedef double* func_arg_t;
    
    ResultArg(const GiNaC::ex& _ex): ex(_ex) {}
    ResultArg(const GiNaC::lst& _ex): ex(_ex) {}
    ResultArg(const GiNaC::matrix& _ex): ex(_ex) {}
    virtual void dummy() {}
    inline const GiNaC::ex& getEx() const { return ex; }
  private:
    GiNaC::ex ex;
};

class DummyArg_Any: public Argument {
  public:
    virtual void dummy() {}
    virtual const CType_Any* getCType() const = 0;
};

template<typename T>
class DummyArg: public DummyArg_Any {
  public:
    typedef T func_arg_t;
    
    DummyArg() {}
    DummyArg(void*) {}
    DummyArg(int) {}
    virtual const CType_Any* getCType() const { return &ct; }
  private:
    CType<T> ct;
};

typedef NumberArg Number;
typedef VectorArg Vector;
typedef ResultArg Result;

class Return {
  public:
    virtual void dummy() = 0;
};

class ReturnVoid: public Return {
  public:
    typedef void func_ret_t;
    
    ReturnVoid() {}
    virtual void dummy() {}
};

class ReturnNumber: public Return {
  public:
    typedef double func_ret_t;
    
    ReturnNumber(const GiNaC::ex& _ex): ex(_ex) {}
    virtual void dummy() {}
    inline const GiNaC::ex& getEx() const { return ex; }
  private:
    GiNaC::ex ex;
};

class ReturnDummyNumber: public Return {
  public:
    typedef double func_ret_t;
    
    ReturnDummyNumber(double _x): x(_x) {}
    virtual void dummy() {}
    inline double getX() const { return x; }
  private:
    double x;
};

class ReturnDummyInt: public Return {
  public:
    typedef int func_ret_t;
    
    ReturnDummyInt(int _x): x(_x) {}
    virtual void dummy() {}
    inline int getX() const { return x; }
  private:
    int x;
};

typedef std::vector<const Argument*> ArgVector;

void dump();
void* compileImp(const ArgVector& args, const Return* ret);

#define GIJIT_FUNC_ARG(n) typename T##n::func_arg_t
#define GIJIT_FUNC_ARGS_0 
#define GIJIT_FUNC_ARGS_1 GIJIT_FUNC_ARG(1)
#define GIJIT_FUNC_ARGS_2 GIJIT_FUNC_ARGS_1, GIJIT_FUNC_ARG(2)
#define GIJIT_FUNC_ARGS_3 GIJIT_FUNC_ARGS_2, GIJIT_FUNC_ARG(3)
#define GIJIT_FUNC_ARGS_4 GIJIT_FUNC_ARGS_3, GIJIT_FUNC_ARG(4)
#define GIJIT_FUNC_ARGS_5 GIJIT_FUNC_ARGS_4, GIJIT_FUNC_ARG(5)
#define GIJIT_FUNC_ARGS_6 GIJIT_FUNC_ARGS_5, GIJIT_FUNC_ARG(6)
#define GIJIT_FUNC_ARGS_7 GIJIT_FUNC_ARGS_6, GIJIT_FUNC_ARG(7)
#define GIJIT_FUNC_ARGS_8 GIJIT_FUNC_ARGS_7, GIJIT_FUNC_ARG(8)

#define GIJIT_COMPILE_ARG(n) T##n a##n
#define GIJIT_COMPILE_ARGS_0 
#define GIJIT_COMPILE_ARGS_1 GIJIT_COMPILE_ARG(1)
#define GIJIT_COMPILE_ARGS_2 GIJIT_COMPILE_ARGS_1, GIJIT_COMPILE_ARG(2)
#define GIJIT_COMPILE_ARGS_3 GIJIT_COMPILE_ARGS_2, GIJIT_COMPILE_ARG(3)
#define GIJIT_COMPILE_ARGS_4 GIJIT_COMPILE_ARGS_3, GIJIT_COMPILE_ARG(4)
#define GIJIT_COMPILE_ARGS_5 GIJIT_COMPILE_ARGS_4, GIJIT_COMPILE_ARG(5)
#define GIJIT_COMPILE_ARGS_6 GIJIT_COMPILE_ARGS_5, GIJIT_COMPILE_ARG(6)
#define GIJIT_COMPILE_ARGS_7 GIJIT_COMPILE_ARGS_6, GIJIT_COMPILE_ARG(7)
#define GIJIT_COMPILE_ARGS_8 GIJIT_COMPILE_ARGS_7, GIJIT_COMPILE_ARG(8)

#define GIJIT_SEP ,

#define GIJIT_BUILD_ARG_VECTOR_N(n) args.push_back(&a##n);

#define GIJIT_BUILD_ARG_VECTOR_0 
#define GIJIT_BUILD_ARG_VECTOR_1 GIJIT_BUILD_ARG_VECTOR_N(1)
#define GIJIT_BUILD_ARG_VECTOR_2 GIJIT_BUILD_ARG_VECTOR_1 GIJIT_BUILD_ARG_VECTOR_N(2)
#define GIJIT_BUILD_ARG_VECTOR_3 GIJIT_BUILD_ARG_VECTOR_2 GIJIT_BUILD_ARG_VECTOR_N(3)
#define GIJIT_BUILD_ARG_VECTOR_4 GIJIT_BUILD_ARG_VECTOR_3 GIJIT_BUILD_ARG_VECTOR_N(4)
#define GIJIT_BUILD_ARG_VECTOR_5 GIJIT_BUILD_ARG_VECTOR_4 GIJIT_BUILD_ARG_VECTOR_N(5)
#define GIJIT_BUILD_ARG_VECTOR_6 GIJIT_BUILD_ARG_VECTOR_5 GIJIT_BUILD_ARG_VECTOR_N(6)
#define GIJIT_BUILD_ARG_VECTOR_7 GIJIT_BUILD_ARG_VECTOR_6 GIJIT_BUILD_ARG_VECTOR_N(7)
#define GIJIT_BUILD_ARG_VECTOR_8 GIJIT_BUILD_ARG_VECTOR_7 GIJIT_BUILD_ARG_VECTOR_N(8)

#define GIJIT_CODE_GEN_R(FUNC_ARGS, COMPILE_ARGS, BUILD_ARG_VECTOR) \
  public: \
    typedef typename TR::func_ret_t (*func_t)(FUNC_ARGS); \
    static func_t compile(TR ar COMPILE_ARGS) \
    { \
        ArgVector args; \
        BUILD_ARG_VECTOR \
        return (func_t) compileImp(args, &ar); \
    } \
}; \

template<typename TR,
         typename T1=void, typename T2=void, typename T3=void, typename T4=void,
         typename T5=void, typename T6=void, typename T7=void, typename T8=void>
class CodeGenR {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_8, GIJIT_SEP GIJIT_COMPILE_ARGS_8, GIJIT_BUILD_ARG_VECTOR_8)

template<typename TR, typename T1, typename T2, typename T3, typename T4,
         typename T5, typename T6, typename T7>
class CodeGenR<TR, T1, T2, T3, T4, T5, T6, T7, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_7, GIJIT_SEP GIJIT_COMPILE_ARGS_7, GIJIT_BUILD_ARG_VECTOR_7)

template<typename TR, typename T1, typename T2, typename T3, typename T4,
         typename T5, typename T6>
class CodeGenR<TR, T1, T2, T3, T4, T5, T6, void, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_6, GIJIT_SEP GIJIT_COMPILE_ARGS_6, GIJIT_BUILD_ARG_VECTOR_6)

template<typename TR, typename T1, typename T2, typename T3, typename T4,
         typename T5>
class CodeGenR<TR, T1, T2, T3, T4, T5, void, void, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_5, GIJIT_SEP GIJIT_COMPILE_ARGS_5, GIJIT_BUILD_ARG_VECTOR_5)

template<typename TR, typename T1, typename T2, typename T3, typename T4>
class CodeGenR<TR, T1, T2, T3, T4, void, void, void, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_4, GIJIT_SEP GIJIT_COMPILE_ARGS_4, GIJIT_BUILD_ARG_VECTOR_4)

template<typename TR, typename T1, typename T2, typename T3>
class CodeGenR<TR, T1, T2, T3, void, void, void, void, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_3, GIJIT_SEP GIJIT_COMPILE_ARGS_3, GIJIT_BUILD_ARG_VECTOR_3)

template<typename TR, typename T1, typename T2>
class CodeGenR<TR, T1, T2, void, void, void, void, void, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_2, GIJIT_SEP GIJIT_COMPILE_ARGS_2, GIJIT_BUILD_ARG_VECTOR_2)

template<typename TR, typename T1>
class CodeGenR<TR, T1, void, void, void, void, void, void, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_1, GIJIT_SEP GIJIT_COMPILE_ARGS_1, GIJIT_BUILD_ARG_VECTOR_1)

template<typename TR>
class CodeGenR<TR, void, void, void, void, void, void, void, void> {
GIJIT_CODE_GEN_R(GIJIT_FUNC_ARGS_0, GIJIT_COMPILE_ARGS_0, GIJIT_BUILD_ARG_VECTOR_0)

template<typename T1=void, typename T2=void, typename T3=void, typename T4=void,
         typename T5=void, typename T6=void, typename T7=void, typename T8=void>
class CodeGen: public CodeGenR<ReturnNumber, T1, T2, T3, T4, T5, T6, T7, T8> {};

#define GIJIT_CODE_GEN_V(FUNC_ARGS, COMPILE_ARGS, BUILD_ARG_VECTOR) \
  public: \
    typedef void (*func_t)(FUNC_ARGS); \
    static func_t compile(COMPILE_ARGS) \
    { \
        ArgVector args; \
        BUILD_ARG_VECTOR \
        ReturnVoid rv; \
        return (func_t) compileImp(args, &rv); \
    } \
}; \

template<typename T1=void, typename T2=void, typename T3=void, typename T4=void,
         typename T5=void, typename T6=void, typename T7=void, typename T8=void>
class CodeGenV {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_8, GIJIT_COMPILE_ARGS_8, GIJIT_BUILD_ARG_VECTOR_8)

template<typename T1, typename T2, typename T3, typename T4,
         typename T5, typename T6, typename T7>
class CodeGenV<T1, T2, T3, T4, T5, T6, T7, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_7, GIJIT_COMPILE_ARGS_7, GIJIT_BUILD_ARG_VECTOR_7)

template<typename T1, typename T2, typename T3, typename T4,
         typename T5, typename T6>
class CodeGenV<T1, T2, T3, T4, T5, T6, void, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_6, GIJIT_COMPILE_ARGS_6, GIJIT_BUILD_ARG_VECTOR_6)

template<typename T1, typename T2, typename T3, typename T4,
         typename T5>
class CodeGenV<T1, T2, T3, T4, T5, void, void, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_5, GIJIT_COMPILE_ARGS_5, GIJIT_BUILD_ARG_VECTOR_5)

template<typename T1, typename T2, typename T3, typename T4>
class CodeGenV<T1, T2, T3, T4, void, void, void, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_4, GIJIT_COMPILE_ARGS_4, GIJIT_BUILD_ARG_VECTOR_4)

template<typename T1, typename T2, typename T3>
class CodeGenV<T1, T2, T3, void, void, void, void, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_3, GIJIT_COMPILE_ARGS_3, GIJIT_BUILD_ARG_VECTOR_3)

template<typename T1, typename T2>
class CodeGenV<T1, T2, void, void, void, void, void, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_2, GIJIT_COMPILE_ARGS_2, GIJIT_BUILD_ARG_VECTOR_2)

template<typename T1>
class CodeGenV<T1, void, void, void, void, void, void, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_1, GIJIT_COMPILE_ARGS_1, GIJIT_BUILD_ARG_VECTOR_1)

template<>
class CodeGenV<void, void, void, void, void, void, void, void> {
GIJIT_CODE_GEN_V(GIJIT_FUNC_ARGS_0, GIJIT_COMPILE_ARGS_0, GIJIT_BUILD_ARG_VECTOR_0)

} // end namespace GiJIT

#endif
