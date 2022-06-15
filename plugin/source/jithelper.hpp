/*
jitHelper.hpp
Auxiliary class for LLVM/Clang operations.
*/
#pragma	once

#include <clang/AST/ASTContext.h>
#include <clang/AST/ASTConsumer.h>
#include <clang/Basic/DiagnosticOptions.h>
#include <clang/Basic/Diagnostic.h>
#include <clang/Basic/FileManager.h>
#include <clang/Basic/FileSystemOptions.h>
#include <clang/Basic/LangOptions.h>
#include <clang/Basic/SourceManager.h>
#include <clang/Basic/TargetInfo.h>
#include <clang/CodeGen/CodeGenAction.h>
#include <clang/Frontend/CompilerInstance.h>
#include <clang/Frontend/CompilerInvocation.h>
#include <clang/Frontend/TextDiagnosticPrinter.h>
#include <clang/Lex/HeaderSearch.h>
#include <clang/Lex/HeaderSearchOptions.h>
#include <clang/Lex/Preprocessor.h>
#include <clang/Lex/PreprocessorOptions.h>
#include <clang/Parse/ParseAST.h>
#include <clang/Sema/Sema.h>

#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Bitcode/BitcodeWriter.h"
#include "llvm/Support/CrashRecoveryContext.h"
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/GenericValue.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/ExecutionEngine/Orc/ExecutionUtils.h>
#include "llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h"
#include <llvm/ExecutionEngine/SectionMemoryManager.h>
#include <llvm/InitializePasses.h>
#include <llvm/Support/Casting.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/Object/ObjectFile.h>
#include <llvm/Object/Archive.h>
#include <llvm/Linker/Linker.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/SourceMgr.h>
#include <llvm/Support/DynamicLibrary.h>
#include <llvm/Passes/PassBuilder.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO.h>

#include <utility>

using namespace std;
using namespace llvm;

// jit symbol resolving
#define EXPORT_SYMBOL(sym) __pragma(comment(linker, "/export:" sym))
EXPORT_SYMBOL("??_7type_info@@6B@")                 // type_info
EXPORT_SYMBOL("??2@YAPEAX_K@Z")                     // operator new
EXPORT_SYMBOL("??2@YAPEAX_KAEBUnothrow_t@std@@@Z")  // operator new nothrow
EXPORT_SYMBOL("??3@YAXPEAX@Z")                      // operator delete
EXPORT_SYMBOL("??3@YAXPEAX_K@Z")                    // sized operator delete
EXPORT_SYMBOL("??_U@YAPEAX_K@Z")                    // operator new[]
EXPORT_SYMBOL("??_V@YAXPEAX@Z")                     // operator delete[]
#undef EXPORT_SYMBOL

typedef int MainFunctionType();

inline string llvmErrorToString(llvm::Error& err)
{
    string s;
    llvm::raw_string_ostream ss(s);
    llvm::logAllUnhandledErrors(move(err), ss);
    return s;
}

class JITHelper
{
public:
    static string ROOT_PATH; // root path to 'rigpp' and windows/clang includes.

    JITHelper() {}

    void initJit()
    {
        llvm::ExitOnError ExitOnErr;
        jit = ExitOnErr(llvm::orc::LLJITBuilder().create());

        unique_ptr<llvm::LLVMContext> ctx = make_unique<llvm::LLVMContext>();
        context = std::make_unique<llvm::orc::ThreadSafeContext>(move(ctx));
    }

    static void initLLVM()
    {
        llvm::InitializeNativeTarget();
        llvm::InitializeNativeTargetAsmPrinter();
        llvm::InitializeNativeTargetAsmParser();
    }
    
    static void shutdownLLVM() { llvm::llvm_shutdown(); }

    // Compile cppcode to llvm::Module and add it to llvm::orc::LLJITBuilder.
    bool compileCpp(const string& cppcode, string &log, const string &outputIRPath="")
    {
        llvm::raw_string_ostream logStream(log);

        clang::IntrusiveRefCntPtr<clang::DiagnosticOptions> diagnosticOptions(new clang::DiagnosticOptions);
        auto textDiagnosticPrinter = make_unique<clang::TextDiagnosticPrinter>(logStream, diagnosticOptions.get()); // llvm::outs()

        clang::IntrusiveRefCntPtr<clang::DiagnosticIDs> diagIDs(new clang::DiagnosticIDs);
        clang::IntrusiveRefCntPtr<clang::DiagnosticsEngine> diagnosticsEngine(new clang::DiagnosticsEngine(diagIDs, diagnosticOptions, textDiagnosticPrinter.get(), false));

        vector<const char*> itemstrs = { "-ferror-limit", "2", "-fms-extensions", "-fms-compatibility", "-fms-compatibility-version=22", "-x", "c++"};

        // Clang compilation in debug mode as well. Without this you'll get crashes as std::vector in visual studio with -O0 is not the same as in clang with -O2
#ifdef _DEBUG 
        itemstrs.push_back("-O0");
#else
        itemstrs.push_back("-O2");
#endif

        unique_ptr<clang::CompilerInstance> compilerInstance = make_unique<clang::CompilerInstance>();
        auto& compilerInvocation = compilerInstance->getInvocation();
        if (!clang::CompilerInvocation::CreateFromArgs(compilerInvocation, itemstrs, *diagnosticsEngine))
            return false;

        auto* languageOptions = compilerInvocation.getLangOpts();
        auto& preprocessorOptions = compilerInvocation.getPreprocessorOpts();
        auto& targetOptions = compilerInvocation.getTargetOpts();
        auto& frontEndOptions = compilerInvocation.getFrontendOpts();
        auto& headerSearchOptions = compilerInvocation.getHeaderSearchOpts();
        auto& codeGenOptions = compilerInvocation.getCodeGenOpts();

        frontEndOptions.Inputs.clear();
        unique_ptr<llvm::MemoryBuffer> buf = llvm::MemoryBuffer::getMemBuffer(llvm::StringRef(cppcode.data(), cppcode.size()));
        frontEndOptions.Inputs.push_back(clang::FrontendInputFile(*buf, clang::InputKind(clang::Language::CXX)));

        targetOptions.Triple = LLVMGetDefaultTargetTriple();
        preprocessorOptions.addMacroDef("_DLL");
        preprocessorOptions.addMacroDef("_CRT_SECURE_NO_WARNINGS");

#ifdef _DEBUG
        preprocessorOptions.addMacroDef("_DEBUG");
#endif

        headerSearchOptions.AddPath(ROOT_PATH + "/include", clang::frontend::System, false, false);
        headerSearchOptions.AddPath(ROOT_PATH + "/include/sdk/shared", clang::frontend::System, false, false);
        headerSearchOptions.AddPath(ROOT_PATH + "/include/sdk/ucrt", clang::frontend::System, false, false);
        headerSearchOptions.AddPath(ROOT_PATH + "/include/clang", clang::frontend::Angled, false, false);
        headerSearchOptions.AddPath(ROOT_PATH + "/include/msvc", clang::frontend::CXXSystem, false, false);

        compilerInstance->createDiagnostics(textDiagnosticPrinter.get(), false);

        unique_ptr<clang::CodeGenAction> action = make_unique<clang::EmitLLVMOnlyAction>(context->getContext());

        if (!compilerInstance->ExecuteAction(*action))
            return false;

        unique_ptr<llvm::Module> module = action->takeModule();
        if (!module)
            return false;

        if (!outputIRPath.empty())
        {
            error_code error;
            llvm::ToolOutputFile outfile(outputIRPath, error, llvm::sys::fs::OF_None);
            WriteBitcodeToFile(*module, outfile.os()); // save as bitcode
            outfile.keep();
        }

        llvm::ExitOnError ExitOnErr;
        ExitOnErr(jit->addIRModule(llvm::orc::ThreadSafeModule(move(module), *context)));
        
        return true;
    }

    bool compileIR(const string& path, string &log)
    {
        llvm::SMDiagnostic error;

        unique_ptr<llvm::Module> module = parseIRFile(path, error, *context->getContext());
        if (!module)
        {
            log = error.getMessage().str();
            return false;
        }

        llvm::ExitOnError ExitOnErr;
        ExitOnErr(jit->addIRModule(llvm::orc::ThreadSafeModule(move(module), *context)));
        return true;
    }

    void addSymbols(const map<string, void*>& symbols)
    {
        auto& es = jit->getExecutionSession();
        auto& dl = jit->getDataLayout();
        auto& jd = jit->getMainJITDylib();
        
        llvm::orc::SymbolMap llvmSymbols;

        for (const auto& s : symbols)
            llvmSymbols[es.intern(s.first)] = { llvm::pointerToJITTargetAddress(s.second), llvm::JITSymbolFlags::Exported | llvm::JITSymbolFlags::Absolute };

        llvm::Error err = jd.define(llvm::orc::absoluteSymbols(llvmSymbols));
        if (err)
        {
            cerr << llvmErrorToString(err) << endl;
            return;
        }

        auto expectedProc = llvm::orc::DynamicLibrarySearchGenerator::GetForCurrentProcess(dl.getGlobalPrefix());
        err = expectedProc.takeError();
        if (err)
        {
            cerr << llvmErrorToString(err) << endl;
            return;
        }
        
        jd.addGenerator(move(expectedProc.get()));
    }

    template<typename T>
    T* lookup(const string& f)
    {
        auto expectedSym = jit->lookup(f);
        llvm::Error err = expectedSym.takeError();
        if (err)
        {
            cerr << llvmErrorToString(err) << endl;
            return nullptr;
        }

        auto* func = (T*)expectedSym.get().getAddress();
        if (!func)
            return nullptr;

        return func;
    }

private:
    unique_ptr<llvm::orc::LLJIT> jit; // these objects must exist during the node lifetime
    unique_ptr< llvm::orc::ThreadSafeContext> context;
};
