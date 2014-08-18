//===-- C65TargetInfo.cpp - 6502 compatibles Target Implementation --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "C65.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

Target llvm::The65C816Target;

extern "C" void LLVMInitializeC65TargetInfo() {
  RegisterTarget<Triple::c65, /*HasJIT=*/false>
    X(The65C816Target, "c65", "c65 [experimental]");
}
