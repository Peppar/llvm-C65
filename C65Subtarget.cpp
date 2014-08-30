//===-- C65Subtarget.cpp - C65 subtarget information ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "C65Subtarget.h"
#include "MCTargetDesc/C65MCTargetDesc.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/Support/Host.h"

using namespace llvm;

#define DEBUG_TYPE "c65-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "C65GenSubtargetInfo.inc"

// Pin the vtable to this file.
void C65Subtarget::anchor() {}

static std::string computeDataLayout(const C65Subtarget &ST) {
  // Stub 2 simulates handling all 8, 16, 32, 64-bit integers natively
  return "e-p:16:8-n8:16:32:64-S8";
}

C65Subtarget &
C65Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS) {
  std::string CPUName = CPU;
  if (CPUName.empty()) {
    CPUName = "generic";
  }
  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
  return *this;
}

C65Subtarget::C65Subtarget(const std::string &TT,
                           const std::string &CPU,
                           const std::string &FS,
                           TargetMachine &TM)
  : C65GenSubtargetInfo(TT, CPU, FS),
    DL(computeDataLayout(initializeSubtargetDependencies(CPU, FS))),
    InstrInfo(*this), TLInfo(TM), TSInfo(DL), FrameLowering(*this) {}
