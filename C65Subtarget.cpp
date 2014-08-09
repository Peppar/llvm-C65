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

C65Subtarget::C65Subtarget(const std::string &TT,
                           const std::string &CPU,
                           const std::string &FS)
  : C65GenSubtargetInfo(TT, CPU, FS),
    Has65C02(false), Has65C816(false),
    TargetTriple(TT) {
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "generic";

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
}
