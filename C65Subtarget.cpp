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

void
C65Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS) {
  StringRef CPUName = CPU;
  if (CPUName.empty() || CPUName == "generic") {
    CPUName = "65816";
  }
  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);

  // It's important to keep the MCSubtargetInfo feature bits in sync with
  // target data structure which is shared with MC code emitter, etc.
  if (InAcc8Mode)
    ToggleFeature(C65::ModeAcc8Bit);
  if (InAcc16Mode)
    ToggleFeature(C65::ModeAcc16Bit);
  if (InIx8Mode)
    ToggleFeature(C65::ModeIx8Bit);
  if (InIx16Mode)
    ToggleFeature(C65::ModeIx16Bit);
}

C65Subtarget::C65Subtarget(const Triple &TT, StringRef CPU, StringRef FS,
                           TargetMachine &TM)
  : C65GenSubtargetInfo(TT, CPU, FS),
    InstrInfo(*this),
    TLInfo(TM, *this), TSInfo(), FrameLowering() {
  initializeSubtargetDependencies(CPU, FS);
}
