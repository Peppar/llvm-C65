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

static const char *DescriptionString6502 =
  "e-p:16:8-i16:8-i32:8-i64:8-n8:16:32:64-S8";
static const char *DescriptionString65816 =
  "e-p:16:8-p1:32:8-i16:8-i32:8-i64:8-n8:16:32:64-S8";

// Pin the vtable to this file.
void C65Subtarget::anchor() {}

static std::string computeDataLayout(const C65Subtarget &ST) {
  if (ST.has65816())
    return DescriptionString65816;
  else
    return DescriptionString6502;
}

C65Subtarget &
C65Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS) {
  std::string CPUName = CPU;
  if (CPUName.empty()) {
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

  return *this;
}

C65Subtarget::C65Subtarget(const std::string &TT,
                           const std::string &CPU,
                           const std::string &FS,
                           TargetMachine &TM)
  : C65GenSubtargetInfo(TT, CPU, FS),
    DL(computeDataLayout(initializeSubtargetDependencies(CPU, FS))),
    InstrInfo(*this), TLInfo(TM), TSInfo(DL), FrameLowering(*this) {}
