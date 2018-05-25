//===-- C65MCObjectWriter.cpp - WLAK object target writer -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/MC/MCWLAKObjectWriter.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/SourceMgr.h"

#include "C65MCTargetDesc.h"

using namespace llvm;

std::unique_ptr<MCObjectTargetWriter>
llvm::createC65WLAKObjectTargetWriter(uint8_t OSABI) {
  return llvm::make_unique<MCWLAKObjectTargetWriter>();
}
