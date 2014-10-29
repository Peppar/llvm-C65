//===-- C65MCExpr.cpp - C65 specific MC expression classes ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of assembly expressions performing
// a logical bit shift right.
//
//===----------------------------------------------------------------------===//

#include "C65MCExpr.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELF.h"
#include "llvm/MC/MCObjectStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Object/ELF.h"

using namespace llvm;

#define DEBUG_TYPE "c65-mc-expr"

const C65MCExpr*
C65MCExpr::Create(unsigned ShiftAmt, const MCExpr *Expr, MCContext &Ctx) {
  return new (Ctx) C65MCExpr(ShiftAmt, Expr);
}

void C65MCExpr::PrintImpl(raw_ostream &OS) const {
  OS << "((";
  getSubExpr()->print(OS);
  OS << ") >> " << ShiftAmt << ')';
}

void C65MCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}
