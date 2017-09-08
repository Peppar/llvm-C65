//====- C65MCExpr.h - C65 specific MC expression classes --*- C++ -*-=========//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file wraps a sub-expression with a bit shift amount.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_C65_MCTARGETDESC_C65MCEXPR_H
#define LLVM_LIB_TARGET_C65_MCTARGETDESC_C65MCEXPR_H

#include "llvm/MC/MCExpr.h"

namespace llvm {

class StringRef;
class C65MCExpr : public MCTargetExpr {
private:
  const unsigned ShiftAmt;
  const MCExpr *Expr;

  explicit C65MCExpr(unsigned _ShiftAmt, const MCExpr *_Expr)
    : ShiftAmt(_ShiftAmt), Expr(_Expr) {}

public:
  static const C65MCExpr *create(unsigned ShiftAmt, const MCExpr *Expr,
                                 MCContext &Ctx);

  /// getOpcode - Get the kind of this expression.
  unsigned getShiftAmt() const { return ShiftAmt; }

  /// getSubExpr - Get the child of this expression.
  const MCExpr *getSubExpr() const { return Expr; }

  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;

  bool evaluateAsRelocatableImpl(MCValue &Res,
                                 const MCAsmLayout *Layout,
                                 const MCFixup *Fixup) const override {
    return getSubExpr()->evaluateAsRelocatable(Res, Layout, Fixup);
  }

  void visitUsedExpr(MCStreamer &Streamer) const override;

  MCFragment *findAssociatedFragment() const override {
    return getSubExpr()->findAssociatedFragment();
  }

  void fixELFSymbolsInTLSFixups(MCAssembler &Asm) const override {}

  static bool classof(const MCExpr *E) {
    return E->getKind() == MCExpr::Target;
  }

  static bool classof(const C65MCExpr *) { return true; }
};

} // end namespace llvm.

#endif
