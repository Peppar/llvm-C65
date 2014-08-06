//===-- C65Operand.h - Parsed C65 machine instruction --------------------===//

#ifndef C65_OPERAND_H
#define C65_OPERAND_H

#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/ADT/STLExtras.h"

namespace llvm {

struct C65Operand : public MCParsedAsmOperand {
  enum KindTy {
    Immediate,
    Memory
  } Kind;

  SMLoc StartLoc, EndLoc;
  SMLoc OffsetOfLoc;
  StringRef SymName;
  void *OpDecl;
  bool AddressOf;

  struct ImmOp {
    const MCExpr *Val;
  };

  struct MemOp {
    const MCExpr *Disp;
    unsigned IndexReg;
    unsigned Size;
  };

  union {
    struct ImmOp Imm;
    struct MemOp Mem;
  };

  C65Operand(KindTy K, SMLoc Start, SMLoc End)
    : Kind(K), StartLoc(Start), EndLoc(End) {}


  StringRef getSymName() override { return SymName; }
  void *getOpDecl() override { return OpDecl; }

  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const override { return EndLoc; }
  /// getLocRange - Get the range between the first and last token of this
  /// operand.
  SMRange getLocRange() const { return SMRange(StartLoc, EndLoc); }
  /// getOffsetOfLoc - Get the location of the offset operator.
  SMLoc getOffsetOfLoc() const override { return OffsetOfLoc; }

  void print(raw_ostream &OS) const override {}

  StringRef getToken() const {
    assert(Kind == Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }
  void setTokenValue(StringRef Value) {
    assert(Kind == Token && "Invalid access!");
    Tok.Data = Value.data();
    Tok.Length = Value.size();
  }

  const MCExpr *getImm() const {
    assert(Kind == Immediate && "Invalid access!");
    return Imm.Val;
  }

  const MCExpr *getMemDisp() const {
    assert(Kind == Memory && "Invalid access!");
    return Mem.Disp;
  }
  unsigned getMemIndexReg() const {
    assert(Kind == Memory && "Invalid access!");
    return Mem.SegReg;
  }

  bool isImm() const override { return Kind == Immediate; }
  bool isMem() const override { return Kind == Memory; }

  bool isAbsMem() const {
    return Kind == Memory && !getMemIndexReg();
  }

}

}

#endif // C65_OPERAND
