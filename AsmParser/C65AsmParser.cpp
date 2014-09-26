//===-- C65AsmParser.cpp - Parse C65 assembly instructions --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/C65MCTargetDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCTargetAsmParser.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

// Return true if Expr is in the range [MinValue, MaxValue].
// static bool inRange(const MCExpr *Expr, int64_t MinValue, int64_t MaxValue) {
//   if (auto *CE = dyn_cast<MCConstantExpr>(Expr)) {
//     int64_t Value = CE->getValue();
//     return Value >= MinValue && Value <= MaxValue;
//   }
//   return false;
// }

namespace {

enum MemoryKind {
  MemPCRel8,
  MemPCRel16,
  MemAbs,
  MemAbsX,
  MemAbsY,
  MemAbsPreIX,
  MemAbsInd,
  MemAbsIndL,
  MemAbsL,
  MemAbsXL,
  MemZP,
  MemZPX,
  MemZPY,
  MemZPPreIX,
  MemZPInd,
  MemDPIndL,
  MemZPPostIY,
  MemDPPostIYL,
  MemSRel,
  MemSPostIY,
  MemIZ,
  MemZZ
};

class C65Operand : public MCParsedAsmOperand {
  enum OperandKind {
    KindToken,
    KindImm,
    KindMem
  } Kind;

private:
  SMLoc StartLoc, EndLoc;

  struct Token {
    const char *Data;
    unsigned Length;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  struct MemOp {
    const MCExpr *Addr;
    unsigned Kind;
  };

  union {
    struct Token Tok;
    struct ImmOp Imm;
    struct MemOp Mem;
  };

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediates when possible.  Null MCExpr = 0.
    if (!Expr)
      Inst.addOperand(MCOperand::CreateImm(0));
    else if (auto *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::CreateImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::CreateExpr(Expr));
  }

public:
  C65Operand(OperandKind kind, SMLoc startLoc, SMLoc endLoc)
    : MCParsedAsmOperand(), Kind(kind), StartLoc(startLoc), EndLoc(endLoc) {}

  // Create particular kinds of operand.
  static std::unique_ptr<C65Operand> createToken(StringRef Str, SMLoc Loc) {
    auto Op = make_unique<C65Operand>(KindToken, Loc, Loc);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    return Op;
  }
  static std::unique_ptr<C65Operand>
  createImm(const MCExpr *Expr, SMLoc StartLoc, SMLoc EndLoc) {
    auto Op = make_unique<C65Operand>(KindImm, StartLoc, EndLoc);
    Op->Imm.Val = Expr;
    return Op;
  }
  static std::unique_ptr<C65Operand>
  createMem(const MCExpr *Addr, unsigned Kind, SMLoc StartLoc, SMLoc EndLoc) {
    auto Op = make_unique<C65Operand>(KindMem, StartLoc, EndLoc);
    Op->Mem.Kind = Kind;
    Op->Mem.Addr = Addr;
    return Op;
  }

  // Token operands
  bool isToken() const override {
    return Kind == KindToken;
  }
  StringRef getToken() const {
    assert(Kind == KindToken && "Not a token");
    return StringRef(Tok.Data, Tok.Length);
  }

  // Register operands.
  bool isReg() const override {
    return false;
  }
  unsigned getReg() const override {
    llvm_unreachable("C65 has only implicit registers.");
  }

  // Immediate operands.
  bool isImm() const override {
    return Kind == KindImm;
  }
  //  bool isImm(int64_t MinValue, int64_t MaxValue) const {
  //    return Kind == KindImm && inRange(Imm, MinValue, MaxValue);
  //  }
  const MCExpr *getImm() const {
    assert(Kind == KindImm && "Not an immediate");
    return Imm.Val;
  }

  // Memory operands.
  bool isMem() const override {
    return Kind == KindMem;
  }
  bool isMem(MemoryKind MemKind) const {
    return Kind == KindMem && Mem.Kind == MemKind;
  }
  const MCExpr *getMemAddr() const {
    assert(Kind == KindMem && "Not an address");
    return Mem.Addr;
  }

  // Override MCParsedAsmOperand.
  SMLoc getStartLoc() const override { return StartLoc; }
  SMLoc getEndLoc() const override { return EndLoc; }
  void print(raw_ostream &OS) const override;

  // Used by the TableGen code to add particular types of operand
  // to an instruction.
  void addRegOperands(MCInst &Inst, unsigned N) const {
    llvm_unreachable("C65 has only implicit registers.");
  }
  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands");
    addExpr(Inst, getImm());
  }
  void addMemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands");
    addExpr(Inst, getMemAddr());
  }

  // Used by the TableGen code to check for particular operand types.
  bool isasmop_pcrel8() { return isMem(MemPCRel8); }
  bool isasmop_pcrel16() { return isMem(MemPCRel16); }
  bool isasmop_abs() { return isMem(MemAbs); }
  bool isasmop_absx() { return isMem(MemAbsX); }
  bool isasmop_absy() { return isMem(MemAbsY); }
  bool isasmop_abspreix() { return isMem(MemAbsPreIX); }
  bool isasmop_absind() { return isMem(MemAbsInd); }
  bool isasmop_absindl() { return isMem(MemAbsIndL); }
  bool isasmop_absl() { return isMem(MemAbsL); }
  bool isasmop_absxl() { return isMem(MemAbsXL); }
  bool isasmop_zp() { return isMem(MemZP); }
  bool isasmop_zpx() { return isMem(MemZPX); }
  bool isasmop_zpy() { return isMem(MemZPY); }
  bool isasmop_zppreix() { return isMem(MemZPPreIX); }
  bool isasmop_zpind() { return isMem(MemZPInd); }
  bool isasmop_dpindl() { return isMem(MemDPIndL); }
  bool isasmop_zppostiy() { return isMem(MemZPPostIY); }
  bool isasmop_dppostiyl() { return isMem(MemDPPostIYL); }
  bool isasmop_srel() { return isMem(MemSRel); }
  bool isasmop_spostiy() { return isMem(MemSPostIY); }
};

class C65AsmParser : public MCTargetAsmParser {
#define GET_ASSEMBLER_HEADER
#include "C65GenAsmMatcher.inc"

private:
  MCSubtargetInfo &STI;
  MCAsmParser &Parser;

  //  bool parseAddress(unsigned &Base, const MCExpr *&Disp,
  //                    unsigned &Index, const MCExpr *&Length,
  //                    const unsigned *Regs, RegisterKind RegKind);

  //  OperandMatchResultTy parseAddress(OperandVector &Operands,
  //                                    const unsigned *Regs, RegisterKind RegKind,
  //                                    MemoryKind MemKind);
  bool parseOperand(OperandVector &Operands, StringRef Mnemonic);

public:
  C65AsmParser(MCSubtargetInfo &sti, MCAsmParser &parser,
                   const MCInstrInfo &MII,
                   const MCTargetOptions &Options)
      : MCTargetAsmParser(), STI(sti), Parser(parser) {
    MCAsmParserExtension::Initialize(Parser);

    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }

  bool ParseDirective(AsmToken DirectiveID) override;
  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;
  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;
  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  OperandMatchResultTy
  parseAddress(OperandVector &Operands, MemoryKind MemKind,
               unsigned Length, bool AllowZExt, unsigned Indirection,
               char PreIndexReg, char PostIndexReg);

  bool matchRegister(char Reg);

  bool parseAddress(const MCExpr *&Addr, unsigned Length,
                    bool AllowZExt, unsigned Indirection,
                    char PreIndexReg, char PostIndexReg);

  // Used by the TableGen code to parse particular operand types.
  OperandMatchResultTy parsePCRel8Operand(OperandVector &Operands) {
    return parseAddress(Operands, MemPCRel8, 8, true, 0, 0, 0);
  }
  OperandMatchResultTy parsePCRel16Operand(OperandVector &Operands) {
    return parseAddress(Operands, MemPCRel16, 16, false, 0, 0, 0);
  }
  OperandMatchResultTy parseAbsOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbs, 16, false, 0, 0, 0);
  }
  OperandMatchResultTy parseAbsXOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbsX, 16, false, 0, 'X', 0);
  }
  OperandMatchResultTy parseAbsYOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbsY, 16, false, 0, 'Y', 0);
  }
  OperandMatchResultTy parseAbsPreIXOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbsPreIX, 16, false, 1, 'X', 0);
  }
  OperandMatchResultTy parseAbsIndOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbsInd, 16, false, 1, 0, 0);
  }
  OperandMatchResultTy parseAbsIndLOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbsIndL, 16, false, 2, 0, 0);
  }
  OperandMatchResultTy parseAbsLOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbsL, 24, false, 0, 0, 0);
  }
  OperandMatchResultTy parseAbsXLOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemAbsXL, 24, false, 0, 'X', 0);
  }
  OperandMatchResultTy parseZPOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemZP, 8, true, 0, 0, 0);
  }
  OperandMatchResultTy parseZPXOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemZPX, 8, true, 0, 'X', 0);
  }
  OperandMatchResultTy parseZPYOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemZPY, 8, true, 0, 'Y', 0);
  }
  OperandMatchResultTy parseZPPreIXOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemZPPreIX, 8, true, 1, 'X', 0);
  }
  OperandMatchResultTy parseZPIndOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemZPInd, 8, true, 1, 0, 0);
  }
  OperandMatchResultTy parseDPIndLOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemDPIndL, 8, true, 2, 0, 0);
  }
  OperandMatchResultTy parseZPPostIYOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemZPPostIY, 8, true, 1, 0, 'Y');
  }
  OperandMatchResultTy parseDPPostIYLOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemDPPostIYL, 8, true, 2, 0, 'Y');
  }
  OperandMatchResultTy parseSRelOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemSRel, 8, true, 0, 'S', 0);
  }
  OperandMatchResultTy parseSPostIYOperand(OperandVector &Operands) {
    return parseAddress(Operands, MemSPostIY, 8, true, 1, 'S', 'Y');
  }
  OperandMatchResultTy parseIZOperand(OperandVector &Operands) {
    llvm_unreachable("Zero-page register operands should have been expanded.");
  }
  OperandMatchResultTy parseZZOperand(OperandVector &Operands) {
    llvm_unreachable("Zero-page register operands should have been expanded.");
  }

  bool isAcc8Bit() const {
    return (STI.getFeatureBits() & C65::ModeAcc8Bit) != 0;
  }
  bool isAcc16Bit() const {
    return (STI.getFeatureBits() & C65::ModeAcc16Bit) != 0;
  }
  bool isIx8Bit() const {
    return (STI.getFeatureBits() & C65::ModeIx8Bit) != 0;
  }
  bool isIx16Bit() const {
    return (STI.getFeatureBits() & C65::ModeIx16Bit) != 0;
  }
  void SwitchAccMode(uint64_t mode) {
    uint64_t oldMode = STI.getFeatureBits() &
      (C65::ModeAcc8Bit | C65::ModeAcc16Bit);
    unsigned FB = ComputeAvailableFeatures(STI.ToggleFeature(oldMode | mode));
    setAvailableFeatures(FB);
    assert(mode == (STI.getFeatureBits() &
                    (C65::ModeAcc8Bit | C65::ModeAcc16Bit)));
  }
  void SwitchIxMode(uint64_t mode) {
    uint64_t oldMode = STI.getFeatureBits() &
      (C65::ModeIx8Bit | C65::ModeIx16Bit);
    unsigned FB = ComputeAvailableFeatures(STI.ToggleFeature(oldMode | mode));
    setAvailableFeatures(FB);
    assert(mode == (STI.getFeatureBits() &
                    (C65::ModeIx8Bit | C65::ModeIx16Bit)));
  }
};
} // end anonymous namespace

#define GET_REGISTER_MATCHER
#define GET_SUBTARGET_FEATURE_NAME
#define GET_MATCHER_IMPLEMENTATION
#include "C65GenAsmMatcher.inc"

void C65Operand::print(raw_ostream &OS) const {
  llvm_unreachable("Not implemented");
}

bool C65AsmParser::matchRegister(char Reg) {
  // Expect a register name.
  if (Parser.getTok().isNot(AsmToken::Identifier))
    return true;

  // Check that there's a prefix.
  StringRef Name = Parser.getTok().getString();
  if (Name.size() != 1 || Name[0] != Reg)
    return true;

  Parser.Lex();
  return false;
}

// Match an address with the specified constraints, indirection and
// index registers. An indirection of 1 implies 16-bit indirection,
// indicated with parentheses, while an indirection of 2 implies a
// 24-bit indirection, indicated with brackets.
//
bool C65AsmParser::parseAddress(const MCExpr *&Addr, unsigned Length,
                                bool AllowZExt, unsigned Indirection,
                                char PreIndexReg, char PostIndexReg) {
  unsigned LengthConstraint;

  // Opening parenthesis or bracket
  if (Indirection == 1) {
    if (!getLexer().is(AsmToken::LParen))
      return true;
  } else if (Indirection == 2) {
    if (!getLexer().is(AsmToken::LBrac))
      return true;
  }
  // Length contraint token
  if (getLexer().is(AsmToken::Less)) {
    LengthConstraint = 8;
    Parser.Lex();
  } else if (getLexer().is(AsmToken::Exclaim)) {
    LengthConstraint = 16;
    Parser.Lex();
  } else if (getLexer().is(AsmToken::Greater)) {
    LengthConstraint = 24;
    Parser.Lex();
  }
  // Parse the address
  if (getParser().parseExpression(Addr))
    return true;
  int64_t Address;
  if (LengthConstraint && LengthConstraint != Length) {
    return true;
  } else if (Addr->EvaluateAsAbsolute(Address) && Address >= 0) {
    if (Address >= (1 << Length))
      return true;
    if (!AllowZExt && !(Address >> (Length - 8)))
      return true;
  }
  // Pre-index register
  if (PreIndexReg) {
    if (!getLexer().is(AsmToken::Comma))
      return true;
    if (matchRegister(PreIndexReg))
      return true;
  }
  // Closing parenthesis or bracket
  if (Indirection == 1) {
    if (!getLexer().is(AsmToken::RParen))
      return true;
  } else if (Indirection == 2) {
    if (!getLexer().is(AsmToken::RBrac))
      return true;
  }
  // Post-index register
  if (PostIndexReg) {
    if (!getLexer().is(AsmToken::Comma))
      return true;
    if (matchRegister(PostIndexReg))
      return true;
  }
  return false;
}

// Parse a memory operand and add it to Operands.  The other arguments
// are as above.
C65AsmParser::OperandMatchResultTy
C65AsmParser::parseAddress(OperandVector &Operands, MemoryKind MemKind,
                           unsigned Length,
                           bool AllowZExt, unsigned Indirection,
                           char PreIndexReg, char PostIndexReg) {
  SMLoc StartLoc = Parser.getTok().getLoc();
  const MCExpr *Addr;
  if (parseAddress(Addr, Length, AllowZExt,
                   Indirection, PreIndexReg, PostIndexReg))
    return MatchOperand_ParseFail;

  SMLoc EndLoc =
    SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(C65Operand::createMem(Addr, MemKind, StartLoc, EndLoc));
  return MatchOperand_Success;
}

bool C65AsmParser::ParseDirective(AsmToken DirectiveID) {
  return true;
}

bool C65AsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                 SMLoc &EndLoc) {
  return true;
}

bool C65AsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                    StringRef Name, SMLoc NameLoc,
                                    OperandVector &Operands) {
  Operands.push_back(C65Operand::createToken(Name, NameLoc));

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (parseOperand(Operands, Name)) {
      Parser.eatToEndOfStatement();
      return true;
    }

    // Read any subsequent operands.
    while (getLexer().is(AsmToken::Comma)) {
      Parser.Lex();
      if (parseOperand(Operands, Name)) {
        Parser.eatToEndOfStatement();
        return true;
      }
    }
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }
  }

  // Consume the EndOfStatement.
  Parser.Lex();
  return false;
}

bool C65AsmParser::parseOperand(OperandVector &Operands,
                                StringRef Mnemonic) {
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);
  if (ResTy == MatchOperand_Success)
    return false;
  return true;
  // if (ResTy == MatchOperand_ParseFail)
  //   return true;

  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.

  // C65 doesn't have explicit registers; they are always part of
  // addressing modes.
  // if (Parser.getTok().is(AsmToken::Percent))
  //   return true;

  // The only other type of operand is an immediate or address.  As above,
  // real address operands should have used a context-dependent parse routine,
  // so we treat any plain expression as an immediate.
  //  SMLoc StartLoc = Parser.getTok().getLoc();
  //  unsigned Base, Index;
  //  const MCExpr *Expr, *Length;
  //  if (parseAddress(Base, Expr, Index, Length, C65MC::GR64Regs, ADDR64Reg))
  //    return true;

  // SMLoc EndLoc =
  //   SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  // if (Base || Index || Length)
  //   Operands.push_back(C65Operand::createInvalid(StartLoc, EndLoc));
  // else
  //   Operands.push_back(C65Operand::createImm(Expr, StartLoc, EndLoc));
  // return false;
}

bool C65AsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                           OperandVector &Operands,
                                           MCStreamer &Out,
                                           uint64_t &ErrorInfo,
                                           bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult;

  MatchResult = MatchInstructionImpl(Operands, Inst, ErrorInfo,
                                     MatchingInlineAsm);
  switch (MatchResult) {
  default: break;
  case Match_Success:
    Inst.setLoc(IDLoc);
    Out.EmitInstruction(Inst, STI);
    return false;

  case Match_MissingFeature: {
    assert(ErrorInfo && "Unknown missing feature!");
    // Special case the error message for the very common case where only
    // a single subtarget feature is missing
    std::string Msg = "instruction requires:";
    uint64_t Mask = 1;
    for (unsigned I = 0; I < sizeof(ErrorInfo) * 8 - 1; ++I) {
      if (ErrorInfo & Mask) {
        Msg += " ";
        Msg += getSubtargetFeatureName(ErrorInfo & Mask);
      }
      Mask <<= 1;
    }
    return Error(IDLoc, Msg);
  }

  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0ULL) {
      if (ErrorInfo >= Operands.size())
        return Error(IDLoc, "too few operands for instruction");

      ErrorLoc = ((C65Operand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }

  case Match_MnemonicFail:
    return Error(IDLoc, "invalid instruction");
  }

  llvm_unreachable("Unexpected match type");
}

// C65AsmParser::OperandMatchResultTy
// C65AsmParser::parsePCRel(OperandVector &Operands, int64_t MinVal,
//                              int64_t MaxVal) {
//   MCContext &Ctx = getContext();
//   MCStreamer &Out = getStreamer();
//   const MCExpr *Expr;
//   SMLoc StartLoc = Parser.getTok().getLoc();
//   if (getParser().parseExpression(Expr))
//     return MatchOperand_NoMatch;

//   // For consistency with the GNU assembler, treat immediates as offsets
//   // from ".".
//   if (auto *CE = dyn_cast<MCConstantExpr>(Expr)) {
//     int64_t Value = CE->getValue();
//     if ((Value & 1) || Value < MinVal || Value > MaxVal) {
//       Error(StartLoc, "offset out of range");
//       return MatchOperand_ParseFail;
//     }
//     MCSymbol *Sym = Ctx.CreateTempSymbol();
//     Out.EmitLabel(Sym);
//     const MCExpr *Base = MCSymbolRefExpr::Create(Sym, MCSymbolRefExpr::VK_None,
//                                                  Ctx);
//     Expr = Value == 0 ? Base : MCBinaryExpr::CreateAdd(Base, Expr, Ctx);
//   }

//   SMLoc EndLoc =
//     SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
//   Operands.push_back(C65Operand::createImm(Expr, StartLoc, EndLoc));
//   return MatchOperand_Success;
// }

// Force static initialization.
extern "C" void LLVMInitializeC65AsmParser() {
  RegisterMCAsmParser<C65AsmParser> X(The65C816Target);
}
