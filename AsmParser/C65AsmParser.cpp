//===-- C65AsmParser.cpp - Parse C65 assembly instructions ----------------===//
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
      Inst.addOperand(MCOperand::createImm(0));
    else if (auto *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
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
  bool isasmop_imm()       { return isImm(); }
  bool isasmop_pcrel8()    { return isMem(MemPCRel8   ); }
  bool isasmop_pcrel16()   { return isMem(MemPCRel16  ); }
  bool isasmop_abs()       { return isMem(MemAbs      ); }
  bool isasmop_absx()      { return isMem(MemAbsX     ); }
  bool isasmop_absy()      { return isMem(MemAbsY     ); }
  bool isasmop_abspreix()  { return isMem(MemAbsPreIX ); }
  bool isasmop_absind()    { return isMem(MemAbsInd   ); }
  bool isasmop_absindl()   { return isMem(MemAbsIndL  ); }
  bool isasmop_absl()      { return isMem(MemAbsL     ); }
  bool isasmop_absxl()     { return isMem(MemAbsXL    ); }
  bool isasmop_zp()        { return isMem(MemZP       ); }
  bool isasmop_zpx()       { return isMem(MemZPX      ); }
  bool isasmop_zpy()       { return isMem(MemZPY      ); }
  bool isasmop_zppreix()   { return isMem(MemZPPreIX  ); }
  bool isasmop_zpind()     { return isMem(MemZPInd    ); }
  bool isasmop_dpindl()    { return isMem(MemDPIndL   ); }
  bool isasmop_zppostiy()  { return isMem(MemZPPostIY ); }
  bool isasmop_dppostiyl() { return isMem(MemDPPostIYL); }
  bool isasmop_srel()      { return isMem(MemSRel     ); }
  bool isasmop_spostiy()   { return isMem(MemSPostIY  ); }
};

class C65AsmParser : public MCTargetAsmParser {
#define GET_ASSEMBLER_HEADER
#include "C65GenAsmMatcher.inc"

private:
  MCAsmParser &Parser;

  // This structure is used to parse all possible addresses once,
  // since they are impossible to distinguish without either a
  // backtracking lexer or extended peeking capabilities.
  struct C65AddressMatch {
    SMLoc StartLoc, EndLoc;
    bool Valid, Lexed, Match;
    const MCExpr *Addr;
    unsigned Indirection;
    unsigned LengthConstraint;
    char PreIndexReg;
    char PostIndexReg;
  } AddressMatch;

  // This function is used to parse an address structure.
  OperandMatchResultTy
  parseAddress();

  OperandMatchResultTy
  parseAddress(OperandVector &Operands, MemoryKind MemKind,
               unsigned Length, bool AllowZExt, unsigned Indirection,
               char PreIndexReg, char PostIndexReg);

  bool matchRegister(char Reg);

  bool parseRegister(char &Reg);

  bool parseOperand(OperandVector &Operands, StringRef Mnemonic);

public:
  C65AsmParser(const MCSubtargetInfo &sti, MCAsmParser &parser,
               const MCInstrInfo &MII,
               const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, sti), Parser(parser) {
    MCAsmParserExtension::Initialize(Parser);

    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));
  }

  bool ParseDirective(AsmToken DirectiveID) override;
  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;
  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;
  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  // Used by the TableGen code to parse particular operand types.
  OperandMatchResultTy parseImmOperand(OperandVector &Operands) {
    SMLoc StartLoc = Parser.getTok().getLoc();
    const MCExpr *Imm;
    // Hash required for immediate operands
    if (!getLexer().is(AsmToken::Hash))
      return MatchOperand_NoMatch;
    Parser.Lex();
    if (getLexer().is(AsmToken::Less)) {
      // Get lower byte of operand (no-op)
      Parser.Lex();
    } else if (getLexer().is(AsmToken::Greater)) {
      // Get higher byte of operand (shift right 8 bits)
      Parser.Lex();
      const MCExpr *ShiftAmt = MCConstantExpr::create(8, Parser.getContext());
      Imm = MCBinaryExpr::createLShr(Imm, ShiftAmt, Parser.getContext());
    } else if (getLexer().is(AsmToken::Colon)) {
      // Get bank byte of operand (shift right 16 bits)
      Parser.Lex();
      const MCExpr *ShiftAmt = MCConstantExpr::create(16, Parser.getContext());
      Imm = MCBinaryExpr::createLShr(Imm, ShiftAmt, Parser.getContext());
    }
    if (getParser().parseExpression(Imm))
      return MatchOperand_ParseFail;
    SMLoc EndLoc =
      SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

    Operands.push_back(C65Operand::createImm(Imm, StartLoc, EndLoc));
    return MatchOperand_Success;
  }
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
    return getSTI().getFeatureBits()[C65::ModeAcc8Bit];
  }
  bool isAcc16Bit() const {
    return getSTI().getFeatureBits()[C65::ModeAcc16Bit];
  }
  bool isIx8Bit() const {
    return getSTI().getFeatureBits()[C65::ModeIx8Bit];
  }
  bool isIx16Bit() const {
    return getSTI().getFeatureBits()[C65::ModeIx16Bit];
  }
  void SwitchAccMode(unsigned mode) {
    // uint64_t oldMode = getSTI().getFeatureBits() &
    //   (C65::ModeAcc8Bit | C65::ModeAcc16Bit);
    // unsigned FB = ComputeAvailableFeatures(getSTI().ToggleFeature(oldMode | mode));
    // setAvailableFeatures(FB);
    // assert(mode == (getSTI().getFeatureBits() &
    //                 (C65::ModeAcc8Bit | C65::ModeAcc16Bit)));
    // MCSubtargetInfo &getSTI() = copySTI();
    MCSubtargetInfo &STI = copySTI();
    FeatureBitset AllModes({C65::ModeAcc8Bit, C65::ModeAcc16Bit});
    FeatureBitset OldMode = STI.getFeatureBits() & AllModes;
    unsigned FB = ComputeAvailableFeatures(
      STI.ToggleFeature(OldMode.flip(mode)));
    setAvailableFeatures(FB);
    assert(FeatureBitset({mode}) == (STI.getFeatureBits() & AllModes));
  }
  void SwitchIxMode(unsigned mode) {
    // uint64_t oldMode = STI.getFeatureBits() &
    //   (C65::ModeIx8Bit | C65::ModeIx16Bit);
    // unsigned FB = ComputeAvailableFeatures(STI.ToggleFeature(oldMode | mode));
    // setAvailableFeatures(FB);
    // assert(mode == (STI.getFeatureBits() &
    //                 (C65::ModeIx8Bit | C65::ModeIx16Bit)));
    MCSubtargetInfo &STI = copySTI();
    FeatureBitset AllModes({C65::ModeIx8Bit, C65::ModeIx16Bit});
    FeatureBitset OldMode = STI.getFeatureBits() & AllModes;
    unsigned FB = ComputeAvailableFeatures(
      STI.ToggleFeature(OldMode.flip(mode)));
    setAvailableFeatures(FB);
    assert(FeatureBitset({mode}) == (STI.getFeatureBits() & AllModes));
  }
};
} // end anonymous namespace

#define GET_REGISTER_MATCHER
#define GET_SUBTARGET_FEATURE_NAME
#define GET_MATCHER_IMPLEMENTATION
#include "C65GenAsmMatcher.inc"

void C65Operand::print(raw_ostream &OS) const {
  if (isMem()) {
    switch(Mem.Kind) {
    case MemPCRel8:    OS << "PCRel8";    break;
    case MemPCRel16:   OS << "PCRel16";   break;
    case MemAbs:       OS << "Abs";       break;
    case MemAbsX:      OS << "AbsX";      break;
    case MemAbsY:      OS << "AbsY";      break;
    case MemAbsPreIX:  OS << "AbsPreIX";  break;
    case MemAbsInd:    OS << "AbsInd";    break;
    case MemAbsIndL:   OS << "AbsIndL";   break;
    case MemAbsL:      OS << "AbsL";      break;
    case MemAbsXL:     OS << "AbsXL";     break;
    case MemZP:        OS << "ZP";        break;
    case MemZPX:       OS << "ZPX";       break;
    case MemZPY:       OS << "ZPY";       break;
    case MemZPPreIX:   OS << "ZPPreIX";   break;
    case MemZPInd:     OS << "ZPInd";     break;
    case MemDPIndL:    OS << "DPIndL";    break;
    case MemZPPostIY:  OS << "ZPPostIY";  break;
    case MemDPPostIYL: OS << "DPPostIYL"; break;
    case MemSRel:      OS << "SRel";      break;
    case MemSPostIY:   OS << "SPostIY";   break;
    case MemIZ:        OS << "IZ";        break;
    case MemZZ:        OS << "ZZ";        break;
    default: llvm_unreachable("Unexpected memory kind.");
    }
    OS << "(" << getMemAddr() << ")";
  } else if (isImm()) {
    OS << '#' << getImm();
  } else if (isToken()) {
    OS << getToken();
  } else {
    llvm_unreachable("Unexpected operand type.");
  }
}

// Match a specific register for an adressing mode. Only X, Y and S
// are supported.
//
bool C65AsmParser::parseRegister(char &Reg) {
  if (Parser.getTok().isNot(AsmToken::Identifier)) {
    return Error(Parser.getTok().getLoc(), "expected identifier");
  }
  Reg = StringSwitch<char>(Parser.getTok().getString())
    .Cases("x", "X", 'X')
    .Cases("y", "Y", 'Y')
    .Cases("s", "S", 'S')
    .Default(0);
  if (!Reg) {
    return Error(Parser.getTok().getLoc(), "expected indexing register name");
  }
  Parser.Lex();
  return false;
}

// Match an address with the specified constraints, indirection and
// index registers. An indirection of 1 implies 16-bit indirection,
// indicated with parentheses, while an indirection of 2 implies a
// 24-bit indirection, indicated with brackets.
//
C65AsmParser::OperandMatchResultTy
C65AsmParser::parseAddress() {
  if (AddressMatch.Valid) {
    if (AddressMatch.Match)
      return MatchOperand_Success;
    else
      return MatchOperand_NoMatch;
  }
  AddressMatch.StartLoc = Parser.getTok().getLoc();
  AddressMatch.Valid = true;
  AddressMatch.Lexed = false;
  AddressMatch.Match = false;

  // Hash indicates immediate constant
  if (getLexer().is(AsmToken::Hash))
    return MatchOperand_NoMatch;
  // Opening parenthesis or bracket
  if (getLexer().is(AsmToken::LParen)) {
    AddressMatch.Indirection = 1;
    Parser.Lex();
  } else if (getLexer().is(AsmToken::LBrac)) {
    AddressMatch.Indirection = 2;
    Parser.Lex();
  } else {
    AddressMatch.Indirection = 0;
  }
  // Length contraint token
  if (getLexer().is(AsmToken::Less)) {
    AddressMatch.LengthConstraint = 8;
    Parser.Lex();
  } else if (getLexer().is(AsmToken::Exclaim)) {
    AddressMatch.LengthConstraint = 16;
    Parser.Lex();
  } else if (getLexer().is(AsmToken::Greater)) {
    AddressMatch.LengthConstraint = 24;
    Parser.Lex();
  } else {
    AddressMatch.LengthConstraint = 0;
  }
  AddressMatch.Lexed = true;
  // Parse the address
  if (getParser().parseExpression(AddressMatch.Addr))
    return MatchOperand_ParseFail;
  // Pre-index register
  if (getLexer().is(AsmToken::Comma)) {
    Parser.Lex();
    if (parseRegister(AddressMatch.PreIndexReg))
      return MatchOperand_ParseFail;
  } else {
    AddressMatch.PreIndexReg = 0;
  }
  // Closing parenthesis or bracket
  if (AddressMatch.Indirection == 1) {
    if (!getLexer().is(AsmToken::RParen)) {
      Error(Parser.getTok().getLoc(), "expected closing parenthesis ')'");
      return MatchOperand_ParseFail;
    }
    Parser.Lex();
  } else if (AddressMatch.Indirection == 2) {
    if (!getLexer().is(AsmToken::RBrac)) {
      Error(Parser.getTok().getLoc(), "expected closing bracket ']'");
      return MatchOperand_ParseFail;
    }
    Parser.Lex();
  }
  // Post-index register
  if (getLexer().is(AsmToken::Comma)) {
    Parser.Lex();
    if (parseRegister(AddressMatch.PostIndexReg))
      return MatchOperand_ParseFail;
  } else {
    AddressMatch.PostIndexReg = 0;
  }
  AddressMatch.EndLoc =
    SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  AddressMatch.Match = true;
  return MatchOperand_Success;
}

// Parse a memory operand and add it to Operands.
//
C65AsmParser::OperandMatchResultTy
C65AsmParser::parseAddress(OperandVector &Operands, MemoryKind MemKind,
                           unsigned Length, bool AllowZExt,
                           unsigned Indirection, char PreIndexReg,
                           char PostIndexReg) {
  OperandMatchResultTy Result;

  Result = parseAddress();
  if (Result == MatchOperand_ParseFail ||
      Result == MatchOperand_NoMatch)
    return Result;

  if (Indirection != AddressMatch.Indirection ||
      PreIndexReg != AddressMatch.PreIndexReg ||
      PostIndexReg != AddressMatch.PostIndexReg)
    return MatchOperand_NoMatch;

  int64_t Address;
  if (AddressMatch.LengthConstraint) {
    if (AddressMatch.LengthConstraint != Length)
      return MatchOperand_NoMatch;
  } else if (AddressMatch.Addr->evaluateAsAbsolute(Address) && Address >= 0) {
    if (Address >= (1 << Length))
      return MatchOperand_NoMatch;
    if (!AllowZExt && !(Address >> (Length - 8)))
      return MatchOperand_NoMatch;
  }

  Operands.push_back(C65Operand::createMem(AddressMatch.Addr, MemKind,
                                           AddressMatch.StartLoc,
                                           AddressMatch.EndLoc));
  return MatchOperand_Success;
}

// Parse target-specific directives.
//
bool C65AsmParser::ParseDirective(AsmToken DirectiveID) {
  StringRef IDVal = DirectiveID.getIdentifier();
  if (IDVal == ".accu") {
    StringRef Size = Parser.getTok().getString();
    if (Size == "8") {
      Parser.Lex();
      if (!isAcc8Bit())
        SwitchAccMode(C65::ModeAcc8Bit);
      return false;
    } else if (Size == "16") {
      Parser.Lex();
      if (!isAcc16Bit())
        SwitchAccMode(C65::ModeAcc16Bit);
      return false;
    } else {
      return Error(Parser.getTok().getLoc(), "expected 8 or 16");
    }
  } else if (IDVal == ".index") {
    StringRef Size = Parser.getTok().getString();
    if (Size == "8") {
      Parser.Lex();
      if (!isIx8Bit())
        SwitchIxMode(C65::ModeIx8Bit);
      return false;
    } else if (Size == "16") {
      Parser.Lex();
      if (!isIx16Bit())
        SwitchIxMode(C65::ModeIx16Bit);
      return false;
    } else {
      return Error(Parser.getTok().getLoc(), "expected 8 or 16");
    }
  }
  return true;
}

// Registers are always implicit or distinguish an adressing
// mode. This function never matches.
//
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
  // New operand; force a new address to be parsed if requested.
  AddressMatch.Valid = false;

  // Check if the current operand has a custom associated parser. The
  // custom associated parsers used by C65 are all addresses; the
  // first one called will fill out the AddressMatch structure.
  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);
  if (ResTy == MatchOperand_Success)
    return false;

  // If there was a parse fail, we need to exit now, since we have
  // already supplied an appropriate error.
  if (ResTy == MatchOperand_ParseFail)
    return true;

  // If we consumed tokens to parse an address, and none of the custom
  // parser matched it, then we need to flag this as a match
  // fail.
  if (AddressMatch.Valid && AddressMatch.Lexed) {
    return Error(AddressMatch.StartLoc,
                 "instruction does not support this addressing mode");
  }
  return true;
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
    Out.EmitInstruction(Inst, getSTI());
    // Accumulator and index register size selection directives for
    // 65802 and 65816
    if (Inst.getOpcode() == C65::LONGA_ON) {
      SwitchAccMode(C65::ModeAcc16Bit);
    } else if (Inst.getOpcode() == C65::LONGA_OFF) {
      SwitchAccMode(C65::ModeAcc8Bit);
    } else if (Inst.getOpcode() == C65::LONGI_ON) {
      SwitchIxMode(C65::ModeIx16Bit);
    } else if (Inst.getOpcode() == C65::LONGI_OFF) {
      SwitchIxMode(C65::ModeIx8Bit);
    }
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

// Force static initialization.
extern "C" void LLVMInitializeC65AsmParser() {
  RegisterMCAsmParser<C65AsmParser> X(The65C816Target);
}
