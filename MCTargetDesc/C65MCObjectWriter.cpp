//===-- C65MCObjectWriter.cpp - WLAK object writer ------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/C65MCTargetDesc.h"
#include "MCTargetDesc/C65MCFixups.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"

using namespace llvm;

namespace {


namespace WLAK {
enum {
  FREE = 0,
  FORCED = 1,
  OVERWRITE = 2,
  HEADER = 3,
  SEMIFREE = 4,
  ABSOLUTE = 5,
  RAM = 6,
  SUPERFREE = 7
};
}

typedef llvm::DenseMap<const MCSection *, unsigned> SectionIDMapType;

class WLAKCalcStackEntry {
  unsigned Type;
  unsigned Sign;
  // Source: wladx/wlalink/defines.h
  enum {
    VALUE = 0, OPERATOR = 1, STRING = 2, STACK = 4
  };
  union {
    double Value;
    const MCSymbol *Symbol;
  };
  WLAKCalcStackEntry(double Imm)
    : Type(VALUE), Sign(0), Value(Imm) {};
  WLAKCalcStackEntry(unsigned Op)
    : Type(OPERATOR), Sign(0), Value((double)Op) {};
  WLAKCalcStackEntry(const MCSymbol &Symbol, unsigned Sign)
    : Type(STRING), Sign(Sign), Symbol(&Symbol) {};

public:
  static WLAKCalcStackEntry createImm(unsigned Imm) {
    return WLAKCalcStackEntry((double)Imm);
  }
  static WLAKCalcStackEntry createOp(unsigned Op) {
    return WLAKCalcStackEntry(Op);
  }
  static WLAKCalcStackEntry createSymb(const MCSymbol &Symbol,
                                       bool Invert = false) {
    return WLAKCalcStackEntry(Symbol, Invert ? 1 : 0);
  }
  enum {
    OP_ADD         =  0,
    OP_SUB         =  1,
    OP_MUL         =  2,
    OP_OR          =  5,
    OP_AND         =  5,
    OP_DIVIDE      =  7,
    OP_POWER       =  8,
    OP_SHIFT_LEFT  =  9,
    OP_SHIFT_RIGHT = 10,
    OP_MODULO      = 11,
    OP_XOR         = 12,
    OP_LOW_BYTE    = 13,
    OP_HIGH_BYTE   = 14
  };
  void Write(MCObjectWriter &Writer) const {
    Writer.Write8(Type);
    Writer.Write8(Sign);
    if (Type == VALUE || Type == OPERATOR) {
      uint64_t *X = (uint64_t *)(&Value);
      Writer.WriteBE64(*X);
    } else {
      Writer.getStream() << Symbol->getName() << '\0';
    }
  }
};

class WLAKRelocationEntry {
protected:
  const MCSection *Section;
  unsigned Type;
  unsigned FileID;
  unsigned LineNumber;
  unsigned Offset;

public:
  WLAKRelocationEntry(const MCSection &Section, unsigned Type,
                      unsigned FileID, unsigned LineNumber,
                      unsigned Offset)
    : Section(&Section), Type(Type), FileID(FileID),
      LineNumber(LineNumber), Offset(Offset) {};

  const MCSection &getSection() {
    return *Section;
  }

  virtual ~WLAKRelocationEntry() {}
  enum {
    DIRECT_8BIT, DIRECT_16BIT, DIRECT_24BIT,
    RELATIVE_8BIT, RELATIVE_16BIT
  };
};

class WLAKComplexRelocationEntry : public WLAKRelocationEntry {
protected:
  std::vector<WLAKCalcStackEntry> Stack;

public:
  WLAKComplexRelocationEntry(const MCSection &Section, unsigned Type,
                             unsigned FileID, unsigned LineNumber,
                             unsigned Offset)
    : WLAKRelocationEntry(Section, Type, FileID, LineNumber, Offset) {};

  void addImm(int Value) {
    Stack.push_back(WLAKCalcStackEntry::createImm(Value));
  }
  void addOp(unsigned Op) {
    Stack.push_back(WLAKCalcStackEntry::createOp(Op));
  }
  void addSymb(const MCSymbol &Symb) {
    Stack.push_back(WLAKCalcStackEntry::createSymb(Symb));
  }

  void Write(MCObjectWriter &Writer, unsigned ID, SectionIDMapType &Map) const {
    Writer.WriteBE32(ID);
    if (Type == WLAKRelocationEntry::DIRECT_8BIT)
      Writer.Write8(0x0);
    else if (Type == WLAKRelocationEntry::DIRECT_16BIT)
      Writer.Write8(0x1);
    else if (Type == WLAKRelocationEntry::DIRECT_24BIT)
      Writer.Write8(0x2);
    else if (Type == WLAKRelocationEntry::RELATIVE_8BIT)
      Writer.Write8(0x80);
    else /* Type == WLAKRelocationEntry::RELATIVE_16BIT */
      Writer.Write8(0x81);
    Writer.Write8(Map.lookup(Section));
    Writer.Write8(FileID);
    Writer.Write8(Stack.size());
    Writer.Write8(0);
    Writer.WriteBE32(Offset);
    Writer.WriteBE32(LineNumber);
    for (const WLAKCalcStackEntry &E : Stack)
      E.Write(Writer);
  }
};

class WLAKSimpleRelocationEntry : public WLAKRelocationEntry {
protected:
  const MCSymbol *Symbol;

public:
  WLAKSimpleRelocationEntry(const MCSection &Section, unsigned Type,
                            unsigned FileID, unsigned LineNumber,
                            unsigned Offset, const MCSymbol &Symbol)
    : WLAKRelocationEntry(Section, Type, FileID, LineNumber, Offset),
      Symbol(&Symbol) {};

  void Write(MCObjectWriter &Writer, SectionIDMapType Map) const {
    Writer.getStream() << Symbol->getName() << '\0';
    if (Type == WLAKRelocationEntry::DIRECT_8BIT)
      Writer.Write8(0x2);
    else if (Type == WLAKRelocationEntry::DIRECT_16BIT)
      Writer.Write8(0x0);
    else if (Type == WLAKRelocationEntry::DIRECT_24BIT)
      Writer.Write8(0x3);
    else if (Type == WLAKRelocationEntry::RELATIVE_8BIT)
      Writer.Write8(0x1);
    else /* Type == WLAKRelocationEntry::RELATIVE_16BIT */
      Writer.Write8(0x4);
    Writer.Write8(Map.lookup(Section));
    Writer.Write8(FileID);
    Writer.WriteBE32(LineNumber);
    Writer.WriteBE32(Offset);
  }
};

class WLAKObjectWriter : public MCObjectWriter {
protected:
  static bool isFixupKindPCRel(const MCAssembler &Asm, unsigned Kind);

  std::vector<WLAKSimpleRelocationEntry> SimpleRelocations;

  std::vector<WLAKComplexRelocationEntry> ComplexRelocations;

  SectionIDMapType SectionIDMap;

  void GetSections(MCAssembler &Asm, std::vector<const MCSection*> &Sections);

  void EnumerateSections(MCAssembler &Asm, const MCAsmLayout &Layout);

public:
  WLAKObjectWriter(raw_ostream &_OS)
    : MCObjectWriter(_OS, false) {};

  virtual ~WLAKObjectWriter() {}

  unsigned getSectionID(const MCSection &Section) {
    return SectionIDMap.lookup(&Section);
  }

  raw_ostream &getStream() { return OS; }

  /// \brief Record a relocation entry.
  ///
  /// This routine is called by the assembler after layout and relaxation, and
  /// post layout binding. The implementation is responsible for storing
  /// information about the relocation so that it can be emitted during
  /// WriteObject().
  void RecordRelocation(const MCAssembler &Asm, const MCAsmLayout &Layout,
                        const MCFragment *Fragment, const MCFixup &Fixup,
                        MCValue Target, bool &IsPCRel,
                        uint64_t &FixedValue) override;

  /// \brief Perform any late binding of symbols (for example, to assign symbol
  /// indices for use when generating relocations).
  ///
  /// This routine is called by the assembler after layout and relaxation is
  /// complete.
  virtual void ExecutePostLayoutBinding(MCAssembler &Asm,
                                        const MCAsmLayout &Layout) override;

  void WriteSection(MCAssembler &Asm,
                    const MCAsmLayout &Layout,
                    const MCSection &Section);

  void WriteSymbol(MCAssembler &Asm,
                   const MCAsmLayout &Layout,
                   const MCSymbolData &SymbolData);

  void WriteSymbolTable(MCAssembler &Asm,
                        const MCAsmLayout &Layout);

  virtual void WriteObject(MCAssembler &Asm,
                           const MCAsmLayout &Layout) override;
};
} // end anonymous namespace

// WLAKObjectWriter Impl

static unsigned GetRelocType(const MCValue &Target,
                             const MCFixup &Fixup,
                             bool IsPCRel) {
  switch((unsigned)Fixup.getKind()) {
  default:
    llvm_unreachable("Unimplemented fixup -> relocation");
  case FK_PCRel_1: return WLAKRelocationEntry::RELATIVE_8BIT;
  case FK_PCRel_2: return WLAKRelocationEntry::RELATIVE_16BIT;
  case FK_Data_1: return WLAKRelocationEntry::DIRECT_8BIT;
  case FK_Data_2: return WLAKRelocationEntry::DIRECT_16BIT;
  case FK_Data_4: return WLAKRelocationEntry::DIRECT_24BIT;
  }
}

void WLAKObjectWriter::RecordRelocation(const MCAssembler &Asm,
                                        const MCAsmLayout &Layout,
                                        const MCFragment *Fragment,
                                        const MCFixup &Fixup,
                                        MCValue Target,
                                        bool &IsPCRel,
                                        uint64_t &FixedValue) {
  const MCSectionData *FixupSection = Fragment->getParent();
  unsigned C = Target.getConstant();
  unsigned Offset = Layout.getFragmentOffset(Fragment) + Fixup.getOffset();
  unsigned Type = GetRelocType(Target, Fixup, IsPCRel);
  unsigned FileID = 1;
  unsigned LineNumber = 0;

  const MCSymbolRefExpr *RefA = Target.getSymA();
  const MCSymbolRefExpr *RefB = Target.getSymB();
  const MCSymbol &SymA = RefA->getSymbol();

  const MCSection &Section = FixupSection->getSection();
  //  assert(SymA.isInSection());

  if (RefB || C) {
    // WLAK supports arbitrary calculations for relocations using a
    // stack-based language.
    WLAKComplexRelocationEntry Rel(Section, Type, FileID, LineNumber, Offset);
    Rel.addSymb(SymA);
    if (RefB) {
      assert(RefB->getKind() == MCSymbolRefExpr::VK_None &&
             "Should not have constructed this");

      const MCSymbol &SymB = RefB->getSymbol();
      //      const MCSection &SecB = SymB.getSection();
      //      assert(&SecB == &FixupSection->getSection());
      assert(!SymB.isAbsolute() && "Should have been folded");

      Rel.addSymb(RefB->getSymbol());
      Rel.addOp(WLAKCalcStackEntry::OP_SUB);
    }
    if (C) {
      Rel.addImm(C);
      Rel.addOp(WLAKCalcStackEntry::OP_ADD);
    }
    ComplexRelocations.push_back(Rel);
  } else {
    WLAKSimpleRelocationEntry Rel(Section, Type, FileID, LineNumber,
                                  Offset, SymA);
    SimpleRelocations.push_back(Rel);
  }
}

void WLAKObjectWriter::ExecutePostLayoutBinding(MCAssembler &Asm,
                                                const MCAsmLayout &Layout) {}

void WLAKObjectWriter::WriteSection(MCAssembler &Asm,
                                    const MCAsmLayout &Layout,
                                    const MCSection &Section) {
  const MCSectionData &SD = Asm.getOrCreateSectionData(Section);
  unsigned Size = Layout.getSectionFileSize(&SD);
  StringRef SectionName;
  SectionKind Kind = Section.getKind();

  if (Kind.isText())
    SectionName = StringRef("TEXT");
  else if (Kind.isDataRel())
    SectionName = StringRef("DATA_REL");
  else if (Kind.isDataRelLocal())
    SectionName = StringRef("DATA_REL_LOCAL");
  else if (Kind.isDataNoRel())
    SectionName = StringRef("DATA_NOREL");
  else
    SectionName = StringRef("UNKNOWN");
  getStream() << SectionName;
  // String terminator decides section constraint
  Write8(0); // 0 - FREE, 7 - SUPERFREE
  Write8(getSectionID(Section)); // Section ID
  Write8(1); // FileID
  WriteBE32(Size);
  WriteBE32(1); // Alignment
  Asm.writeSectionData(&SD, Layout);
  Write8(0); // List file information, 0 - not present
}

void WLAKObjectWriter::WriteSymbol(MCAssembler &Asm,
                                   const MCAsmLayout &Layout,
                                   const MCSymbolData &SymbolData) {
  const MCSymbol &Symbol = SymbolData.getSymbol();
  getStream() << Symbol.getName();
  // String terminator decides type
  Write8(0); // 0 - is label, 1 - symbol, 2 - breakpoint
  Write8(getSectionID(Symbol.getSection())); // Section ID
  Write8(1); // File ID
  WriteBE32(0); // Line number
  WriteBE32(Layout.getSymbolOffset(&SymbolData));
}

void WLAKObjectWriter::WriteSymbolTable(MCAssembler &Asm,
                                        const MCAsmLayout &Layout) {
  unsigned NumSymbols = 0;

  for (const MCSymbolData &SD : Asm.symbols()) {
    const MCSymbol &Symbol = SD.getSymbol();
    if (Symbol.isDefined() && Symbol.isInSection()) {
      NumSymbols++;
    }
  }

  WriteBE32(NumSymbols);

  for (const MCSymbolData &SD : Asm.symbols()) {
    const MCSymbol &Symbol = SD.getSymbol();
    if (Symbol.isDefined() && Symbol.isInSection()) {
      WriteSymbol(Asm, Layout, SD);
    }
  }
}

void WLAKObjectWriter::EnumerateSections(MCAssembler &Asm,
                                         const MCAsmLayout &Layout) {
  unsigned NextID = 1;
  for (const MCSectionData &SD : Asm.getSectionList())
    SectionIDMap[&SD.getSection()] = NextID++;
}

void WLAKObjectWriter::WriteObject(MCAssembler &Asm,
                                   const MCAsmLayout &Layout) {
  EnumerateSections(Asm, Layout);

  // Header
  Write8('W');
  Write8('L');
  Write8('A');
  Write8('V'); // 'WLAV'

  // Write source file name list
  WriteBE32(1);
  OS << "Sourcecode.c" << '\0';
  Write8(1);

  // Write exported definitions
  WriteBE32(0);

  // Write labels, symbols and breakpoints
  WriteSymbolTable(Asm, Layout);

  // Write "Outside references"
  WriteBE32(SimpleRelocations.size());
  for (const WLAKSimpleRelocationEntry &Rel : SimpleRelocations) {
    Rel.Write(*this, SectionIDMap);
  }

  // Write "Pending calculations"
  WriteBE32(ComplexRelocations.size());
  unsigned CalcID = 1;
  for (const WLAKComplexRelocationEntry &Rel : ComplexRelocations) {
    Rel.Write(*this, CalcID++, SectionIDMap);
  }

  // Write data sections
  std::vector<const MCSection*> Sections;
  GetSections(Asm, Sections);
  for (const MCSection *Section : Sections) {
    WriteSection(Asm, Layout, *Section);
  }
}

void
WLAKObjectWriter::GetSections(MCAssembler &Asm,
                              std::vector<const MCSection*> &Sections) {
  for (MCAssembler::iterator IT = Asm.begin(),
         IE = Asm.end(); IT != IE; ++IT) {
    Sections.push_back(&IT->getSection());
  }
}

MCObjectWriter *llvm::createC65WLAKObjectWriter(raw_ostream &OS,
                                                uint8_t OSABI) {
  return new WLAKObjectWriter(OS);
}
