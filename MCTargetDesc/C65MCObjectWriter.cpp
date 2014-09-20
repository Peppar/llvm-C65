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

// class WLAKSection {
//   MCSection *Section;
//   StringRef SectionName;

//   unsigned ID;
//   unsigned Constraint;
//   unsigned Alignment;
//   unsigned Slot;
//   unsigned FileID;
//   unsigned Offset;
//   unsigned Bank;

//   const MCSymbol *Group;

// private:
//   WLAKSection(MCSection &Section, StringRef SectionName,
//               unsigned ID, unsigned Constraint, unsigned Alignment,
//               unsigned Slot, unsigned FileID,
//               unsigned Offset, unsigned Bank)
//     : Section(&Section), SectionName(Section), ID(ID),
//       Constraint(Constraint), Alignment(Alignment),
//       Slot(Slot), FileID(FileID),
//       Offset(Offset), Bank(Bank) {};

//   virtual ~MCSectionWLAK() {}

//   void setSectionName(StringRef Name) { SectionName = Name; }

// public:
//   enum {
//     FREE = 0,
//     FORCED = 1,
//     OVERWRITE = 2,
//     HEADER = 3,
//     SEMIFREE = 4,
//     ABSOLUTE = 5,
//     RAM = 6,
//     SUPERFREE = 7
//   };

//   const MCSymbol *getGroup() const { return Group; }

//   StringRef getSectionName() const { return SectionName; }

//   void Write(MCAssembler &Asm, const MCAsmLayout &Layout,
//              MCObjectWriter &Writer) const;
// };

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

// class WLAKCalcStackEntry {
// public:
//   virtual ~WLAKCalcStackEntry() {}
//   virtual void Write(MCObjectWriter &Writer) const = 0;
// };

// class WLAKCalcImm : public WLAKCalcStackEntry {
//   double Value;
// public:
//   WLAKCalcImm(int Value)
//     : Value(Value) {}
//   virtual void Write(MCObjectWriter &Writer) const override {
//     Writer.Write8(0);
//     Writer.getStream() << Value;
//   }
// };

// class WLAKCalcOp : public WLAKCalcStackEntry {
//   unsigned Op;
// public:
//   enum {
//     ADD = 0,
//     SUB = 1
//   };
//   WLAKCalcOp(unsigned Op)
//     : Op(Op) {}
//   virtual void Write(MCObjectWriter &Writer) const override {
//     Writer.Write8(1);
//     Writer.WriteLE32(Op);
//   }
// };

// class WLAKCalcSymb : public WLAKCalcStackEntry {
//   const MCSymbol *Symbol;
// public:
//   WLAKCalcSymb(const MCSymbol &Symbol)
//     : Symbol(&Symbol) {}
//   virtual void Write(MCObjectWriter &Writer) const override {
//     Writer.Write8(2);
//     Writer.getStream() << Symbol->getName();
//   }
// };

class WLAKRelocationEntry {
public:
  virtual ~WLAKRelocationEntry() {}
  virtual void Write(MCObjectWriter &Writer) const = 0;
  enum {
    DIRECT_8BIT, DIRECT_16BIT, DIRECT_24BIT,
    RELATIVE_8BIT, RELATIVE_16BIT
  };
};

class WLAKComplexRelocationEntry : public WLAKRelocationEntry {
  unsigned Type;
  unsigned FileId;
  unsigned Slot;
  unsigned Section;
  unsigned LineNumber;
  unsigned Offset;
  unsigned Bank;
  std::vector<WLAKCalcStackEntry> Stack;

public:
  WLAKComplexRelocationEntry(unsigned Type, unsigned FileId,
                             unsigned Slot, unsigned Section,
                             unsigned LineNumber, unsigned Offset,
                             unsigned Bank)
    : Type(Type), FileId(FileId), Slot(Slot), Section(Section),
      LineNumber(LineNumber), Offset(Offset), Bank(Bank) {}

  void addImm(int Value) {
    Stack.push_back(WLAKCalcStackEntry::createImm(Value));
  }
  void addOp(unsigned Op) {
    Stack.push_back(WLAKCalcStackEntry::createOp(Op));
  }
  void addSymb(const MCSymbol &Symb) {
    Stack.push_back(WLAKCalcStackEntry::createSymb(Symb));
  }

  virtual void Write(MCObjectWriter &Writer) const override {
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
    Writer.Write8(FileId);
    Writer.Write8(Stack.size());
    Writer.Write8(0);
    Writer.Write8(Slot);
    Writer.WriteBE32(Offset);
    Writer.WriteBE32(LineNumber);
    Writer.WriteBE32(Bank);
    for (const WLAKCalcStackEntry &E : Stack)
      E.Write(Writer);
  }
};

class WLAKSimpleRelocationEntry : public WLAKRelocationEntry {
  unsigned Type;
  unsigned FileId;
  unsigned Slot;
  unsigned Section;
  unsigned LineNumber;
  unsigned Offset;
  unsigned Bank;
  const MCSymbol *Symbol;

public:
  WLAKSimpleRelocationEntry(unsigned Type, unsigned FileId,
                            unsigned Slot, unsigned Section,
                            unsigned LineNumber, unsigned Offset,
                            unsigned Bank, const MCSymbol &Symbol)
    : Type(Type), FileId(FileId), Slot(Slot), Section(Section),
      LineNumber(LineNumber), Offset(Offset), Bank(Bank),
      Symbol(&Symbol) {}

  virtual void Write(MCObjectWriter &Writer) const override {
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
    Writer.Write8(FileId);
    Writer.Write8(Slot);
    Writer.Write8(Section);
    Writer.WriteBE32(LineNumber);
    Writer.WriteBE32(Offset);
    Writer.WriteBE32(Bank);
  }
};

class WLAKObjectWriter : public MCObjectWriter {
protected:
  static bool isFixupKindPCRel(const MCAssembler &Asm, unsigned Kind);

  std::vector<WLAKSimpleRelocationEntry> SimpleRelocations;

  std::vector<WLAKComplexRelocationEntry> ComplexRelocations;

  void GetSections(MCAssembler &Asm,
                   std::vector<const MCSection*> &Sections);

public:
  WLAKObjectWriter(raw_ostream &_OS)
    : MCObjectWriter(_OS, false) {};

  virtual ~WLAKObjectWriter() {}

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

  virtual void WriteObject(MCAssembler &Asm,
                           const MCAsmLayout &Layout) override;
};
} // end anonymous namespace

static unsigned GetRelocType(const MCValue &Target,
                             const MCFixup &Fixup,
                             bool IsPCRel) {
  if (IsPCRel) {
    switch((unsigned)Fixup.getKind()) {
    default:
      llvm_unreachable("Unimplemented fixup -> relocation");
    case FK_Data_1: return WLAKRelocationEntry::RELATIVE_8BIT;
    case FK_Data_2: return WLAKRelocationEntry::RELATIVE_16BIT;
    }
  }

  switch((unsigned)Fixup.getKind()) {
  default:
    llvm_unreachable("Unimplemented fixup -> relocation");
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
  unsigned FileId, Slot, Section, LineNumber, Bank;
  FileId = 0;
  Slot = 0;
  Section = 0;
  LineNumber = 0;
  Bank = 0;

  const MCSymbolRefExpr *RefA = Target.getSymA();
  const MCSymbolRefExpr *RefB = Target.getSymB();

  if (RefB || C) {
    // WLAK supports arbitrary calculations for relocations using a
    // stack-based language.
    WLAKComplexRelocationEntry Rel(Type, FileId, Slot, Section, LineNumber,
                                   Offset, Bank);
    Rel.addSymb(RefA->getSymbol());
    if (RefB) {
      assert(RefB->getKind() == MCSymbolRefExpr::VK_None &&
             "Should not have constructed this");

      const MCSymbol &SymB = RefB->getSymbol();
      const MCSection &SecB = SymB.getSection();
      assert(&SecB == &FixupSection->getSection());
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
    WLAKSimpleRelocationEntry Rel(Type, FileId, Slot, Section, LineNumber,
                                  Offset, Bank, RefA->getSymbol());
    SimpleRelocations.push_back(Rel);
  }
}

void WLAKObjectWriter::ExecutePostLayoutBinding(MCAssembler &Asm,
                                                const MCAsmLayout &Layout) {}

static void WriteSection(MCAssembler &Asm, const MCAsmLayout &Layout,
                         MCObjectWriter &Writer, const MCSection &Section,
                         unsigned Constraint, unsigned ID,
                         unsigned Slot, unsigned FileID,
                         unsigned Offset, unsigned Bank,
                         unsigned Alignment) {
  const MCSectionData &SD = Asm.getOrCreateSectionData(Section);
  unsigned Size = Layout.getSectionFileSize(&SD);
  StringRef SectionName;
  SectionKind Kind;

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
  for (StringRef::iterator I = SectionName.begin(), E = SectionName.end();
       I != E; ++I) {
    Writer.Write8(*I);
  }
  Writer.Write8(Constraint);
  Writer.Write8(ID);
  Writer.Write8(Slot);
  Writer.Write8(FileID);
  Writer.WriteBE32(Offset);
  Writer.WriteBE32(Bank);
  Writer.WriteBE32(Size);
  Writer.WriteBE32(Alignment);
  Asm.writeSectionData(&SD, Layout);
  Writer.Write8(0); // List file information
}

void WLAKObjectWriter::WriteObject(MCAssembler &Asm,
                                   const MCAsmLayout &Layout) {
  // Header
  Write8('W');
  Write8('L');
  Write8('A');
  Write8('K'); // 'WLAK'

  Write8(0x00); // Empty fill
  Write8(0x80); // Misc bits (0x80 -- 65C816)
  Write8(0x00); // More bits

  // Write ROM bank map
  WriteBE32(0);

  // Write memory map
  WriteBE32(0);

  // Write source file name list
  WriteBE32(1);
  OS << "Sourcecode.c";
  Write8(1);

  // Write exported definitions
  WriteBE32(0);

  // Write labels, symbols and breakpoints
  WriteBE32(0);

  // Write "Outside references"
  WriteBE32(SimpleRelocations.size());
  for (const WLAKSimpleRelocationEntry &Rel : SimpleRelocations) {
    Rel.Write(*this);
  }

  // Write "Pending calculations"
  WriteBE32(ComplexRelocations.size());
  for (const WLAKComplexRelocationEntry &Rel : ComplexRelocations) {
    Rel.Write(*this);
  }

  // Write data area
  std::vector<const MCSection*> Sections;
  GetSections(Asm, Sections);
  WriteBE32(Sections.size());

  for (const MCSection *Section : Sections) {
    WriteSection(Asm, Layout, *this, *Section,
                 WLAK::FREE, 1, 1, 1, 0, 0, 0);
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
