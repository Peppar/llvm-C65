//===-- C65MCObjectWriter.cpp - C65 ELF writer ----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/C65MCTargetDesc.h"
#include "MCTargetDesc/C65MCFixups.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCValue.h"

using namespace llvm;


namespace {

class MCSymbol;

class WLAKSection {
  StringRef SectionName;

  unsigned Constraint;
  unsigned ID;
  unsigned Slot;
  unsigned FileID;
  unsigned Alignment;
  unsigned Offset;

  const MCSymbol *Group

private:
  MCSectionWLAK(StringRef Section, unsigned ID, unsigned Constraint,
                unsigned Alignment, unsigned Slot, unsigned FileID,
                unsigned Offset)
    : SectionName(Section), ID(ID), Constraint(Constraint),
      Alignment(Alignment), Slot(Slot), FileID(FileID),
      Offset(Offset){};

  ~MCSectionWLAK();

  void setSectionName(StringRef Name) { SectionName = Name; }

public:
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

  const MCSymbol *getGroup() const { return Group; }

  StringRef getSectionName() const { return SectionName; }

  void Write(MCObjectWriter &Writer) const;
}

void WLAKSection::Write(MCObjectWriter &Writer) const {
  const MCSectionData &SD = Asm.getOrCreateSectionData(Section);
  unsigned Size = Layout.getSectionFileSize(&SD)

  for (iterator I = SectionName.begin(), E = SectionName.end();
       I != E; ++I) {
    Writer.Write8(*I);
  }
  Writer.Write8(Constraint);
  Writer.Write8(ID);
  Writer.Write8(Slot);
  Writer.Write8(FileID);
  Writer.Write8(Offset);
  Writer.Write8(Bank);
  Writer.Write8(Size);
  Writer.Write8(Alignment);
  Asm.writeSectionData(&SD, Layout);
  Writer.Write8(0); // List file information
}

class WLAKCalcStackEntry {
public:
  virtual ~WLAKCalcStackEntry();
  virtual void Write(MCObjectWriter &Writer) const;
}

class WLAKCalcImm : WLAKCalcStackEntry {
  double Value;
public:
  WLAKCalcImm(int Value)
    : Value(Value) {}
  virtual void Write(MCObjectWriter &Writer) const override {
    Writer.Write8(0);
    Writer.getStream() << Value;
  }
}

class WLAKCalcOp : WLAKCalcStackEntry {
  unsigned Op;
public:
  enum {
    ADD = 0,
    SUB = 1
  };
  WLAKCalcOp(unsigned Op)
    : Op(Op) {}
  virtual void Write(MCObjectWriter &Writer) const override {
    Writer.Write8(1);
  }
}

class WLAKCalcSymb : WLAKCalcStackEntry {
  const MCSymbol *Symbol;
public:
  WLAKCalcSymb(const MCSymbol *Symbol)
    : Symbol(Symbol) {}
  virtual void Write(MCObjectWriter &Writer) const override {
    Writer.Write8(2);
    Writer.getStream() << Symbol->getName();
  }
}

class WLAKRelocationEntry {
protected:
  unsigned Type;
  unsigned Section;
  unsigned FileId;
  unsigned LineNumber;
  unsigned Slot;
  unsigned Offset;
  unsigned Bank;

public:
  virtual ~WLAKRelocationEntry();
  virtual void Write(MCObjectWriter &Writer) const;

  enum {
    DIRECT_8BIT, DIRECT_16BIT, DIRECT_24BIT,
    RELATIVE_8BIT, RELATIVE_16BIT
  };
}

class WLAKComplexRelocationEntry {
protected:
  std::vector<WLAKCalcStackEntry> Stack;

public:
  WLAKComplexRelocationEntry(unsigned Type, unsigned FileId,
                             unsigned Slot, unsigned Section,
                             unsigned LineNumber, unsigned Offset,
                             unsigned Bank)
    : Type(Type), FileId(FileId), Slot(Slot), Section(Section),
      LineNumber(LineNumber), Offset(Offset), Bank(Bank) {}

  void addImm(int Value) {
    Stack.push_back(WLAKCalcImm(Value));
  }
  void addOp(unsigned Op) {
    Stack.push_back(WLAKCalcOp(Op));
  }
  void addSymb(const MCSymbol *Symb) {
    Stack.push_back(WLAKCalcSymb(Symb));
  }

  virtual void Write(MCObjectWriter &Writer) const override {
    if (Type == REL_DIRECT_8BIT)          Writer.Write8(0x0);
    else if (Type == REL_DIRECT_16BIT)    Writer.Write8(0x1);
    else if (Type == REL_DIRECT_24BIT)    Writer.Write8(0x2);
    else if (Type == REL_RELATIVE_8BIT)   Writer.Write8(0x80);
    else /* Type == REL_RELATIVE_16BIT */ Writer.Write8(0x81);
    Writer.Write8(FileId);
    Writer.Write8(Stack.count());
    Writer.Write8(0);
    Writer.Write8(Slot);
    Writer.WriteBE32(Offset);
    Writer.WriteBE32(LineNumber);
    Writer.WriteBE32(Bank);
    for (WLAKCalcStackEntry &E : Stack)
      E.Write(Writer);
  }
}

class WLAKSimpleRelocationEntry {
protected:
  const MCSymbol *Symbol;

public:
  WLAKSimpleRelocationEntry(unsigned Type, unsigned FileId,
                            unsigned Slot, unsigned Section,
                            unsigned LineNumber, unsigned Offset,
                            unsigned Bank, const MCSymbol *Symbol)
    : Type(Type), FileId(FileId), Slot(Slot), Section(Section),
      LineNumber(LineNumber), Offset(Offset), Bank(Bank),
      Symbol(Symbol) {}

  virtual void Write(MCObjectWriter &Writer) const override {
    Writer.getStream() << Symbol->getName();
    if (Type == REL_DIRECT_8BIT)          Writer.Write8(0x2);
    else if (Type == REL_DIRECT_16BIT)    Writer.Write8(0x0);
    else if (Type == REL_DIRECT_24BIT)    Writer.Write8(0x3);
    else if (Type == REL_RELATIVE_8BIT)   Writer.Write8(0x1);
    else /* Type == REL_RELATIVE_16BIT */ Writer.Write8(0x4);
    Writer.Write8(FileId);
    Writer.Write8(Slot);
    Writer.Write8(Section);
    Writer.WriteBE32(LineNumber);
    Writer.WriteBE32(Offset);
    Writer.WriteBE32(Bank);
  }
}

class C65ObjectWriter : MCObjectWriter {
protected:
  raw_ostream &OS;

  static bool isFixupKindPCRel(const MCAssembler &Asm, unsigned Kind);

  std::vector<WLAKSimpleRelocationEntry> SimpleRelocations;

  std::vector<WLAKComplexRelocationEntry> ComplexRelocations;

public:
  WLAKObjectWriter(raw_ostream &_OS)
    : MCObjectWriter(_OS, false);

  virtual ~WLAKObjectWriter();

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

  /// \brief Check whether the difference (A - B) between two symbol
  /// references is fully resolved.
  ///
  /// Clients are not required to answer precisely and may conservatively return
  /// false, even when a difference is fully resolved.
  bool
  IsSymbolRefDifferenceFullyResolved(const MCAssembler &Asm,
                                     const MCSymbolRefExpr *A,
                                     const MCSymbolRefExpr *B,
                                     bool InSet) const;

  // Override MCELFMCObjectWriter.
  unsigned GetRelocType(const MCValue &Target, const MCFixup &Fixup,
                        bool IsPCRel) const override;
};
} // end anonymous namespace

// bool WLAKObjectWriter::isFixupKindPCRel(const MCAssembler &Asm, unsigned Kind) {
//   const MCFixupKindInfo &FKI =
//     Asm.getBackend().getFixupKindInfo((MCFixupKind) Kind);

//   return FKI.Flags & MCFixupKindInfo::FKF_IsPCRel;
// }

// WLAKObjectWriter::WLAKObjectWriter(uint8_t OSABI)
//   : MCELFMCObjectWriter(/*Is64Bit=*/false, OSABI, ELF::EM_S390,
//                             /*HasRelocationAddend=*/ true) {}

WLAKObjectWriter::~WLAKObjectWriter() {}

// Emit the WLAK header.
void WLAKObjectWriter::WriteHeader(const MCAssembler &Asm) {
}

void WLAKObjectWriter::WriteROMBankMap(const MCAssembler &Asm) {
  Write32(ROMBankMap.size());
  for (unsigned I = 0, E = ROMBankMap.size(); I != E; ++I) {
    ...
  }
}


// class WLAKCalcStackEntry {
// public:
//   virtual ~WLAKCalcStackEntry();
//   virtual void Write(raw_ostream &OS) const = 0;
// }

// class WLAKCalcConstant : WLAKCalcStackEntry {
// private:
//   double Value;
// public:
//   WLAKCalcConstant(double _Value) : Value(_Value);
//   virtual void Write(raw_ostream &OS) const { OS << Value; }
// }

// class WLAKCalcOp : WLAKCalcStackEntry {
// private:
//   unsigned Op;
// public:
//   enum {
//     OP_PLUS = 0,
//     OP_MINUS = 1
//   };
//   WLAKCalcOp(unsigned _Op)
//     : Op(_Op);

//   virtual void Write(raw_ostream &OS) const {
//     OS << char(1) << char(Op);
//   }
// }

void WLAKObjectWriter::WriteLabelsSymbolsBreakpoints(const MCAssembler &Asm) {
}

void WLAKObjectWriter::WriteSection(const MCAssembler &Asm) {
}

static GetRelocType(const MCValue &Target,
                    const MCFixup &Fixup,
                    bool IsPCRel) const {
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

  const MCSymbolRefExpr *RefA = Target.getSymA();
  const MCSymbolRefExpr *RefB = Target.getSymB();

  if (RefB || C) {
    assert(RefB->getKind() == MCSymbolRefExpr::VK_None &&
           "Should not have constructed this");

    const MCSymbol &SymB = RefB->getSymbol();
    const MCSection &SecB = SymB.getSection();
    if (&SecB != &FixupSection->getSection())
      Asm.getContext().FatalError(
          Fixup.getLoc(), "Cannot represent a difference across sections");

    assert(!SymB.isAbsolute() && "Should have been folded");

    // WLAK supports arbitrary calculations for relocations using a
    // stack-based language.
    WLAKComplexRelocation Rel(Type, FileId, Slot, Section, LineNumber,
                              Offset, Bank);
    Rel.addSymb(RefA->getSymbol());
    if (RefB) {
      Rel.addSymb(RefB->getSymbol());
      Rel.addOp(WLAKCalcOp::SUB);
    }
    if (C) {
      Rel.addImm(C);
      Rel.addOp(WLAKCalcOp::ADD);
    }
    ComplexRelocations.push_back(Rec);
  } else {
    WLAKSimpleRelocation Rel(Type, FieldId, Slot, Section, LineNumber,
                             Offset, Bank, RefA->getSymbol());
    SimpleRelocations.push_back(Rec);
  }
}

void WLAKObjectWriter::WriteObject(MCAssembler &Asm,
                                   const MCAsmLayout &Layout) {
  // Header
  Write8('W');
  Write8('L');
  Write8('A');
  Write8('K'); // 'WLAK'

  Write8(0); // Empty fill
  Write8(0x80); // Misc bits (0x80 -- 65C816)
  Write8(0); // More bits

  // Write ROM bank map
  WriteLE32(0);

  // Write memory map
  WriteLE32(0);

  // Write source file name list
  WriteLE32(1);
  OS << "Sourcecode.c";
  Write8(1);

  // Write exported definitions
  WriteLE32(0);

  // Write labels, symbols and breakpoints
  WriteLE32(0);

  // Write "Outside references"
  WriteLE32(SimpleRelocations.Count());
  for (WLAKSimpleRelocation &Rel : SimpleRelocations) {
    Rel.Write(*this);
  }

  // Write "Pending calculations"
  WriteLE32(ComplexRelocations.Count());
  for (WLAKComplexRelocation &Rel : ComplexRelocations) {
    Rel.Write(*this);
  }

  // Write data area
}

MCObjectWriter *llvm::createC65WLAKObjectWriter(raw_ostream &OS,
                                                uint8_t OSABI) {
  return new WLAKObjectWriter(OS)
}
