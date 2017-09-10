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
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/SourceMgr.h"

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

class WLAKObjectWriter;

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
  static WLAKCalcStackEntry createImm(int Imm) {
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
    OP_AND         =  6,
    OP_DIVIDE      =  7,
    OP_POWER       =  8,
    OP_SHIFT_LEFT  =  9,
    OP_SHIFT_RIGHT = 10,
    OP_MODULO      = 11,
    OP_XOR         = 12,
    OP_LOW_BYTE    = 13,
    OP_HIGH_BYTE   = 14
  };
  void write(MCAssembler &Asm, WLAKObjectWriter &Writer) const;
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
  void addSymb(const MCSymbol &Symbol) {
    Stack.push_back(WLAKCalcStackEntry::createSymb(Symbol));
  }

  void write(MCAssembler &Asm, WLAKObjectWriter &Writer, unsigned ID) const;
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

  void write(MCAssembler &Asm, WLAKObjectWriter &Writer) const;
};

class WLAKObjectWriter : public MCObjectWriter {
protected:
  static bool isFixupKindPCRel(const MCAssembler &Asm, unsigned Kind);

  std::vector<WLAKSimpleRelocationEntry> SimpleRelocations;

  std::vector<WLAKComplexRelocationEntry> ComplexRelocations;

  // Maps MCSection to WLAK section ID.
  llvm::DenseMap<const MCSection *, unsigned> SectionIDMap;

  // Symbol information.
  struct SymbolInfo {
    bool Exported;
    bool Private;
    SymbolInfo() {}
  };
  DenseMap<const MCSymbol *, SymbolInfo> SymbolInfoMap;

  // If we have symbols without file or line number information, they
  // are assigned this file ID. Set to 0 if no such symbols are
  // encountered, in which case this "unknown" file is not emitted to
  // the source file list.
  unsigned UnknownFileID;

  unsigned NextSourceID;
  SmallDenseMap<unsigned, unsigned> BufferIDMap;
  SmallDenseMap<unsigned, const char*> SourceFilenameMap;

  std::pair<unsigned, unsigned>
  getFileAndLine(const MCAssembler &Asm, const MCFixup &Fixup);

  void getSections(MCAssembler &Asm, std::vector<const MCSection*> &Sections);

  void enumerateSections(MCAssembler &Asm, const MCAsmLayout &Layout);

public:
  WLAKObjectWriter(raw_pwrite_stream &_OS)
    : MCObjectWriter(_OS, false), UnknownFileID(0), NextSourceID(0) {};

  virtual ~WLAKObjectWriter() {}

  unsigned getSectionID(const MCSection &Section) const {
    return SectionIDMap.lookup(&Section);
  }

  bool symbolExists(const MCSymbol &Symb) const {
    return SymbolInfoMap.find(&Symb) != SymbolInfoMap.end();
  }

  // bool isSymbolExternal(const MCSymbol &Symb) const {
  //   return SymbolInfoMap.lookup(&Symb).External;
  // }

  bool isSymbolPrivate(const MCSymbol &Symb) const {
    assert (symbolExists(Symb));
    return Symb.isTemporary() || SymbolInfoMap.lookup(&Symb).Private;
 // &&
 //       !SymbolInfoMap.lookup(&Symb).External);
  }

  /// \brief Record a relocation entry.
  ///
  /// This routine is called by the assembler after layout and relaxation, and
  /// post layout binding. The implementation is responsible for storing
  /// information about the relocation so that it can be emitted during
  /// WriteObject().
  void recordRelocation(MCAssembler &Asm, const MCAsmLayout &Layout,
                        const MCFragment *Fragment,
                        const MCFixup &Fixup, MCValue Target,
                        bool &IsPCRel, uint64_t &FixedValue) override;

  /// \brief Perform any late binding of symbols (for example, to assign symbol
  /// indices for use when generating relocations).
  ///
  /// This routine is called by the assembler after layout and relaxation is
  /// complete.
  virtual void executePostLayoutBinding(MCAssembler &Asm,
                                        const MCAsmLayout &Layout) override;
  void writeSection(MCAssembler &Asm,
                    const MCAsmLayout &Layout,
                    const MCSection &Section);

  void writeSymbolName(MCAssembler &Asm,
                       const MCSymbol &Symbol);

  void writeSymbol(MCAssembler &Asm,
                   const MCAsmLayout &Layout,
                   const MCSymbol &Symbol);

  void writeSymbolTable(MCAssembler &Asm,
                        const MCAsmLayout &Layout);

  void writeSourceFiles(MCAssembler &Asm, const MCAsmLayout &Layout);

  virtual void writeObject(MCAssembler &Asm,
                           const MCAsmLayout &Layout) override;
};
} // end anonymous namespace

// WLAKCalcStackEntry Impl

void WLAKCalcStackEntry::write(MCAssembler &Asm,
                               WLAKObjectWriter &Writer) const {
  Writer.write8(Type);
  Writer.write8(Sign);
  if (Type == VALUE || Type == OPERATOR) {
    const uint64_t *X = (const uint64_t *)(&Value);
    Writer.writeBE64(*X);
  } else {
    Writer.writeSymbolName(Asm, *Symbol);
    Writer.write8(0);
  }
}

// WLAKComplexRelocationEntry Impl

void WLAKComplexRelocationEntry::write(MCAssembler &Asm,
                                       WLAKObjectWriter &Writer,
                                       unsigned ID) const {
  Writer.writeBE32(ID);
  if (Type == WLAKRelocationEntry::DIRECT_8BIT)
    Writer.write8(0x0);
  else if (Type == WLAKRelocationEntry::DIRECT_16BIT)
    Writer.write8(0x1);
  else if (Type == WLAKRelocationEntry::DIRECT_24BIT)
    Writer.write8(0x2);
  else if (Type == WLAKRelocationEntry::RELATIVE_8BIT)
    Writer.write8(0x80);
  else /* Type == WLAKRelocationEntry::RELATIVE_16BIT */
    Writer.write8(0x81);
  Writer.write8(Writer.getSectionID(*Section));
  Writer.write8(FileID);
  Writer.write8(Stack.size());
  Writer.write8(0);
  Writer.writeBE32(Offset);
  Writer.writeBE32(LineNumber);
  for (const WLAKCalcStackEntry &E : Stack)
    E.write(Asm, Writer);
}

// WLAKSimpleRelocationEntry Impl

void WLAKSimpleRelocationEntry::write(MCAssembler &Asm,
                                      WLAKObjectWriter &Writer) const {
  Writer.writeSymbolName(Asm, *Symbol);
  Writer.write8(0);
  if (Type == WLAKRelocationEntry::DIRECT_8BIT)
    Writer.write8(0x2);
  else if (Type == WLAKRelocationEntry::DIRECT_16BIT)
    Writer.write8(0x0);
  else if (Type == WLAKRelocationEntry::DIRECT_24BIT)
    Writer.write8(0x3);
  else if (Type == WLAKRelocationEntry::RELATIVE_8BIT)
    Writer.write8(0x1);
  else /* Type == WLAKRelocationEntry::RELATIVE_16BIT */
    Writer.write8(0x4);
  Writer.write8(Writer.getSectionID(*Section));
  Writer.write8(FileID);
  Writer.writeBE32(LineNumber);
  Writer.writeBE32(Offset);
}

// WLAKObjectWriter Impl

static unsigned getRelocType(const MCFixup &Fixup) {
  switch((unsigned)Fixup.getKind()) {
  default:
    llvm_unreachable("Unimplemented fixup -> relocation");
  case FK_PCRel_1:
    return WLAKRelocationEntry::RELATIVE_8BIT;
  case FK_PCRel_2:
    return WLAKRelocationEntry::RELATIVE_16BIT;
  case FK_Data_1:
  case C65::FK_C65_8:
  case C65::FK_C65_8s8:
  case C65::FK_C65_8s16:
  case C65::FK_C65_8s24:
  case C65::FK_C65_8s32:
  case C65::FK_C65_8s40:
  case C65::FK_C65_8s48:
  case C65::FK_C65_8s56:
    return WLAKRelocationEntry::DIRECT_8BIT;
  case FK_Data_2:
  case C65::FK_C65_16:
  case C65::FK_C65_16s16:
  case C65::FK_C65_16s32:
  case C65::FK_C65_16s48:
    return WLAKRelocationEntry::DIRECT_16BIT;
  case FK_Data_4:
  case C65::FK_C65_24:
    return WLAKRelocationEntry::DIRECT_24BIT;
  }
}

static unsigned getFixupShiftAmt(const MCFixup &Fixup) {
  int Kind = Fixup.getKind();
  if (C65::isFixup8Bit(Kind))
    return C65::get8BitFixupShiftAmt(Kind);
  else if (C65::isFixup16Bit(Kind))
    return C65::get16BitFixupShiftAmt(Kind);
  else
    return 0;
}

std::pair<unsigned, unsigned>
WLAKObjectWriter::getFileAndLine(const MCAssembler &Asm,
                                 const MCFixup &Fixup) {
  MCContext &Ctx = Asm.getContext();
  const SourceMgr *SrcMgr = Ctx.getSourceManager();
  if (!SrcMgr) {
    if (!UnknownFileID)
      UnknownFileID = NextSourceID++;
    return std::make_pair(UnknownFileID, 0);
  }

  SMLoc Loc = Fixup.getLoc();
  if (!Loc.isValid()) {
    if (!UnknownFileID)
      UnknownFileID = NextSourceID++;
    return std::make_pair(UnknownFileID, 0);
  }

  unsigned BufferID = SrcMgr->FindBufferContainingLoc(Loc);
  unsigned SourceID;

  // See if we have already assigned an ID for this buffer.
  auto I = BufferIDMap.find(BufferID);
  if (I != BufferIDMap.end()) {
    SourceID = I->second;
  } else {
    SourceID = NextSourceID++;
    BufferIDMap[BufferID] = SourceID;
  }
  unsigned LineNumber = SrcMgr->FindLineNumber(Loc, BufferID);
  return std::make_pair(SourceID, LineNumber);
}

void WLAKObjectWriter::recordRelocation(MCAssembler &Asm,
                                        const MCAsmLayout &Layout,
                                        const MCFragment *Fragment,
                                        const MCFixup &Fixup,
                                        MCValue Target,
                                        bool &IsPCRel,
                                        uint64_t &FixedValue) {
  const MCSection *Section = Fragment->getParent();
  std::pair<unsigned, unsigned> FileAndLine = getFileAndLine(Asm, Fixup);
  unsigned FileID = FileAndLine.first;
  unsigned LineNumber = FileAndLine.second;

  unsigned C = Target.getConstant();
  unsigned Offset = Layout.getFragmentOffset(Fragment) + Fixup.getOffset();
  unsigned Type = getRelocType(Fixup);
  unsigned ShiftAmt = getFixupShiftAmt(Fixup);

  const MCSymbolRefExpr *RefA = Target.getSymA();
  const MCSymbolRefExpr *RefB = Target.getSymB();
  const MCSymbol &SymA = RefA->getSymbol();

  if (ShiftAmt || RefB || C) {
    // WLAK supports arbitrary calculations for relocations using a
    // stack-based language.
    WLAKComplexRelocationEntry Rel(*Section, Type, FileID, LineNumber, Offset);
    Rel.addSymb(SymA);
    if (RefB) {
      assert(RefB->getKind() == MCSymbolRefExpr::VK_None &&
             "Should not have constructed this");

      const MCSymbol &SymB = RefB->getSymbol();
      assert(!SymB.isAbsolute() && "Should have been folded");

      Rel.addSymb(RefB->getSymbol());
      Rel.addOp(WLAKCalcStackEntry::OP_SUB);
    }
    if (C) {
      Rel.addImm(C);
      Rel.addOp(WLAKCalcStackEntry::OP_ADD);
    }
    if (ShiftAmt) {
      Rel.addImm(ShiftAmt);
      Rel.addOp(WLAKCalcStackEntry::OP_SHIFT_RIGHT);
    }
    ComplexRelocations.push_back(Rel);
  } else {
    WLAKSimpleRelocationEntry Rel(*Section, Type, FileID, LineNumber,
                                  Offset, SymA);
    SimpleRelocations.push_back(Rel);
  }
}

void WLAKObjectWriter::executePostLayoutBinding(MCAssembler &Asm,
                                                const MCAsmLayout &Layout) {
  for (const MCSymbol &SD : Asm.symbols()) {
    SymbolInfo SI;
    SI.Exported = SD.isInSection() && !SD.getName().empty();
    SI.Private = SD.isDefined() && !SD.isExternal();
    SymbolInfoMap[&SD] = SI;
  }
}

void WLAKObjectWriter::writeSection(MCAssembler &Asm,
                                    const MCAsmLayout &Layout,
                                    const MCSection &Section) {
  unsigned Size = Layout.getSectionFileSize(&Section);
  StringRef SectionName;
  SectionKind Kind = Section.getKind();

  if (Kind.isText())
    SectionName = StringRef("TEXT");
  else if (Kind.isData())
    SectionName = StringRef("DATA_REL");
  // else if (Kind.isDataRelLocal())
  //   SectionName = StringRef("DATA_REL_LOCAL");
  // else if (Kind.isDataNoRel())
  //   SectionName = StringRef("DATA_NOREL");
  else
    SectionName = StringRef("UNKNOWN");
  getStream() << SectionName;
  // String terminator decides section constraint
  write8(0); // 0 - FREE, 7 - SUPERFREE
  write8(getSectionID(Section)); // Section ID
  write8(1); // FileID
  writeBE32(Size);
  writeBE32(1); // Alignment
  Asm.writeSectionData(&Section, Layout);
  write8(0); // List file information, 0 - not present
}

// For the WLAK file type, we need to add underscores to names that
// are not external and not guaranteed to be unique across all object
// files. This function serves this purpose.
//
void WLAKObjectWriter::writeSymbolName(MCAssembler &Asm,
                                       const MCSymbol &Symbol) {
  raw_pwrite_stream &OS = getStream();
  // Append the file name to private symbols. We can't use underscores
  // here, since they are section-private and does not resolve across
  // sections.
  if (isSymbolPrivate(Symbol)) {
    ArrayRef<std::string> FileNames = Asm.getFileNames();
    if (FileNames.begin() != FileNames.end()) {
      std::string Name = *FileNames.begin();
      for (auto I = Name.begin(), E = Name.end(); I != E; ++I) {
        if (*I == '_')
          OS << '~';
        else
          OS << *I;
      }
      OS << '~';
    } else {
      // With no file name defined, prepend an underscore.
      OS << '_';
    }
  }
  getStream() << Symbol.getName();
}

void WLAKObjectWriter::writeSymbol(MCAssembler &Asm,
                                   const MCAsmLayout &Layout,
                                   const MCSymbol &Symbol) {
  writeSymbolName(Asm, Symbol);
  // String terminator decides type
  write8(0); // 0 - label, 1 - symbol, 2 - breakpoint
  write8(getSectionID(Symbol.getSection())); // Section ID
  write8(1); // File ID
  writeBE32(0); // Line number

  // Write symbol offset
  uint64_t Val;
  if (!Layout.getSymbolOffset(Symbol, Val))
    report_fatal_error("expected absolute expression");
  writeBE32(Val);
}

void WLAKObjectWriter::writeSymbolTable(MCAssembler &Asm,
                                        const MCAsmLayout &Layout) {
  unsigned NumExportedSymbols = 0;

  for (auto I : SymbolInfoMap) {
    if (I.second.Exported)
      ++NumExportedSymbols;
  }

  writeBE32(NumExportedSymbols);

  for (auto I : SymbolInfoMap) {
    if (I.second.Exported)
      writeSymbol(Asm, Layout, *I.first);
  }

  // for (const MCSymbolData &SD : Asm.symbols()) {
  //   const MCSymbol &Symbol = SD.getSymbol();
  //   SymbolInfo SI;
  //   if (Symbol.isDefined() && Symbol.isInSection()) {
  //     SI.Exists = true;
  //     SI.External =
  //       SD.isExternal() ||
  //       (!SD.getFragment() && !SD.getSymbol().isVariable());
  //     SymbolInfoMap[&Symbol] = SI;
  //     NumSymbols++;
  //   }
  // }
  // for (const MCSymbolData &SD : Asm.symbols()) {
  //   const MCSymbol &Symbol = SD.getSymbol();
  //   if (Symbol.isDefined()) {
  //     SymbolInfo SI;
  //     SI.Exists = true;
  //     SI.Exported = !Symbol.isTemporary() ||
  //       (Symbol.isInSection() && !Symbol.isVariable());
  //     SI.Local = !SD.isExternal() && Symbol.isInSection();
  //     SI.External = SD.isExternal() ||
  //       (!SD.getFragment() && !Symbol.isVariable());
  //       // SD.isExternal() ||
  //       // (!Symbol.isInSection() && !SD.getFragment() &&
  //       //  !SD.getSymbol().isVariable());
  //     SymbolInfoMap[&Symbol] = SI;
  //   }
  // }

}

void WLAKObjectWriter::enumerateSections(MCAssembler &Asm,
                                         const MCAsmLayout &Layout) {
  unsigned NextID = 1;
  for (const MCSection &Section : Asm)
    SectionIDMap[&Section] = NextID++;
}

void WLAKObjectWriter::writeSourceFiles(MCAssembler &Asm,
                                        const MCAsmLayout &Layout) {
  // If there is no buffer information, try to use the file names
  // supplied in Asm.
  if (BufferIDMap.empty()) {
    // In case there are no symbols, UnknownFileID will be empty even
    // here.
    if (!UnknownFileID)
      UnknownFileID = NextSourceID++;

    //    unsigned SourceFileCount = 0;
    ArrayRef<std::string> FileNames = Asm.getFileNames();
    if (FileNames.begin() != FileNames.end() &&
        FileNames.begin() + 1 == FileNames.end()) {
      // There is only one file name, use it as the "unknown file".
      writeBE32(1);
      getStream() << *FileNames.begin() << '\0';
      write8(UnknownFileID);
      return;
    }
  }

  // unsigned SourceFileCount = 0;
  // for (auto I = Asm.file_names_begin(), E = Asm.file_names_end();
  //      I != E; ++I)
  //   ++SourceFileCount;
  // writeBE32(SourceFileCount);
  // assert(SourceFileCount == (Asm.file_names_end() - Asm.file_names_begin()));
  // assert(SourceFileCount < 255);

  //  BufferIDMap[&SD.getSection()] = NextID++;

  MCContext &Ctx = Asm.getContext();
  const SourceMgr *SrcMgr = Ctx.getSourceManager();
  if (UnknownFileID)
    writeBE32(BufferIDMap.size() + 1);
  else
    writeBE32(BufferIDMap.size());

  for (auto I = BufferIDMap.begin(), E = BufferIDMap.end(); I != E; ++I) {
    unsigned BufferID = I->first;
    const char *Identifier = nullptr;
    if (SrcMgr) {
      const MemoryBuffer *MemBuff = SrcMgr->getMemoryBuffer(BufferID);
      if (MemBuff && MemBuff->getBufferIdentifier())
        Identifier = MemBuff->getBufferIdentifier();
    }
    if (Identifier)
      getStream() << Identifier;
    else
      getStream() << "anonymous file " << BufferID;
    getStream() << '\0';
    write8(I->second);
  }
  if (UnknownFileID) {
    getStream() << "unknown file" << '\0';
    write8(UnknownFileID);
  }

  //  SourceMgr::SrcBuffer Buff = SourceMgr->getBufferInfo(BufferID);
  //  const char *BuffIdent = MemBuff->getBufferIdentifier();

  // unsigned FileId = 1;
  // for (auto I = Asm.file_names_begin(), E = Asm.file_names_end();
  //      I != E; ++I) {
  // }
}

void WLAKObjectWriter::writeObject(MCAssembler &Asm,
                                   const MCAsmLayout &Layout) {
  enumerateSections(Asm, Layout);

  // Header
  write8('W');
  write8('L');
  write8('A');
  write8('V'); // 'WLAV'

  // Write the name of the source files, if available.
  writeSourceFiles(Asm, Layout);

  // Write exported definitions
  writeBE32(0);

  // Write labels, symbols and breakpoints. Also creates a table
  // outlining which symbols are marked as private.
  writeSymbolTable(Asm, Layout);

  // Write "Outside references"
  writeBE32(SimpleRelocations.size());
  for (const WLAKSimpleRelocationEntry &Rel : SimpleRelocations) {
    Rel.write(Asm, *this);
  }

  // Write "Pending calculations"
  writeBE32(ComplexRelocations.size());
  unsigned CalcID = 1;
  for (const WLAKComplexRelocationEntry &Rel : ComplexRelocations) {
    Rel.write(Asm, *this, CalcID++);
  }

  // Write data sections
  std::vector<const MCSection*> Sections;
  getSections(Asm, Sections);
  for (const MCSection *Section : Sections) {
    writeSection(Asm, Layout, *Section);
  }
}

void
WLAKObjectWriter::getSections(MCAssembler &Asm,
                              std::vector<const MCSection*> &Sections) {
  for (MCAssembler::iterator IT = Asm.begin(),
         IE = Asm.end(); IT != IE; ++IT) {
    Sections.push_back(&(*IT));
  }
}

MCObjectWriter *llvm::createC65WLAKObjectWriter(raw_pwrite_stream &OS,
                                                uint8_t OSABI) {
  return new WLAKObjectWriter(OS);
}
