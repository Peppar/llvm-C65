//===- C65RegisterInfo.td - Describe the C65 Register File --*- tablegen -*-==//

class C65Reg<string n, bits<16> Enc, list<Register> subregs = []> : Register<n> {
  let Namespace = "C65";
  let HWEncoding = Enc;
  let SubRegs = subregs;
}

//let Namespace = "C65" in {
//  def sub_8bit    : SubRegIndex<8>;
//  def sub_8bit_hi : SubRegIndex<8, 8>;
//  def sub_16bit   : SubRegIndex<16>;
//}

// Register definitions

// 8-bit registers
//def AL : C65Reg<"al", 0>;
//def XL : C65Reg<"xl", 1>;
//def YL : C65Reg<"yl", 2>;

// 8-bit high registers
//def AH : C65Reg<"ah", 3>;

// 16-bit registers
//let SubRegIndices = [sub_8bit, sub_8bit_hi], CoveredBySubRegs = 1 in {
//  def A : C65Reg<"a", 0, [AL, AH]>, DwarfRegNum<[0]>;
//}
def A : C65Reg<"a", 0, []>, DwarfRegNum<[0]>;
def X : C65Reg<"x", 1, []>, DwarfRegNum<[1]>;
def Y : C65Reg<"y", 2, []>, DwarfRegNum<[2]>;

//let SubRegIndices = [sub_8bit] in {
//  def X : C65Reg<"x", 1, [XL]>, DwarfRegNum<[1]>;
//  def Y : C65Reg<"y", 2, [YL]>, DwarfRegNum<[2]>;
//}

def SP : C65Reg<"sp", 4>, DwarfRegNum<[3]>;
def DP : C65Reg<"db", 5>, DwarfRegNum<[4]>;
def PC : C65Reg<"pc", 6>, DwarfRegNum<[5]>;

def SR : C65Reg<"sr", 7>, DwarfRegNum<[6]>;

def PB : C65Reg<"pb", 8>, DwarfRegNum<[7]>;
def DB : C65Reg<"db", 9>, DwarfRegNum<[8]>;

//def ACC8 : RegisterClass<"C65", [i8], 8, (add AL)>;
def ACC16 : RegisterClass<"C65", [i16], 16, (add A)>;

//def IX8 : RegisterClass<"C65", [i8], 8, (add XL)>;
def IX16 : RegisterClass<"C65", [i16], 16, (add X)>;

//def IY8 : RegisterClass<"C65", [i8], 8, (add YL)>;
def IY16 : RegisterClass<"C65", [i16], 16, (add Y)>;

// SP, DP

def BANK_REG : RegisterClass<"C65", [i8], 8, (add PB, DB)>;
def PC_REG : RegisterClass<"C65", [i16], 16, (add PC)>;

def CCR : RegisterClass<"C65", [i8], 8, (add SR)> {
  let CopyCost = -1;
  let isAllocatable = 0;
}
