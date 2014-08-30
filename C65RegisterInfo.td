//===- C65RegisterInfo.td - Describe the C65 Register File --*- tablegen -*-==//

class C65Reg<string n> : Register<n> {
  let Namespace = "C65";
}
class C65RegWithSubregs<string n, list<Register> subregs>
  : RegisterWithSubRegs<n, subregs> {
  let Namespace = "C65";
}

let Namespace = "C65" in {
  def sub_8bit     : SubRegIndex<8>;
  def sub_8bit_hi  : SubRegIndex<8, 8>;
  def sub_16bit    : SubRegIndex<16>;
  def sub_16bit_hi : SubRegIndex<16>;
  def sub_32bit    : SubRegIndex<32>;
  def sub_32bit_hi : SubRegIndex<32, 32>;
}


// Zero-page registers
class C65ZReg<string n, bits<16> num> : C65Reg<n> {
  let HWEncoding = num;
}
class C65ZRegWithSubregs<string n, bits<16> num, list<Register> subregs>
  : C65RegWithSubregs<n, subregs> {
  let HWEncoding = num;
}

class C65ZReg8<string n, bits<16> num>
  : C65ZReg<n, num>;
class C65ZReg16<string n, bits<16> num,
                C65ZReg8 low, C65ZReg8 high>
  : C65ZRegWithSubregs<n, num, [low, high]> {
  let SubRegIndices = [sub_8bit, sub_8bit_hi];
}
class C65ZReg32<string n, bits<16> num,
                C65ZReg16 low, C65ZReg16 high>
  : C65ZRegWithSubregs<n, num, [low, high]> {
  let SubRegIndices = [sub_16bit, sub_16bit_hi];
}
class C65ZReg64<string n, bits<16> num,
                C65ZReg32 low, C65ZReg32 high>
  : C65ZRegWithSubregs<n, num, [low, high]> {
  let SubRegIndices = [sub_32bit, sub_32bit_hi];
}

foreach I = 0-31 in {
  def ZR#I : C65ZReg8<"ZR"#I, 0>;
}
foreach I = [0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30] in {
  def ZR#I#W : C65ZReg16<"ZR"#I#"W", I,
                         !cast<C65ZReg8>("ZR"#I),
                         !cast<C65ZReg8>("ZR"#!add(I, 1))>;
}
foreach I = [0,4,8,12,16,20,24,28] in {
  def ZR#I#D : C65ZReg32<"ZR"#I#"D", I,
                         !cast<C65ZReg16>("ZR"#I#"W"),
                         !cast<C65ZReg16>("ZR"#!add(I, 2)#"W")>;
}
foreach I = [0,8,16,24] in {
  def ZR#I#Q : C65ZReg64<"ZR"#I#"Q", I,
                         !cast<C65ZReg32>("ZR"#I#"D"),
                         !cast<C65ZReg32>("ZR"#!add(I, 4)#"D")>;
}

// Zero-page classes
def ZRC8  : RegisterClass<"C65", [i8], 8, (sequence "ZR%u", 0, 31)>;
def ZRC16 : RegisterClass<"C65", [i16], 16, (add ZR0W, ZR2W, ZR4W, ZR6W, ZR8W,
                                                 ZR10W, ZR12W, ZR14W, ZR16W,
                                                 ZR18W, ZR20W, ZR22W, ZR24W,
                                                 ZR26W, ZR28W, ZR30W)>;
def ZRC32 : RegisterClass<"C65", [i32], 32, (add ZR0D, ZR4D, ZR8D, ZR12D,
                                                 ZR16D, ZR20D, ZR24D, ZR28D)>;
def ZRC64 : RegisterClass<"C65", [i64], 64, (add ZR0Q, ZR8Q, ZR16Q, ZR24Q)>;


// 8-bit registers
def A  : C65Reg<"A">,  DwarfRegNum<[0]>;
def B  : C65Reg<"B">,  DwarfRegNum<[1]>;
def XL : C65Reg<"XL">, DwarfRegNum<[2]>;
def XH : C65Reg<"XH">, DwarfRegNum<[3]>;
def YL : C65Reg<"YL">, DwarfRegNum<[4]>;
def YH : C65Reg<"YH">, DwarfRegNum<[5]>;
def SL : C65Reg<"SL">, DwarfRegNum<[6]>;
def SH : C65Reg<"SH">, DwarfRegNum<[7]>;

// 16-bit registers
let SubRegIndices = [sub_8bit, sub_8bit_hi] in {
def C : C65RegWithSubregs<"C", [A, B]>, DwarfRegNum<[8]>;
def X : C65RegWithSubregs<"X", [XL, XH]>,  DwarfRegNum<[9]>;
def Y : C65RegWithSubregs<"Y", [YL, YH]>,  DwarfRegNum<[10]>;
def S : C65RegWithSubregs<"S", [SL, SH]>, DwarfRegNum<[11]>;
}

// Program counter
def PC : C65Reg<"PC">, DwarfRegNum<[12]>;

// Direct page pointer
def D : C65Reg<"D">, DwarfRegNum<[13]>;

// Program back register
def PBR : C65Reg<"PBR">, DwarfRegNum<[14]>;

// Data bank register
def DBR : C65Reg<"DBR">, DwarfRegNum<[15]>;

// Processor status register
def P : C65Reg<"P">, DwarfRegNum<[16]>;

// Accumulator register classes
// def ACC8  : RegisterClass<"C65", [i8],  8,  (add A)>;
// def ACC16 : RegisterClass<"C65", [i16], 16, (add C)>;

// // Other register classes
// def IX8   : RegisterClass<"C65", [i8],  8,  (add XL)>;
// def IX16  : RegisterClass<"C65", [i16], 16, (add X)>;
// def IY8   : RegisterClass<"C65", [i8],  8,  (add YL)>;
// def IY16  : RegisterClass<"C65", [i16], 16, (add Y)>;
// def IS8   : RegisterClass<"C65", [i8],  8,  (add SL)>;
// def IS16  : RegisterClass<"C65", [i16], 16, (add S)>;
// def ID16  : RegisterClass<"C65", [i16], 16, (add D)>;

let CopyCost = -1, isAllocatable = 0 in {
  def PC_REG   : RegisterClass<"C65", [i16], 16, (add PC)>;
  def BANK_REG : RegisterClass<"C65", [i8],  8,  (add PBR, DBR)>;
  def CCR      : RegisterClass<"C65", [i8],  8,  (add P)>;
}
