//===- C65RegisterInfo.td - Describe the C65 Register File --*- tablegen -*-==//

class C65Reg<string n> : Register<n> {
  let Namespace = "C65";
}
class C65RegWithSubregs<string n, list<Register> subregs>
  : RegisterWithSubRegs<n, subregs> {
  let Namespace = "C65";
}

// Accumulator register
def A : C65Reg<"A">,   DwarfRegNum<[0]>;
def B : C65Reg<"B">,   DwarfRegNum<[1]>;
def C : C65RegWithSubregs<"C", [A, B]>, DwarfRegNum<[2]>;

// Index registers
def XL : C65Reg<"XL">, DwarfRegNum<[3]>;
def XH : C65Reg<"XH">, DwarfRegNum<[4]>;
def X  : C65RegWithSubregs<"X", [XL, XH]>,  DwarfRegNum<[5]>;
def YL : C65Reg<"YL">, DwarfRegNum<[6]>;
def YH : C65Reg<"YH">, DwarfRegNum<[7]>;
def Y  : C65RegWithSubregs<"Y", [YL, YH]>,  DwarfRegNum<[8]>;

// Stack pointer
def SL : C65Reg<"SL">, DwarfRegNum<[9]>;
def SH : C65Reg<"SH">, DwarfRegNum<[10]>;
def S : C65RegWithSubregs<"S", [SL, SH]>, DwarfRegNum<[11]>;

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

// Register classes
def ACC8  : RegisterClass<"C65", [i8],  8,  (add A)>;
def ACC16 : RegisterClass<"C65", [i16], 16, (add C)>;
def IX8   : RegisterClass<"C65", [i8],  8,  (add XL)>;
def IX16  : RegisterClass<"C65", [i16], 16, (add X)>;
def IY8   : RegisterClass<"C65", [i8],  8,  (add YL)>;
def IY16  : RegisterClass<"C65", [i16], 16, (add Y)>;
def IS8   : RegisterClass<"C65", [i8],  8,  (add SL)>;
def IS16  : RegisterClass<"C65", [i16], 16, (add S)>;
def ID16  : RegisterClass<"C65", [i16], 16, (add D)>;

let CopyCost = -1, isAllocatable = 0 in {
  def PC_REG   : RegisterClass<"C65", [i16], 16, (add PC)>;
  def BANK_REG : RegisterClass<"C65", [i8],  8,  (add PBR, DBR)>;
  def CCR      : RegisterClass<"C65", [i8],  8,  (add P)>;
}

// Zero-page registers
class C65ZReg<string n, bits<5> num> : C65Reg<n> {
  let HWEncoding = num;
}
class C65ZRegWithSubregs<string n, bits<5> num, C65ZReg low, C65ZReg high>
  : C65RegWithSubregs<n, num> {
  let HWEncoding = num;
}

class C65ZReg8<string n, bits<5> num>
  : C65ZReg<n, num>;
class C65ZReg16<string n, bits<5> num,
                C65ZReg8 low, C65ZReg8 high>
  : C65ZRegWithSubregs<n, num, [low, high]>;
class C65ZReg32<string n, bits<5> num,
                C65ZReg16 low, C65ZReg16 high>
  : C65ZRegWithSubregs<n, num, [low, high]>;
class C65ZReg64<string n, bits<5> num,
                C65ZReg32 low, C65ZReg32 high>
  : C65ZRegWithSubregs<n, num, [low, high]>;

foreach I = 0-31 in {
  def ZR#I : C65ZReg8<"ZP"#I, 0>;
}
foreach I = [0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30] in {
  def ZR#I#W : C65ZReg16<"ZP"#I#"W", I,
                         !cast<C65ZReg8>("ZP"#I)
                         !cast<C65ZReg8>("ZP"#!add(I, 1))>;
}
foreach I = [0,4,8,12,16,20,24,28] in {
  def ZR#I#D : C65ZReg32<"ZP"#I#"D", I,
                         !cast<C65ZReg16>("ZP"#I#"W")
                         !cast<C65ZReg16>("ZP"#!add(I, 2)#"W")>;
}
foreach I = [0,8,16,24] in {
  def ZR#I#Q : C65ZReg32<"ZP"#I#"Q", I,
                         !cast<C65ZReg32>("ZP"#I#"D")
                         !cast<C65ZReg32>("ZP"#!add(I, 4)#"D")>;
}

// Zero-page register classes
def ZR8  : RegisterClass<"C65", [i8], 8, (sequence "ZR%u", 0, 31)>;
def ZR16 : RegisterClass<"C65", [i16], 16, (add ZR0W, ZR2W, ZR4W, ZR6W, ZR8W,
                                                ZR10W, ZR12W, ZR14W, ZR16W,
                                                ZR18W, ZR20W, ZR22W, ZR24W,
                                                ZR26W, ZR28W, ZR30W)>;
def ZR32 : RegisterClass<"C65", [i32], 32, (add ZR0D, ZR4D, ZR8D, ZR12D,
                                                ZR16D, ZR20D, ZR24D, ZR28D)>;
def ZR64 : RegisterClass<"C65", [i64], 64, (add ZR0Q, ZR8Q, ZR16Q, ZR24Q)>;
