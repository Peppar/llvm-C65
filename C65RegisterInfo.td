//===- C65RegisterInfo.td - Describe the C65 Register File --*- tablegen -*-==//

class C65Reg<string n, list<Register> subregs = []> : Register<n> {
  let Namespace = "C65";
  let SubRegs = subregs;
}

// Accumulator register
def A : C65Reg<"A">, DwarfRegNum<[0]>;

// Index registers
def X : C65Reg<"X">, DwarfRegNum<[1]>;
def Y : C65Reg<"Y">, DwarfRegNum<[2]>;

// Stack pointer
def S : C65Reg<"S">, DwarfRegNum<[3]>;

// Program counter
def PC : C65Reg<"PC">, DwarfRegNum<[4]>;

// Direct page pointer
def D : C65Reg<"D">, DwarfRegNum<[5]>;

// Program back register
def PBR : C65Reg<"PBR">, DwarfRegNum<[7]>;

// Data bank register
def DBR : C65Reg<"DBR">, DwarfRegNum<[8]>;

// Processor status register
def P : C65Reg<"P">, DwarfRegNum<[6]>;


def ACC16 : RegisterClass<"C65", [i16], 16, (add A)>;
def IX16 : RegisterClass<"C65", [i16], 16, (add X)>;
def IY16 : RegisterClass<"C65", [i16], 16, (add Y)>;
def IS16 : RegisterClass<"C65", [i16], 16, (add S)>;
def ID16 : RegisterClass<"C65", [i16], 16, (add D)>;

let CopyCost = -1, isAllocatable = 0 in {
  def PC_REG : RegisterClass<"C65", [i16], 16, (add PC)>;
  def BANK_REG : RegisterClass<"C65", [i8], 8, (add PBR, DBR)>;
  def CCR : RegisterClass<"C65", [i8], 8, (add P)>;
}
