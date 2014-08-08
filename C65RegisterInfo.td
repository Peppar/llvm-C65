//===- C65RegisterInfo.td - Describe the C65 Register File --*- tablegen -*-==//

class C65Reg<string n, bits<16> Enc, list<Register> subregs = []> : Register<n> {
  let Namespace = "C65";
  let HWEncoding = Enc;
  let SubRegs = subregs;
}

def A : C65Reg<"A", 0, []>, DwarfRegNum<[0]>;
def X : C65Reg<"X", 1, []>, DwarfRegNum<[1]>;
def Y : C65Reg<"Y", 2, []>, DwarfRegNum<[2]>;

def SP : C65Reg<"SP", 3>, DwarfRegNum<[3]>;
def DP : C65Reg<"DB", 4>, DwarfRegNum<[4]>;
def PC : C65Reg<"PC", 5>, DwarfRegNum<[5]>;

def SR : C65Reg<"SR", 6>, DwarfRegNum<[6]>;

def PB : C65Reg<"PB", 7>, DwarfRegNum<[7]>;
def DB : C65Reg<"DB", 8>, DwarfRegNum<[8]>;

def ACC16 : RegisterClass<"C65", [i16], 16, (add A)>;

def IX16 : RegisterClass<"C65", [i16], 16, (add X)>;

def IY16 : RegisterClass<"C65", [i16], 16, (add Y)>;

def BANK_REG : RegisterClass<"C65", [i8], 8, (add PB, DB)>;
def PC_REG : RegisterClass<"C65", [i16], 16, (add PC)>;

def CCR : RegisterClass<"C65", [i8], 8, (add SR)> {
  let CopyCost = -1;
  let isAllocatable = 0;
}
