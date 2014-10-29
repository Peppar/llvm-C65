//===-- C65CallingConv.td - Calling Conventions for C65 --*- tablegen -*-===//

def CC_C65 : CallingConv<[
  CCIfType<[i8],  CCAssignToReg<[ZR0, ZR1, ZR2, ZR3, ZR4, ZR5, ZR6, ZR7]>>,
  CCIfType<[i16], CCAssignToReg<[ZR0W, ZR2W, ZR4W, ZR6W]>>,
  CCIfType<[i32], CCAssignToReg<[ZR0D, ZR4D]>>,
  CCIfType<[i64], CCAssignToReg<[ZR0Q]>>
]>;

def RetCC_C65 : CallingConv<[
  // Return values are passed exactly like arguments.
  CCDelegateTo<CC_C65>
]>;

// By default, save half of the ZP registers.
def CSR_C65 : CalleeSavedRegs<(add ZR16W, ZR18W, ZR20W, ZR22W, ZR24W,
                                   ZR26W, ZR28W, ZR30W)>;

// Libcalls are required to save all ZP registers that are touched.
def CSR_C65_AllRegs : CalleeSavedRegs<(add CSR_C65, ZR0W, ZR2W, ZR4W, ZR8W,
                                           ZR10W, ZR12W, ZR14W)>;
