//===-- C65CallingConv.td - Calling Conventions for C65 --*- tablegen -*-===//

def CC_C65 : CallingConv<[
  CCIfType<[i8],  CCAssignToReg<[ZR0, ZR1, ZR2, ZR3, ZR4, ZR5, ZR6, ZR7]>>,
  CCIfType<[i16], CCAssignToReg<[ZR0W, ZR2W, ZR4W, ZR6W]>>,
  CCIfType<[i32], CCAssignToReg<[ZR0D, ZR4D]>>,
  CCIfType<[i64], CCAssignToReg<[ZR0Q]>>
]>;

def RetCC_C65 : CallingConv<[
  // Return values are passed exactly like arguments
  CCDelegateTo<CC_C65>
]>;

def CSR_C65 : CalleeSavedRegs<(add ZR16, ZR17, ZR18, ZR19, ZR20, ZR21,
                                   ZR22, ZR23, ZR24, ZR25, ZR26, ZR27,
                                   ZR28, ZR29, ZR30, ZR31,
                                   ZR16W, ZR18W, ZR20W, ZR22W, ZR24W,
                                   ZR26W, ZR28W, ZR30W,
                                   ZR16D, ZR20D, ZR24D, ZR28D,
                                   ZR16Q, ZR24Q)>;
