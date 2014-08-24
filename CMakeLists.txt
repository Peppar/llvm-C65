set(LLVM_TARGET_DEFINITIONS C65.td)

tablegen(LLVM C65GenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM C65GenCallingConv.inc -gen-callingconv)
tablegen(LLVM C65GenDAGISel.inc -gen-dag-isel)
tablegen(LLVM C65GenInstrInfo.inc -gen-instr-info)
tablegen(LLVM C65GenRegisterInfo.inc -gen-register-info)
tablegen(LLVM C65GenSubtargetInfo.inc -gen-subtarget)
add_public_tablegen_target(C65CommonTableGen)

add_llvm_target(C65CodeGen
  C65AsmPrinter.cpp
  C65FrameLowering.cpp
  C65InstrInfo.cpp
  C65ISelLowering.cpp
  C65RegisterInfo.cpp
  C65SelectionDAGInfo.cpp
  C65Subtarget.cpp
  C65TargetMachine.cpp
  )

add_subdirectory(InstPrinter)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
