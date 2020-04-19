6502/65816 compatible target for LLVM.


CAVEATS

This target is experimental, and outputs very unoptimized code.

For now, the target can output only to the library object format used
by WLA DX (WLA7). The target has been tested only on SFC (SNES)
emulators and hardware.

The "standard libraries" consist of a set of assembly routines that
need to be linked externally. See arith.s and shift.s in the SFC
example: https://github.com/Peppar/llvm-C65-sfc-example


BUILDING

Steps roughly based on https://llvm.org/docs/GettingStarted.html

1. Check out the target specific LLVM project https://github.com/Peppar/llvm-project-C65.git

git clone https://github.com/Peppar/llvm-project-C65.git llvm-project-C65
cd llvm-project-C65
git submodule update --init

2. Configure and build LLVM and Clang (here built in the subdirectory llvm-project-C65/build, customize if needed)

mkdir build
cd build
cmake ../llvm -DLLVM_TARGETS_TO_BUILD="X86" -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="C65" -DLLVM_ENABLE_PROJECTS="clang"
cmake --build .


TESTING

Using the target to compile an SFC (SNES) "game":

https://github.com/Peppar/llvm-C65-sfc-example


TROUBLESHOOTING

If you run out of memory while linking a Debug build of clang,
try using the "gold" linker from GNU Binutils.

https://www.gnu.org/software/binutils/

If you want to debug clang with gdb, you need to execute the following
commands for breakpoints to work consistently:

set follow-fork-mode child
