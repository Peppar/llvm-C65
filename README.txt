6502/65816 compatible target for LLVM.


CAVEATS

This is a *very* unstable, alpha version backend.

The "standard libraries" consist of a set of assembly routines that
need to be linked externally. See arith.s and shift.s in the SFC
example: https://github.com/Peppar/llvm-C65-sfc-example


BUILDING

Steps roughly based on https://llvm.org/docs/GettingStarted.html

1. Relative paths specified are important!

2. Check out the modified LLVM

git clone https://github.com/Peppar/llvm-C65-llvm.git llvm

3. Check out the target

cd llvm/lib/Target
git clone https://github.com/Peppar/llvm-C65.git C65

4. Check out the modified Clang

cd llvm/tools
git clone https://github.com/Peppar/llvm-C65-clang.git clang

5. Configure and build LLVM and Clang

mkdir build
cd build

cmake /path/to/llvm/src -DLLVM_TARGETS_TO_BUILD="X86;C65"
cmake --build .


TESTING

Using the backend to compile an SFC (SNES) "game":

https://github.com/Peppar/llvm-C65-sfc-example


TROUBLESHOOTING

If you run out of memory while linking a Debug build of clang,
try using the "gold" linker from GNU Binutils.

https://www.gnu.org/software/binutils/

If you want to debug clang with gdb, you need to execute the following
commands for breakpoints to work consistently:

set follow-fork-mode child
