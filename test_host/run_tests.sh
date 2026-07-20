#!/usr/bin/env bash
# Host test driver: compiles every tests/t_*.c against all of narbis_core
# + support/ with mingw gcc and runs it. Auto-discovers suites so adding
# a test file needs no script edit. Run from anywhere.
set -u
cd "$(dirname "$0")"

GCC="${GCC:-/c/msys64/mingw64/bin/gcc.exe}"
CORE=../firmware/components/narbis_core
OUT=out
mkdir -p "$OUT" "$OUT/fixtures"

CFLAGS="-std=c11 -Wall -Wextra -Werror -g -I$CORE/include -Isupport"
SRCS="$CORE/src/*.c support/*.c"

fails=0
total=0
for t in tests/t_*.c; do
    name=$(basename "$t" .c)
    total=$((total + 1))
    if ! $GCC $CFLAGS $t $SRCS -o "$OUT/$name.exe" -lm 2> "$OUT/$name.buildlog"; then
        echo "BUILD FAIL $name (see $OUT/$name.buildlog)"
        fails=$((fails + 1))
        continue
    fi
    if ! "$OUT/$name.exe"; then
        fails=$((fails + 1))
    fi
done

echo "----"
echo "suites: $total, failing: $fails"
exit $((fails > 0))
