# Host-side self-test for the benchmark core (runs on your PC, not the board).
#   make test    build and run the data-integrity test
#   make clean
CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11 -DDDR_BENCH_HOST -I../src
SRC      = host_test.c ../src/ddr_bench.c

host_test: $(SRC) ../src/ddr_bench.h
	$(CC) $(CFLAGS) -o $@ $(SRC)

test: host_test
	./host_test

clean:
	rm -f host_test

.PHONY: test clean
