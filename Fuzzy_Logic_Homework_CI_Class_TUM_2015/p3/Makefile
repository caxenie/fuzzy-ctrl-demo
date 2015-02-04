# Computational Intelligence Lab
# compile all / independent homework assignments

# setup the compiler to use
CC=gcc -Wall -static -std=c99 -pipe
# binaries
BINARIES=p3

# targets with dependencies and rules to compile

all: $(BINARIES)
	
%: %.c
	$(CC) -g -o $@ $< -lm

clean:
	rm -rf $(BINARIES)

