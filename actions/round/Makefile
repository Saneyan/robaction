#
# Makefile for round action
#
# Author: Saneyuki TADOKORO <s1311374@u.tsukuba.ac.jp>
#

CC := gcc

all: round

dist: round.c
	$(CC) $< -o $@ -lscip2awd -lypspur -lpthread -lm

clean:
	rm -rf ./round

.PHONY: all clean
