#
# Makefile for dist action
#
# Author: Saneyuki TADOKORO <s1311374@u.tsukuba.ac.jp>
#

CC := gcc

all: dist

dist: dist.c
	$(CC) $< -o $@ -lscip2awd -lypspur -lpthread -lm

clean:
	rm -rf ./dist

.PHONY: all clean
