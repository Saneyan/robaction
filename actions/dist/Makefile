#
# Makefile for dist action
#
# Author: Saneyuki TADOKORO <s1311374@u.tsukuba.ac.jp>
#

CC := gcc

all: dist

dist: dist.c
	$(CC) -c $< -o $@

clean:
	rm -rf ./dist

.PHONY: all clean
