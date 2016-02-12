#
# Makefile for odometry action
#
# Author: Saneyuki TADOKORO <s1311374@u.tsukuba.ac.jp>
#

CC := gcc

all: hug

hug: hug.c
	$(CC) $< -o $@ -lscip2awd -lypspur -lpthread -lm

clean:
	rm -rf ./hug.c

.PHONY: all clean
