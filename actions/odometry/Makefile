#
# Makefile for odometry action
#
# Author: Saneyuki TADOKORO <s1311374@u.tsukuba.ac.jp>
#

CC := gcc

all: odometry

odometry: odometry.c
	$(CC) $< -o $@ -lscip2awd -lypspur -lpthread -lm

clean:
	rm -rf ./odometry.c

.PHONY: all clean
