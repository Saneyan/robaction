#
# Makefile for YP-Spur environment
#
# Author: Saneyuki TADOKORO <s1311374@u.tsukuba.ac.jp>
#

YP_SPUR_DIR = ./lib/yp-spur
YP_ROBOT_PARAMS_DIR = ./lib/yp-robot-params
LIBSCIP2AWD_DIR = ./lib/libscip2awd

all: submodule make-yp-spur make-robot-params make-libscip2awd

submodule:
	git submodule init
	git submodule update

make-yp-spur:
	cd $(YP_SPUR_DIR); ./configure && make

make-robot-params:
	cd $(YP_ROBOT_PARAMS_DIR); ./configure && make

make-libscip2awd:
	cd $(LIBSCIP2AWD_DIR); ./configure && make

clean:
	cd $(YP_SPUR_DIR); make clean
	cd $(YP_ROBOT_PARAMS_DIR); make clean
	cd $(LIBSCIP2AWD_DIR); make clean
