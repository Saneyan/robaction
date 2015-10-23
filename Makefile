#
# Makefile for YP-Spur environment
#
# Author: Saneyuki TADOKORO <s1311374@u.tsukuba.ac.jp>
#

YP_SPUR_DIR = ./lib/yp-spur
YP_ROBOT_PARAMS_DIR = ./lib/yp-robot-params
LIBSCIP2AWD_DIR = ./lib/libscip2awd
ODOMETRY_DIR = ./lib/odometry
ODOMETRY_URL = http://www.roboken.iit.tsukuba.ac.jp/~ando-y/lectures/logging/odometry-logger-0.2.tar.gz

all: submodule make-yp-spur make-robot-params make-libscip2awd make-odometry

submodule:
	git submodule init
	git submodule update

make-yp-spur:
	cd $(YP_SPUR_DIR); ./configure && make

make-robot-params:
	cd $(YP_ROBOT_PARAMS_DIR); ./configure && make

make-libscip2awd:
	cd $(LIBSCIP2AWD_DIR); ./configure && make

make-odometry:
	curl $(ODOMETRY_URL) > lib/odometry.tar.gz
	mkdir -p $(ODOMETRY_DIR) && tar zxvf lib/odometry.tar.gz -C $(ODOMETRY_DIR) --strip-components 1
	cd $(ODOMETRY_DIR); make

clean:
	cd $(YP_SPUR_DIR); make clean
	cd $(YP_ROBOT_PARAMS_DIR); make clean
	cd $(LIBSCIP2AWD_DIR); make clean
	cd $(ODOMETRY_DIR); make clean
