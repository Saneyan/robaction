#!/bin/bash

set -e

ODOMETRY_VERSION="0.2"
ODOMETRY_URL="http://www.roboken.iit.tsukuba.ac.jp/~ando-y/lectures/logging/odometry-logger-#{ODOMETRY_VERSION}.tar.gz"

if [ ${PARAM:-0} -eq 0 ]; then
  echo "The robot parameter name is required." 1>&2
  exit 1
fi

if [ ${SENSOR:-0} -eq 0 ]; then
  echo "The device file path of robot sensor is required." 1>&2
  exit 1
fi

if [ ${MICON:-0} -eq 0 ]; then
  echo "The device file path of micon is required." 1>&2
  exit 1
fi

if [ $USE_ODOMETRY ]; then
  if ! [ -e /opt/odometry ]; then
    cd /tmp
    curl $(ODOMETRY_URL) > odometry.tar.gz
    mkdir -p /opt/odometry && tar zxvf odometry.tar.gz -C /opt/odometry --strip-components 1
    cd /opt/odometry
    make
    ln -s /opt/odometry/odmlog /usr/local/bin/odmlog
  fi
fi

ypspur-coordinator -d $MICON -p $PARAM
