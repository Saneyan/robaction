#!/usr/bin/bash

BINFILE=./odometry
DATFILE=./odometry.dat
PNGFILE=./odometry.png

sudo $BINFILE $1 > $DATFILE

gnuplot -e "\
  set size ratio -1; \
  set xlabel 'y'; \
  set ylabel 'x'; \
  set term png; \
  set output '$PNGFILE'; \
  plot '$DATFILE'"

feh $PNGFILE
