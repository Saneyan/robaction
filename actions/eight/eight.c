#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ypspur.h>

#define VEL 0.1
#define ACCEL 1.0
#define ANGVEL 1.0
#define ANGACCEL 1.0
#define SP_X 0.0
#define SP_Y 0.0
#define SP_TH 0.0

void ctrlc(int aStatus)
{
  Spur_stop();
  signal(SIGINT, NULL);
  exit(aStatus);
}

int main(void)
{
  if (!Spur_init()) {
    fprintf(stderr, "ERROR : cannot open spur\n");
    return -1;
  }

  // Setting up signal to stops when entering Ctrl-C.
  signal(SIGINT, ctrlc);

  // Setting up velocity.
  Spur_set_vel(VEL);

  // Setting up acceleration.
  Spur_set_accel(ACCEL);

  // Setting up angular velocity.
  Spur_set_angvel(ANGVEL);

  // Setting up angular acceleration.
  Spur_set_angaccel(ANGACCEL);

  // Setting up starting point.
  Spur_set_pos_GL(SP_X, SP_Y, SP_TH);
  
  Spur_circle_GL(0.1, 0.1, 0.1);
  while (!Spur_near_ang_GL(3.14, 0.1))
    usleep(10000);

  Spur_circle_GL(0.1, 0.1, -0.1);
  while (!Spur_near_ang_GL(0.0, 0.1))
    usleep(10000);

  Spur_circle_GL(0.1, 0.1, 0.1);
  while (!Spur_near_ang_GL(-3.14, 0.1))
    usleep(10000);

  // Stop!
  Spur_stop();

  return 0;
}
