#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ypspur.h>

void ctrlC(int aStatus)
{
  Spur_stop();
  signal(SIGINT, NULL);
  exit(aStatus);
}

void setSigInt()
{
  signal(SIGINT, ctrlC);
}

int main(void)
{
  if (!Spur_init()) {
    fprintf(stderr, "ERROR : cannot open spur\n");
    return -1;
  }

  // Setting up signal to stops when entering Ctrl-C.
  setSigInt();

  // Setting up velocity.
  Spur_set_vel(0.3);

  // Setting up acceleration.
  Spur_set_accel(1.0);

  // Setting up angular velocity.
  Spur_set_angvel(1.5);

  // Setting up angular acceleration.
  Spur_set_angaccel(2.0);

  // Setting up starting point.
  Spur_set_pos_GL(0, 0, 0);
  

  // <Write here>

  // Stop!
  Spur_stop();

  return 0;
}
