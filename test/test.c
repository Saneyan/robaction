#include <ypspur.h>

int main(void)
{
  Spur_init();
  Spur_set_vel(0.3);
  Spur_set_accel(1.0);
  Spur_set_angvel(1.5);
  Spur_set_angaccel(2.0);

  Spur_set_pos_GL(0, 0, 0);

  Spur_line_GL(0, 0, 0);
  sleep(1);
  Spur_spin_GL(3.14/2);
  sleep(1);
  Spur_line_GL(1.0, 0, 3.14/2);
  sleep(1);
  Spur_stop();

  return 0;
}
