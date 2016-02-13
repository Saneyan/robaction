/**
 * The final mission "Hansel and Gretel"
 *
 * @author Saneyuki TADOKORO <s1311374@coins.tsukuba.ac.jp>
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <ypspur.h>
#include <scip2awd.h>

int g_escape;

void ctrlC() {
  signal(SIGINT, NULL);
  g_escape = 1;
}

void setctrlC() {
  signal(SIGINT, ctrlC);
}

int main(int aArgc, char **aArgv) {
    int i, ret;

    g_escape = 0;
    setctrlC();
    ret = Spur_init();

    if(ret <= 0) {
      fprintf( stderr, "ERROR: Failed to open yp-spur.\n" );
      return 0;
    }

    Spur_set_vel(0.15);
    Spur_set_accel(1.0);
    Spur_set_angvel(1.0);
    Spur_set_angaccel(2.0);
    Spur_set_pos_GL(0, 0, 0);

    Spur_free( );

    double pos_x_gl, pos_y_gl, pos_theta_gl;

    while(!g_escape) {
      Spur_get_pos_GL( &pos_x_gl, &pos_y_gl, &pos_theta_gl );
      printf("%f, %f, %f\n", pos_x_gl * 100, pos_x_gl * 100, pos_theta_gl);
      usleep(10000);
    }

    // Start returning
    Spur_spin_GL(M_PI);

    sleep(3);
    Spur_line_GL(0, 0, M_PI + pos_theta_gl);

    while(1) {
      printf("%f, %f, %f\n", pos_x_gl * 100, pos_x_gl * 100, pos_theta_gl);
      Spur_get_pos_GL( &pos_x_gl, &pos_y_gl, &pos_theta_gl );
      if ((pos_x_gl * 100) < 1.0 && (pos_y_gl * 100) < 1.0) break;
      usleep(10000);
    }

    sleep(3);
    Spur_stop();

    fprintf(stderr, "Stopped\n");

  return 0;
}

