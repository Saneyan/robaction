#include <stdio.h>
#include <ypspur.h>
#include <unistd.h>
#include <time.h>
#include <sys/timeb.h>
#include <signal.h>
#include <math.h>
#include <scip2awd.h>

#define VEL 0.10
#define ACCEL 1.0
#define ANGVEL 1.0
#define ANGACCEL 1.0
#define SP_X 0.0
#define SP_Y 0.0
#define SP_TH 0.0

int escape;

// If terminating this program with using MD command, it will cost additional time for the next starting.
void ctrlc(int notused)
{
  escape = 1;
  signal(SIGINT, NULL);
}

int main(int argc, char *argv[])
{
  S2Port *port;    // Port
  S2Sdd_t buf;     // Double-buffer for getting data
  S2Scan_t *scan;  // Structure for reading data
  S2Param_t param; // Structure for censor parameter

  int ret;

  if (argc != 2) {
    fprintf(stderr, "USAGE: %s device\n", argv[0]);
    return 0;
  }

  // Open the port.
  port = Scip2_Open( argv[1], B0 );

  if (port == 0) {
    fprintf(stderr, "ERROR: Failed to open device.\n");
    return 0;
  }

  printf("Port opened\n");

  // Initialize.
  escape = 0;
  signal(SIGINT, ctrlc);
  S2Sdd_Init(&buf);
  printf("Buffer initialized\n");

  // Get URL parameter.
  Scip2CMD_PP(port, &param);

  // Start getting all directions data of URG-04LX.
  Scip2CMD_StartMS(port, param.step_min, param.step_max, 1, 0, 0, &buf, SCIP2_ENC_3BYTE);

  Spur_init();

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

  float target_y = 0;
  int targeted = 0;
  int adjusted = 0;

  // Starting running.
  Spur_line_GL(0, 0, 0);

  double   elapsed_time;
  int sec, millisec;
  struct timeb timebuffer;

  while (!escape) {

    ret = S2Sdd_Begin(&buf, &scan);

    if(ret > 0){

      int j;

      printf("Front distance: %lu mm\n",
          scan->data[ param.step_front - param.step_min ] );
      printf("Left distance: %lu mm\n",
          scan->data[ param.step_front - param.step_min - param.step_resolution / 4]);
      printf("Right distance: %lu mm\n",
          scan->data[ param.step_front - param.step_min + param.step_resolution / 4]);

      unsigned long frontd = scan->data[param.step_front - param.step_min];
      unsigned long leftd = scan->data[param.step_front - param.step_min - param.step_resolution / 4];
      unsigned long rightd = scan->data[param.step_front - param.step_min + param.step_resolution / 4];

      for (j = 0; j < scan->size; j ++) {
        float x, y;
        float scan_theta;

        if (scan->data[j] < 20) continue;

        scan_theta = M_PI * 2.0 * ( j - ( param.step_front - param.step_min ) ) / param.step_resolution;

        x = scan->data[j] * 0.001 * cos(scan_theta);
        y = scan->data[j] * 0.001 * sin(scan_theta);
      }

      if (frontd < 500 && !adjusted) {
        printf("adjusting");
        Spur_spin_GL(3.14 / 2);
        while (!Spur_near_ang_GL(3.14 / 2, 0.1));
          usleep(10000);
        Spur_set_pos_GL(SP_X, SP_Y, SP_TH);
        adjusted = 1;
      }

      if (!targeted) {
        targeted = 1;

        // Start
        ftime( &timebuffer );
        sec = timebuffer.time;
        millisec = timebuffer.millitm;

        target_y = rightd;
      } else {
        targeted = 0;

        // end
        ftime( &timebuffer );
        sec = timebuffer.time - sec;
        millisec = timebuffer.millitm - millisec;
        millisec += sec*1000;
        elapsed_time = (double)millisec/1000;

        float x = VEL * elapsed_time;
        float y = rightd - target_y;
        float r = y != 0 ? (180 / M_PI) * x / y : 0.0;

        printf("X: %f\n", x);
        printf("Y: %f\n", y);
        printf("result: %f\n", r);
        Spur_line_LC(1, 0, -r);

        usleep(100000);
      }

      S2Sdd_End(&buf);
    } else if (ret == -1) {
      fprintf(stderr, "ERROR: Fatal error occurred.\n");
      break;
    } else {
      // No new data found
      usleep(10000);
    }
  }
  printf("\nStopping\n");

  ret = Scip2CMD_StopMS( port, &buf );

  if (ret == 0) {
    fprintf(stderr, "ERROR: StopMS failed.\n");
    return 0;
  }

  printf("Stopped\n");
  S2Sdd_Dest(&buf);
  printf("Buffer destructed\n");
  Scip2_Close(port);
  printf("Port closed\n");
  Spur_stop();

  return 1;
}
