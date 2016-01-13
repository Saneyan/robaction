#include <stdio.h>
#include <stdlib.h>
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

struct Roba_status {

  // Current position.
  double current_x;
  double current_y;
  double current_r;

  // Using when poling or not.
  int targeted;

  // Missing count.
  int misscount;

  // First position.
  double x1;
  double y1;
};

void Roba_ctrlc(int notused);
void Roba_rotate_90();
void Roba_observe_pos(struct Roba_status *status);
void Roba_reset_pos(struct Roba_status *status);
void Roba_inc_misscount(struct Roba_status *status);
void Roba_reset_misscount(struct Roba_status *status);
void Roba_reset_tgt(struct Roba_status *status);
void Roba_end_tgt(struct Roba_status *status);
void Roba_start_tgt(struct Roba_status *status);
void Roba_reset_all(struct Roba_status *status);
void Roba_save_pos(struct Roba_status *status);

int escape;

int main(int argc, char *argv[])
{
  S2Port *port;    // Port
  S2Sdd_t buf;     // Double-buffer for getting data
  S2Scan_t *scan;  // Structure for reading data
  S2Param_t param; // Structure for censor parameter
  struct Roba_status *status = (struct Roba_status*)malloc(sizeof(struct Roba_status));

  Roba_reset_all(status);

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
  signal(SIGINT, Roba_ctrlc);
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

  // Starting running.
  Spur_line_LC(0, 0, 0);

  while (!escape) {

    ret = S2Sdd_Begin(&buf, &scan);

    if (ret > 0) { // Process when a new data is existed.

      // Observing position.
      Roba_observe_pos(status);

      // Getting each side of the distance.
      unsigned long frontd = scan->data[param.step_front - param.step_min];
      unsigned long rightd = scan->data[param.step_front - param.step_min + param.step_resolution / 4];

      // Print each side of the distance.
      printf("Front distance: %lu mm\n", frontd);
      printf("Right distance: %lu mm\n", rightd);
      printf("Current X: %f\n", status->current_x);
      printf("Current Y: %f\n", status->current_y);
      printf("Current R: %f\n", status->current_r);

      // When the front distance is less than or equal to 50cm, the robot rotates 90 degrees to avoid to
      // collision an obstacle.
      if (frontd < 500 && frontd != 0) {
        printf("Rotate 90 before frontd = 500");

        Roba_rotate_90();
        Roba_reset_all(status);

      // When the robot cannot compute right distance, increment miss count. If the robot mistakes to compute
      // more than 3 times, it rotates 90 degrees to try to catch another obstacles. The miss count is reseted
      // when it can compute right distance or after rotating.
      } else if (rightd == 0) {
        printf("Miss count: %d", status->misscount);

        if (status->misscount > 2) {
          Roba_rotate_90();
          Roba_reset_all(status);
        } else {
          Roba_reset_pos(status);
          Roba_reset_tgt(status);
          Roba_inc_misscount(status);
        }

        // Nop
        usleep(1000000);

      // When the robot can compute right distance, computes 2 point difference and degree and rotates itself.
      } else {
        Roba_reset_misscount(status);

        if (!status->targeted) {
          printf("Start TGT\n");

          Roba_start_tgt(status);
          Roba_save_pos(status);
          status->x1 = status->current_x;
          status->y1 = status->current_y;

          // Nop
          usleep(1000000);
        } else {
          printf("Calclating TGT\n");

          double diff_x = status->current_x - status->x1;
          double diff_y = status->current_y - status->y1;
          double r = atan2(diff_y, diff_x);

          printf("X: %f\n", diff_x);
          printf("Y: %f\n", diff_y);
          printf("result: %f\n", r);

          if (r > 0.1) {
            printf("End TGT\n");

            Roba_end_tgt(status);

            Spur_spin_FS(-r);

            Roba_reset_all(status);
          } else {
            Spur_line_FS(0, 0, 0);
          }
        }
      }

      // S2Sdd_BeginとS2Sdd_Endの間でのみ、構造体scanの中身にアクセス可能
      S2Sdd_End(&buf);
    } else if (ret == -1) {
      // 致命的なエラー時(URGのケーブルが外れたときなど)
      fprintf(stderr, "ERROR: Fatal error occurred.\n");
      break;
    } else {
      // 新しいデータはまだ無い
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

// If terminating this program with using MD command, it will cost additional time for the next starting.
void Roba_ctrlc(int notused)
{
  escape = 1;
  signal(SIGINT, NULL);
}

void Roba_rotate_90()
{
  Spur_spin_FS(-M_PI / 2);
  //Spur_line_FS(0.0, 0.0, 0);
}

void Roba_observe_pos(struct Roba_status *status)
{
  // Getting current position of the robot.
  Spur_get_pos_GL(&status->current_x, &status->current_y, &status->current_r);
  status->current_x *= 100;
  status->current_y *= 100;
  status->current_r *= 100;
}

void Roba_reset_pos(struct Roba_status *status)
{
  status->current_x = 0;
  status->current_y = 0;
  status->current_r = 0;
  status->x1 = 0.0;
  status->y1 = 0.0;
}

void Roba_reset_misscount(struct Roba_status *status)
{
  status->misscount = 0;
}

void Roba_inc_misscount(struct Roba_status *status)
{
  status->misscount++;
}

void Roba_reset_tgt(struct Roba_status *status)
{
  status->targeted = 0;
}

void Roba_end_tgt(struct Roba_status *status)
{
  Roba_reset_tgt(status);
}

void Roba_start_tgt(struct Roba_status *status)
{
  status->targeted = 1;
}

void Roba_reset_all(struct Roba_status *status)
{
  Roba_reset_pos(status);
  Roba_reset_misscount(status);
  Roba_reset_tgt(status);
}

void Roba_save_pos(struct Roba_status *status)
{
  status->x1 = status->current_x;
  status->y1 = status->current_y;
}
