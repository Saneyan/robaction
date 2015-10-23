#include <stdio.h>
#include <ypspur.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include <scip2awd.h>

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
  Spur_set_vel(0.1);

  // Setting up acceleration.
  Spur_set_accel(1.0);

  // Setting up angular velocity.
  Spur_set_angvel(1.5);

  // Setting up angular acceleration.
  Spur_set_angaccel(2.0);

  // Setting up starting point.
  Spur_set_pos_GL(0, 0, 0);

  int trying_x = 0;
  int trying_y = 0;
  float target_y1 = 0;
  float target_y2 = 0;

  while (!escape) {

    // Starting running.
    //Spur_line_FS(0, 0, 0);

    ret = S2Sdd_Begin(&buf, &scan);

    if(ret > 0){

      int j;

      // 新しいデータがあった時の処理をここで行う
      printf("Front distance: %lu mm\n",
          scan->data[ param.step_front - param.step_min ] );
      printf("Left distance: %lu mm\n",
          scan->data[ param.step_front - param.step_min - param.step_resolution / 4]);
      printf("Right distance: %lu mm\n",
          scan->data[ param.step_front - param.step_min + param.step_resolution / 4]);

      // 処理例:スキャンしたデータをxy座標(m単位)に変換
      for (j = 0; j < scan->size; j ++) {
        float x, y;
        float scan_theta;

        // scan->data[j]はmm単位の距離を表し、
        // 20mm以下の距離は測距エラーを意味する
        if (scan->data[j] < 20) continue;

        scan_theta = M_PI * 2.0 * ( j - ( param.step_front - param.step_min ) ) / param.step_resolution;

        // URGが上下逆についている場合は、scan_theta = -scan_theta;
        x = scan->data[j] * 0.001 * cos(scan_theta);
        y = scan->data[j] * 0.001 * sin(scan_theta);

        // このx,yやscan->data[]の値を上手く使って処理を行う
        //printf("(X, Y) = (%f, %f)\n", x, y);

        //usleep(10000);

        if (x < 1.0) {
          if (++trying_x >= 10) {
            Spur_stop();
          }
        } else {
          trying_x = 0;
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
