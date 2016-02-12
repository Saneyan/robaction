/**
 * The final mission "Hansel und Gretel"
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
    int ret;
    int i;

    g_escape = 0;
    setctrlC(  );                               // シグナルハンドラの設定

    /**********************spur関連****************************/
    ret = Spur_init(  );                        // 初期化
    if( ret <= 0 )
    {
        fprintf( stderr, "ERROR: Failed to open yp-spur.\n" );
        return 0;
    }
    Spur_set_vel( 0.15 );                       // 最大速度0.15m/sに設定
    Spur_set_accel( 1.0 );                      // 加速度1.0m/sに設定
    Spur_set_angvel( 1.0 );                   // 最大角速度0.25rad/sに設定
    Spur_set_angaccel( 2.0 );                   // 各加速度2.0rad/ssに設定
    Spur_set_pos_GL( 0, 0, 0 );                 // スタート地点を原点にGL座標を設定

    /*********************************************************/
    Spur_free( );

    double x_gl, y_gl;                  // (m) GL座標系のURGのデータ（世界座標系）
    double pos_x_gl, pos_y_gl, pos_theta_gl; // (m) ,(rad) GL座標系のロボットの位置

    /***********************************************/

    while( !g_escape )
    {
      Spur_get_pos_GL( &pos_x_gl, &pos_y_gl, &pos_theta_gl );
      printf("%f, %f, %f\n", pos_x_gl * 100, pos_x_gl * 100, pos_theta_gl);
      usleep( 10000 );                     // 測域データに新しいデータがない
    }

    /************終了処理***************/
    // YP-Spurの停止 
    printf("OK");
    Spur_spin_GL(M_PI);
    sleep(3);
    Spur_line_GL( 0, 0, M_PI + pos_theta_gl );                  // スタート地点を原点にGL座標を設定
    while(1) {
      printf("%f, %f, %f\n", pos_x_gl * 100, pos_x_gl * 100, pos_theta_gl);
      Spur_get_pos_GL( &pos_x_gl, &pos_y_gl, &pos_theta_gl );
      if ((pos_x_gl * 100) < 1.0 && (pos_y_gl * 100) < 1.0) {
        break;
      }
      usleep( 10000 );
    }
    sleep(3);
    Spur_stop(  );

    // URGの終了
    fprintf( stderr, "Stopped\n" );

  return 0;
}

