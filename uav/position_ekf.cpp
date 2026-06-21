// #include "position_ekf.h"
// #include <Arduino.h>
// #include <math.h>
// #include <cstring>
// PositionEKF posEKF;

// void PositionEKF::begin()
// {
//     x=0;
//     y=0;
//     vx=0;
//     vy=0;

//     bax=0;
//     bay=0;

//     memset(P,0,sizeof(P));

//     for(int i=0;i<6;i++)
//         P[i][i]=1.0f;
// }


// void PositionEKF::predict(
//         float ax_body,
//         float ay_body,
//         float yaw_deg,
//         float dt)
// {
//     //---------------------------------
//     // rotate body -> earth
//     //---------------------------------

//     float yaw=yaw_deg*DEG_TO_RAD;

//     float ax=
//         ax_body*cos(yaw)
//         -ay_body*sin(yaw);

//     float ay=
//         ax_body*sin(yaw)
//         +ay_body*cos(yaw);

//     //---------------------------------
//     // remove bias
//     //---------------------------------

//     ax-=bax;
//     ay-=bay;

//     //---------------------------------
//     // state prediction
//     //---------------------------------

//     vx+=ax*dt;
//     vy+=ay*dt;

//     x+=vx*dt;
//     y+=vy*dt;

//     //---------------------------------
//     // covariance
//     //---------------------------------

//     float q_pos=0.01f;
//     float q_vel=0.1f;
//     float q_bias=0.00001f;

//     P[0][0]+=q_pos;
//     P[1][1]+=q_pos;

//     P[2][2]+=q_vel;
//     P[3][3]+=q_vel;

//     P[4][4]+=q_bias;
//     P[5][5]+=q_bias;
// }


// void PositionEKF::updateGPS(
//         float gx,
//         float gy,
//         float gvx,
//         float gvy)
// {

//     //---------------------------------
//     // measurement noise
//     //---------------------------------

//     float Rpos=2.0f;
//     float Rvel=0.3f;

//     //---------------------------------
//     // residual
//     //---------------------------------

//     float ex=gx-x;
//     float ey=gy-y;

//     float evx=gvx-vx;
//     float evy=gvy-vy;

//     //---------------------------------
//     // gain
//     //---------------------------------

//     float Kx=P[0][0]/(P[0][0]+Rpos);
//     float Ky=P[1][1]/(P[1][1]+Rpos);

//     float Kvx=P[2][2]/(P[2][2]+Rvel);
//     float Kvy=P[3][3]/(P[3][3]+Rvel);

//     //---------------------------------
//     // state correction
//     //---------------------------------

//     x+=Kx*ex;
//     y+=Ky*ey;

//     vx+=Kvx*evx;
//     vy+=Kvy*evy;

//     //---------------------------------
//     // bias adaptation
//     //---------------------------------

//     bax-=0.001f*evx;
//     bay-=0.001f*evy;

//     //---------------------------------
//     // covariance update
//     //---------------------------------

//     P[0][0]*=(1-Kx);
//     P[1][1]*=(1-Ky);

//     P[2][2]*=(1-Kvx);
//     P[3][3]*=(1-Kvy);
// }
