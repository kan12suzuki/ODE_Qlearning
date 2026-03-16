#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <ctime>
#include <random>
#include <fstream>
#include <map>


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#endif
#define NUM 4                          // リンク数
#define F "./Qtable_6p_14_no_draw/Sumrewards.csv"
#define F2 "./Qtable_6p_14_no_draw/ep_vs_st.csv"
#define F3 "./Qtable_6p_14_no_draw/Qtable.csv"
#define F4 "./Qtable_6p_14_no_draw/eval_Rewards.csv"
#define F5 "./Qtable_6p_14_no_draw/Qtable_episode" 
double t = 0.0;

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[NUM];              // ジョイントのID番号
dsFunctions   fn;                      // ドロースタッフの描画関数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

dReal lx = 0.25, ly = 0.25, lz = 0.25, m = 0.01;
typedef struct {
  dBodyID body;
  dGeomID geom;
  dReal lx, ly, lz, m;
} Box;
Box box[8];

MyObject rlink[NUM];                   // リンク

dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節
int           ANSWER = 1;              // 逆運動学の解
int ACTION = 0; //action番号

dReal P[3] = {0, 0, 3.0};             // 先端の位置
dReal a[3], b[3];                         // 有顔ベクトル(a,b)
dReal THETA[NUM] = {0.0, 0.0, 0.0, 0.0};  // 関節の目標角度[rad]
dReal l[4] = { 0.10, 0.90, 1.00, 1.00};   // リンクの長さ[m]
dVector3 target_pos;
dJointFeedback feedback[6];
std::vector<double> eval_rewards;

bool fast_mode = false;
int fast_mode_counter = 0;
const int FAST_MODE_DURATION = 100;
double STICK_PENALTY_LEARN = 2.0;
const double B_AVG_SPREAD_DIFF = 1e-5;
double STICK_PENALTY_EVAL = 6.0;



//ここより下は学習用パラメータ
dReal stepsize = 0.01;

//衝突検出
static void nearCallback(void *data, dGeomID o1, dGeomID o2){
  static const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1 || ground == o2));

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  //リンク同士の衝突は回避
  if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  if (n > 0){
    for (int i =0; i < n; i++){
      contact[i].surface.mode =   dContactSoftCFM | dContactSoftERP;
      contact[i].surface.mu = 0.16;
      if (!isGround){contact[i].surface.mu = 0.5; }
      // contact[i].surface.mu = 0.15;
      contact[i].surface.soft_cfm = 1e-7;
      contact[i].surface.soft_erp = 0.3;
      // contact[i].surface.bounce = 0.0;
      dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
      dJointAttach(c,dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));

      for (int j = 0; j < 6; j++){
        if ((o1 == box[j].geom && o2 == rlink[NUM-1].geom) ||
                      (o2 == box[j].geom && o1 == rlink[NUM-1].geom)){
          dJointSetFeedback(c, &feedback[j]);
          b1 = dGeomGetBody(o1);
          b2 = dGeomGetBody(o2);
        }
      }
      
      // if ((o1 == box[1].geom && o2 == box[0].geom) ||(o2 == box[1].geom && o1 == box[0].geom)
      //   ||(o1 == box[1].geom && o2 == box[2].geom) ||(o2 == box[1].geom && o1 == box[2].geom)
      //   ||(o1 == box[0].geom && o2 == box[2].geom) ||(o2 == box[0].geom && o1 == box[2].geom)){
      //   dJointSetFeedback(c, &jcollide);
    }
  }
}


void get_local_goal(dBodyID body, double bx, double by, double bz, double x, double y, double z, dVector3 goal)
{
  //bx, by, bz から粒を押す面を取得
  dVector3 pos;
  dBodyGetRelPointPos(body, bx, by, bz, pos);
  //絶対座標系のもとで移動量を加算する
  goal[0] = pos[0] + x;
  goal[1] = pos[1] + y;
  goal[2] = pos[2];
}


void catchball(){
  double *current_pos = (double *) dBodyGetPosition(sensor);
  double r = 0.04;
  switch(ACTION){
    case 0:
    target_pos[0] = 1.0;
    target_pos[1] = 1.0;
    target_pos[2] = 1.5;
    break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    dBodyGetRelPointPos(box[0].body, 0, 0.5*ly+0.5*r, 0.2*lz, target_pos); break;
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    dBodyGetRelPointPos(box[0].body, -0.5*lx-0.5*r, 0, 0.2*lz, target_pos); break;
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    dBodyGetRelPointPos(box[1].body, -0.5*lx-0.5*r, 0, 0.2*lz, target_pos); break;
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    dBodyGetRelPointPos(box[3].body, -0.5*lx-0.5*r, 0, 0.2*lz, target_pos); break;
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    dBodyGetRelPointPos(box[3].body, 0, -0.5*ly-0.5*r, 0.2*lz, target_pos); break;
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    dBodyGetRelPointPos(box[5].body, 0, -0.5*ly-0.5*r, 0.2*lz, target_pos); break;
    case 31:
    case 32:
    case 33:
    case 34:
    case 35:
    dBodyGetRelPointPos(box[5].body, 0.5*lx+0.5*r, 0, 0.2*lz, target_pos); break;
    case 36:
    case 37:
    case 38:
    case 39:
    case 40:
    dBodyGetRelPointPos(box[4].body, 0.5*lx+0.5*r, 0, 0.2*lz, target_pos); break;
    case 41:
    case 42:
    case 43:
    case 44:
    case 45:
    dBodyGetRelPointPos(box[3].body, 0.5*lx+0.5*r, 0, 0.2*lz, target_pos); break;
    case 46:
    case 47:
    case 48:
    case 49:
    case 50:
    dBodyGetRelPointPos(box[3].body, 0, 0.5*ly+0.5*r, 0.2*lz, target_pos); break;
  }
  double dist = sqrt(pow(current_pos[0] - target_pos[0], 2) +
                     pow(current_pos[1] - target_pos[1], 2) +
                     pow(current_pos[2] - target_pos[2], 2));

  if (!fast_mode && dist < 0.07 && ACTION!=0){
    fast_mode = true;
    fast_mode_counter = FAST_MODE_DURATION;
    // std::cout << "enter fast mode" << std::endl;
  }

  if (fast_mode){
    switch (ACTION)
    {
      //絶対座標系は右が＋ｘ、上が＋y
      //物体座標系は右が＋ｙ、上がーx
      case 1:
      get_local_goal(box[0].body, 0, 0.5*ly+0.5*r, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos); break;
      case 2:
      get_local_goal(box[0].body, 0, 0.5*ly+0.5*r, 0.2*lz, 0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 3:
      get_local_goal(box[0].body, 0, 0.5*ly+0.5*r, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 4:
      get_local_goal(box[0].body, 0, 0.5*ly+0.5*r, 0.2*lz, -0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 5:
      get_local_goal(box[0].body, 0, 0.5*ly+0.5*r, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos); break;
      case 6:
      get_local_goal(box[0].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 7:
      get_local_goal(box[0].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 8:
      get_local_goal(box[0].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos); break;
      case 9:
      get_local_goal(box[0].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 10:
      get_local_goal(box[0].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 11:
      get_local_goal(box[1].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 12:
      get_local_goal(box[1].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 13:
      get_local_goal(box[1].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos); break;
      case 14:
      get_local_goal(box[1].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 15:
      get_local_goal(box[1].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 16:
      get_local_goal(box[2].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 17:
      get_local_goal(box[2].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 18:
      get_local_goal(box[2].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos); break;
      case 19:
      get_local_goal(box[2].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 20:
      get_local_goal(box[2].body, -0.5*lx-0.5*r, 0, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 21:
      get_local_goal(box[2].body, 0, -0.5*ly-0.5*r, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos); break;
      case 22:
      get_local_goal(box[2].body, 0, -0.5*ly-0.5*r, 0.2*lz, -0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 23:
      get_local_goal(box[2].body, 0, -0.5*ly-0.5*r, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 24:
      get_local_goal(box[2].body, 0, -0.5*ly-0.5*r, 0.2*lz, 0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 25:
      get_local_goal(box[2].body, 0, -0.5*ly-0.5*r, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos); break;
      case 26:
      get_local_goal(box[5].body, 0, -0.5*ly-0.5*r, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos); break;
      case 27:
      get_local_goal(box[5].body, 0, -0.5*ly-0.5*r, 0.2*lz, -0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 28:
      get_local_goal(box[5].body, 0, -0.5*ly-0.5*r, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 29:
      get_local_goal(box[5].body, 0, -0.5*ly-0.5*r, 0.2*lz, 0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 30:
      get_local_goal(box[5].body, 0, -0.5*ly-0.5*r, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos); break;
      case 31:
      get_local_goal(box[5].body, 0.5*lx+0.5*r, 0, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 32:
      get_local_goal(box[5].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 33:
      get_local_goal(box[5].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos); break;
      case 34:
      get_local_goal(box[5].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 35:
      get_local_goal(box[5].body, 0.5*lx+0.5*r, 0, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 36:
      get_local_goal(box[4].body, 0.5*lx+0.5*r, 0, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 37:
      get_local_goal(box[4].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 38:
      get_local_goal(box[4].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos); break;
      case 39:
      get_local_goal(box[4].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 40:
      get_local_goal(box[4].body, 0.5*lx+0.5*r, 0, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 41:
      get_local_goal(box[3].body, 0.5*lx+0.5*r, 0, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 42:
      get_local_goal(box[3].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 43:
      get_local_goal(box[3].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos); break;
      case 44:
      get_local_goal(box[3].body, 0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0.5*ly, 0.2*lz, target_pos); break;
      case 45:
      get_local_goal(box[3].body, 0.5*lx+0.5*r, 0, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos); break;
      case 46:
      get_local_goal(box[3].body, 0, 0.5*ly+0.5*r, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos); break;
      case 47:
      get_local_goal(box[3].body, 0, 0.5*ly+0.5*r, 0.2*lz, 0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 48:
      get_local_goal(box[3].body, 0, 0.5*ly+0.5*r, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos); break;
      case 49:
      get_local_goal(box[3].body, 0, 0.5*ly+0.5*r, 0.2*lz, -0.5*lx, -0.5*ly, 0.2*lz, target_pos); break;
      case 50:
      get_local_goal(box[3].body, 0, 0.5*ly+0.5*r, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos); break;
    }
    fast_mode_counter--;
    if (fast_mode_counter <=0){
      fast_mode = false;
      // std::cout << "exit fast mode" << std::endl;
      // ACTION =0;
    }
  }

  

  double rate = fast_mode ? 0.025 : 0.008;

  for (int i = 0; i < 3; i++){
    P[i] += rate*(target_pos[i] - P[i]);
  }
}
void Pcontrol()
{
  dReal k =  10.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
		if (z >=   M_PI) z -= 2.0 * M_PI;
		if (z <= - M_PI) z += 2.0 * M_PI;
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}

void  inverseKinematics()
{
  double Px, Py, Pz;
  double distance = sqrt(P[0] * P[0] + P[1] * P[1]
    + (P[2] - (l[0] + l[1])) * (P[2] - (l[0] + l[1])));
  double maxReach = l[2] + l[3];
  double minReach = fabs(l[2] - l[3]);

  if (distance > maxReach || distance < minReach){
    // printf("Target  Position: x=%6.2f y=%6.2f z=%6.2f \n", P[0], P[1], P[2]);
    return;
  }

  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
 	// printf("Target  Position: x=%6.2f y=%6.2f z=%6.2f \n", Px, Py, Pz);

  double tmpL  = sqrt(Px * Px + Py * Py);
  double P1P   = sqrt(Px * Px + Py * Py
							 + (Pz - (l[0] + l[1])) * (Pz - (l[0] + l[1])));
  double Ca    = (l[2] * l[2] + P1P * P1P -l[3] * l[3])/(2 * l[2] * P1P);  // cosα

  double phi   = atan2(Pz - (l[0] + l[1]), tmpL);                      //φ
  double alpha = atan2(sqrt(1 - Ca * Ca), Ca);                         //α

  double Cb    = (l[2]*l[2] + l[3]*l[3] - P1P*P1P)/(2 * l[2] * l[3]);  //cosβ
  double beta  = atan2(sqrt(1- Cb * Cb), Cb);                          //β

  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
  case 1: // 姿勢１
    THETA[1] = atan2(Py, Px);
    THETA[2] = M_PI/2 - phi - alpha;
    THETA[3] = M_PI - beta; break;
  case 2: // 姿勢２
    THETA[1] = atan2(Py, Px);
    THETA[2] = M_PI/2 - phi + alpha;
    THETA[3] = M_PI + beta; break;
  case 3:  // 姿勢３  海老反り姿勢
    THETA[1] = atan2(Py, Px) + M_PI;
    THETA[2] = -(M_PI/2 - phi - alpha);
    THETA[3] = M_PI + beta; break;
  case 4:  // 姿勢４  海老反り姿勢
    THETA[1] = atan2(Py, Px) + M_PI;
    THETA[2] = -(M_PI/2 - phi + alpha);
    THETA[3] = M_PI - beta; break;
  }
}



class Stickness{
  private:
    double x_dst_lim;
    double y_dst_lim;
    double theta_lim;
    std::vector<double> state;
    int steps_beyond_terminated;
      
  public:
    Stickness(){
      x_dst_lim = 1.5 * lx;
      y_dst_lim = 1.5 * ly;
      theta_lim = M_PI /12;
      state.resize(12);
      steps_beyond_terminated = 0;
    }
    void setState(const std::vector<double>& new_state){
      state = new_state;
    }
  
    bool terminate(){
      // double x1 = state[0];
      // double x2 = state[2];
      // double x3 = state[4];
      // double x4 = state[6];
      // double x5 = state[8];
      // double x6 = state[10];
      // double y1 = state[1];
      // double y2 = state[3];
      // double y3 = state[5];
      // double y4 = state[7];
      // double y5 = state[9];
      // double y6 = state[11];
      int num_particles = state.size()/2;
      double threshold = x_dst_lim;
      double threshold_sq = threshold * threshold;

      for (int i = 0; i < num_particles; ++i){
        double x = state[2 * i];
        double y = state[2 * i +1];
        double range_sq = x * x + y * y;
        if (range_sq > 3.0 * 3.0){
          return true;
        }
      }

      bool all_separated = true;
      for (int i = 0; i < num_particles; ++i){
        for (int j = i +1; j < num_particles; ++j){
          double xi = state[2*i];
          double yi = state[2*i +1];
          double xj = state[2*j];
          double yj = state[2*j +1];
          
          double dx = xi - xj;
          double dy = yi - yj;
          double dist_sq = dx * dx + dy * dy;

        if (dist_sq < threshold_sq) {
            all_separated = false; // まだくっついている
            break; // この時点で「成功」ではないのでループを抜ける
          }
        }
        if (!all_separated) break; // 外側のループも抜ける
      }
      // 全部分離できた(true) なら終了
      if (all_separated) {
        return true;
      }

      // 失敗でも成功でもなければ、まだ継続 (false)
      return false;
    }

  dReal calculateDistance(dBodyID body1, dBodyID body2){
    const dReal*pos1 = dBodyGetPosition(body1);
    const dReal*pos2 = dBodyGetPosition(body2);
    
    dReal distance = sqrt(pow(pos2[0] - pos1[0], 2) 
                        + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));
    return distance;
  }
  int getBodyIndex(dBodyID body)
  {
      for (int i = 0; i < 8; i++) {
          if (box[i].body == body) return i;
      }
      return -1;  // 見つからない場合
  }
  double clamp(double x, double minVal, double maxVal) {
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
  }


  
  void calculatestickness(dBodyID body1, dBodyID body2) {//body1が上側、body2が下側の粒
    int i1 = getBodyIndex(body1);
    int i2 = getBodyIndex(body2);
      // std::cout << "straight " << i1 << "," << i2 << std::endl;
    const dReal *pos1 = dBodyGetPosition(body1);
    const dReal *pos2 = dBodyGetPosition(body2);
    const dReal *force1_raw = dBodyGetForce(body1);
    const dReal *force2_raw = dBodyGetForce(body2);
    const dReal *vel1 = dBodyGetLinearVel(body1);
    const dReal *vel2 = dBodyGetLinearVel(body2);
    double flim = 10.0;

    dReal relPos[3] = {
      pos2[0] - pos1[0],
      pos2[1] - pos1[1],
      pos2[2] - pos1[2]
    };

    // 合力 = 衝突フィードバック + 物体に働いている力
    dReal force1[3], force2[3];
    for (int i = 0; i < 3; i++) {
      // force1[i] = feedback[i1].f1[i] + force1_raw[i];
      // force2[i] = feedback[i2].f1[i] + force2_raw[i];
      force1[i] = clamp(feedback[i1].f1[i] + force1_raw[i], -2, 2);
      force2[i] = clamp(feedback[i2].f1[i] + force2_raw[i], -2, 2);

    }

    double dis = pos1[0] * pos1[0] + pos1[1] * pos1[1];
    if (dis > 3){
      // std::cout << "over range!!!!" << std::endl;
    }
    double dist_ = calculateDistance(body1,body2);

    // 距離条件
    if (dist_ <= 1.40*ly){
    // if (fabs(relPos[0]) <= lx*1.1 && fabs(relPos[1]) <= ly*1.1) {
      // 緑側のY方向の力が強い場合
      if (force2[1] > flim) {
        if (force2[0] > 0) {
        } else {
        }
      }

      if (force2[1] < -flim) {
        if (force2[0] > 0) {
          dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
          dBodyAddForceAtRelPos(body1, force2[0]/2 + relPos[0], force2[1]/2 + relPos[1] , 0, -0.5*lx, -0.5*ly, 0);
          // printf("green jab1\n");
        } else {
          dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
          dBodyAddForceAtRelPos(body1, force2[0]/2 + relPos[0], force2[1]/2 + relPos[1] , 0, -0.5*lx, -0.5*ly, 0);
          // printf("green jab2\n");
        }
      }

      // 青側（body1）からの力が大きいとき
      if (force1[1] > flim) {
        if (force1[0] > 0) {
          dBodyAddForceAtRelPos(body2, - relPos[0], force1[1]/2 - relPos[1] ,0, 0.5*lx, 0.5*ly, 0);
          dBodyAddForceAtRelPos(body2, force1[0]/2 - relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, 0.5*ly, 0);
          // printf("blue crash1\n");
        } else {
          dBodyAddForceAtRelPos(body2, force1[0]/2 - relPos[0], force1[1]/2 - relPos[1] , 0, 0.5*lx, 0.5*ly, 0);
          dBodyAddForceAtRelPos(body2, -relPos[0], force1[1]/2 - relPos[1] ,0, -0.5*lx, 0.5*ly, 0);
          // printf("blue crash2\n");
        }
      }
      if (force1[1] < -flim){
        // printf("wow\n");
      }

      dBodyAddForceAtRelPos(body1, relPos[0], relPos[1], 0, 0.5*lx, -0.5*ly, 0);
      dBodyAddForceAtRelPos(body1, relPos[0], relPos[1], 0, -0.5*lx, -0.5*ly, 0);
      dBodyAddForceAtRelPos(body2, -relPos[0], -relPos[1], 0, 0.5*lx, 0.5*ly, 0);
      dBodyAddForceAtRelPos(body2, -relPos[0], -relPos[1], 0, -0.5*lx, 0.5*ly, 0);  

      // 通常の引き寄せ
      if (force1[2] < -20){
        dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2+feedback[i1].f2[2]/2, 0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2+feedback[i1].f2[2]/2, -0.5*lx, 0.5*ly, 0);  
      } else if (force2[2] < -20){
        dBodyAddForceAtRelPos(body1, 0, 0, force2[2]/2+feedback[i2].f2[2]/2, 0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body1, 0, 0, force2[2]/2+feedback[i2].f2[2]/2, -0.5*lx, 0.5*ly, 0);
          // 押し付けられてる方は、アームから受ける力 - 反発力 してるはずだから、もう一方もそうする？？ 
      }
    } else {
      // printf("NO sticking\n");
    }
  }
  void calculatestickness_side(dBodyID body1, dBodyID body2) { //body1が左側、body2が右側のとき成立する
    int i1 = getBodyIndex(body1);
    int i2 = getBodyIndex(body2);
    const dReal *pos1 = dBodyGetPosition(body1);
    const dReal *pos2 = dBodyGetPosition(body2);
    const dReal *force1_raw = dBodyGetForce(body1);
    const dReal *force2_raw = dBodyGetForce(body2);
    double flim = 10.0;
    // std::cout << "side " << i1 << "," << i2 << std::endl;
    dReal relPos[3] = {
      pos2[0] - pos1[0],
      pos2[1] - pos1[1],
      pos2[2] - pos1[2]
    };


    // 合力 = 衝突フィードバック + 物体に働いている力
    dReal force1[3], force2[3];
    for (int i = 0; i < 3; i++) {
      force1[i] = clamp(feedback[i1].f1[i] + force1_raw[i], -2, 2);
      force2[i] = clamp(feedback[i2].f1[i] + force2_raw[i], -2, 2);
    }

    double dis = pos1[0] * pos1[0] + pos1[1] * pos1[1];
    if (dis > 3){
      // std::cout << "over range!!!!" << std::endl;
    }
    double dist_ = calculateDistance(body1,body2);


    // 距離条件
    if (dist_ <= 1.40*lx){
      if (force2[0] > flim){
        if(force2[1] > 0){
          dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, 0.5*ly, 0);
          dBodyAddForceAtRelPos(body1, force2[0]/2 + relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
          // printf("Mode change1\n");
        }else {
          dBodyAddForceAtRelPos(body1, force2[0]/2 +relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, 0.5*ly, 0);
          dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
          // printf("green jab1\n");
        }
      }

      if (force1[0] < -flim){
        if (force1[1] > 0){
          dBodyAddForceAtRelPos(body2, - relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, 0.5*ly, 0);
          dBodyAddForceAtRelPos(body2, force1[0]/2 -relPos[0], force1[1]/2 - relPos[1] ,0, -0.5*lx, -0.5*ly, 0);
          // printf("blue crash2\n");
        } else{
          dBodyAddForceAtRelPos(body2, force1[0]/2 -relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, 0.5*ly, 0);
          dBodyAddForceAtRelPos(body2, -relPos[0], force1[1]/2 - relPos[1] ,0, -0.5*lx, -0.5*ly, 0);
          // printf("wow2\n");
        }
      }

      dBodyAddForceAtRelPos(body1, relPos[0], relPos[1], 0, 0.5*lx, 0.5*ly, 0);
      dBodyAddForceAtRelPos(body1, relPos[0], relPos[1], 0, 0.5*lx, -0.5*ly, 0);
      dBodyAddForceAtRelPos(body2, -relPos[0], -relPos[1], 0, -0.5*lx, 0.5*ly, 0);
      dBodyAddForceAtRelPos(body2, -relPos[0], -relPos[1], 0, -0.5*lx, -0.5*ly, 0);  
        double damping = 0.5;
        dReal v1[3]; const dReal *vel1 = dBodyGetLinearVel(body1);
        dReal v2[3]; const dReal *vel2 = dBodyGetLinearVel(body2);
        dReal dampF1[3] = { -damping*vel1[0], -damping*vel1[1], -damping*vel1[2] };
        dReal dampF2[3] = { -damping*vel2[0], -damping*vel2[1], -damping*vel2[2] };
        dBodyAddForce(body1, dampF1[0], dampF1[1], dampF1[2]);
        dBodyAddForce(body2, dampF2[0], dampF2[1], dampF2[2]);


      // 通常の引き寄せ
      if (force1[2] < -20){
        // dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2, -0.5*lx, 0.5*ly, 0);
        // dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2, -0.5*lx, -0.5*ly, 0);  
        dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2+feedback[i1].f2[2]/2, -0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2+feedback[i1].f2[2]/2, -0.5*lx, -0.5*ly, 0);  
      } else if (force2[2] < -20){
        dBodyAddForceAtRelPos(body1, 0, 0, force2[2]/2+feedback[i2].f2[2]/2, 0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body1, 0, 0, force2[2]/2+feedback[i2].f2[2]/2, 0.5*lx, -0.5*ly, 0);
          // 押し付けられてる方は、アームから受ける力 - 反発力 してるはずだから、もう一方もそうする？？ 
      }
    } else {
      // printf("NO sticking\n");
    }
  }
  int judge_relation_local(int i, int j){
      // 粒 j の世界座標
      const dReal* pos_j = dBodyGetPosition(box[j].body);

      // 粒 j の位置を i のローカル座標に変換
      dVector3 p_j_local;
      dBodyGetPosRelPoint(box[i].body,
                          pos_j[0], pos_j[1], pos_j[2],
                          p_j_local);

      double lx = p_j_local[0];   // ローカルX（左右判定用）
      double ly = p_j_local[1];   // ローカルY（上下判定用）

      // 上下方向の判定（縦方向が強い場合）
      if (fabs(ly) >= fabs(lx)) {
        if (ly< 0)
          return 0;  // iがjより上にいる body == i
        else 
          return 1; ///iがjより下にいる
      }

      // 左右方向（横方向のほうが支配的）
      if (lx > 0)
          return 2;  // iが左側（i → j）
      else
          return 3;  // iが右側（j → i）
  }

  // これが大きいほど「粒が全体的に散らばっている」ことを意味します
  double calculate_total_spread(const std::vector<double>& obs) {
      double total_dist = 0.0;
      int num_particles = obs.size() / 2; // 12要素なら6粒

      for (int i = 0; i < num_particles; ++i) {
          for (int j = i + 1; j < num_particles; ++j) {
              double dx = obs[2 * i] - obs[2 * j];
              double dy = obs[2 * i + 1] - obs[2 * j + 1];
              total_dist += std::sqrt(dx * dx + dy * dy);
          }
      }
      return total_dist;
  }
  double calculate_separation_score(const std::vector<double>& obs) {
    int num_particles = obs.size() / 2;
    if (num_particles == 0) return 0.0;
    // Graded separation score: give partial credit when particles are somewhat separated.
    // full_threshold: distance considered fully separated
    // partial_threshold: distance considered partially separated
    double full_threshold = 1.5 * lx;
    double partial_threshold = 0.75 * lx;
    double score = 0.0;
    for (int i = 0; i < num_particles; ++i) {
      double minDist = 1e9;
      for (int j = 0; j < num_particles; ++j) {
        if (i == j) continue;
        double dx = obs[2 * i] - obs[2 * j];
        double dy = obs[2 * i + 1] - obs[2 * j + 1];
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < minDist) minDist = d;
      }
      if (minDist >= full_threshold) score += 1.0;
      else if (minDist <= partial_threshold) score += 0.0;
      else {
        // linear interpolation between partial -> 0 and full -> 1
        double frac = (minDist - partial_threshold) / (full_threshold - partial_threshold);
        score += frac;
      }
    }
    return score / static_cast<double>(num_particles);
  }

  double calculate_nearest_distance_variance(const std::vector<double>& obs) {
    int num_particles = obs.size() / 2;
    if (num_particles <= 1) return 0.0;
    std::vector<double> min_dists(num_particles, 1e9);
    for (int i = 0; i < num_particles; ++i) {
      for (int j = 0; j < num_particles; ++j) {
        if (i == j) continue;
        double dx = obs[2*i] - obs[2*j];
        double dy = obs[2*i + 1] - obs[2*j + 1];
        double d = std::sqrt(dx*dx + dy*dy);
        if (d < min_dists[i]) min_dists[i] = d;
      }
    }
    double mean = 0.0;
    for (double v : min_dists) mean += v;
    mean /= static_cast<double>(num_particles);
    double var = 0.0;
    for (double v : min_dists) var += (v - mean)*(v - mean);
    var /= static_cast<double>(num_particles);
    return var;
  }



  double step(int action){
    std::vector<dBodyID> target_bodies = {box[0].body, box[1].body,box[2].body, box[3].body,box[4].body, box[5].body};
    // double delta_theta = state[4];
    ACTION = action;
    
    dSpaceCollide(space, 0, &nearCallback);
    catchball();
  for (int i =0; i < 6; i++){
    for (int j = i+1; j <6; j ++){
      if (i ==j){continue;}
      int t = judge_relation_local(i, j);
      if (t == 0){
        calculatestickness(box[i].body, box[j].body);
      }else if(t == 1) {
        calculatestickness(box[j].body, box[i].body);
      }else if( t ==2){
        calculatestickness_side(box[i].body, box[j].body);
      }else {
        calculatestickness_side(box[j].body, box[i].body);
      }
    }
  }
    inverseKinematics();
    Pcontrol();
    dWorldQuickStep(world, stepsize);
    dJointGroupEmpty(contactgroup);

    const dReal *sensor_pos = dBodyGetPosition(sensor);
    state.clear();

    for (const auto& body : target_bodies){
      const dReal *pos = dBodyGetPosition(body);
      double rel_x =  pos[0] - sensor_pos[0];
      double rel_y =  pos[1] - sensor_pos[1];
      
      state.push_back(rel_x);
      state.push_back(rel_y);
    }    
    bool terminated = terminate();
    double reward;

    if (terminated){
      if (steps_beyond_terminated == 0){
        std::cerr << "You are calling 'step()' even though this environment has already returned terminated = True. You should always call 'reset()' once you receive 'terminated' = True -- any further steps are undefined behavior." << std::endl;
      }
      steps_beyond_terminated += 1;
      reward = 0;
    }
    return reward;
  }

  std::vector<double>getcurrentState(){
    return state;
  }

  std::vector<double>reset(){
    double x = (rand() / (RAND_MAX +1.0)) * 0.5;
    double y = 1 + (rand() / (RAND_MAX +1.0)) * 0.5;
    double z = lz/2 + 0.05 +(rand() / (RAND_MAX +1.0)) * 0.15;
    state[0] = x;
    state[1] = y;
    state[2] = x;
    state[3] = y - 0.25;
    state[4] = x;
    state[5] = y -0.50;
    state[6] = x +0.25;
    state[7] = y;
    state[8] = x + 0.25;
    state[9] = y - 0.25;
    state[10] = x + 0.25;
    state[11] = y - 0.50;

    steps_beyond_terminated = 0.0;
    return state;
  }

  std::vector<double>eval_reset(){
    double x = 0;
    double y = 1;
    double z = lz/2 + 0.02;
    state[0] = x;
    state[1] = y;
    state[2] = x;
    state[3] = y - 0.25;
    state[4] = x;
    state[5] = y -0.50;
    state[6] = x +0.25;
    state[7] = y;
    state[8] = x + 0.25;
    state[9] = y - 0.25;
    state[10] = x + 0.25;
    state[11] = y - 0.50;

    steps_beyond_terminated = 0.0;
    return state;
  }
};
  

// センサの作成
void makeSensor()
{
  dMass mass;
  double sx = 0.0, sy = 0.0, sz = 3.0;  // センサの初期座標[m]
  double size = 0.01, weight = 0.00001; // センサのサイズ[m]と重量[kg]

  sensor = dBodyCreate(world);          // センサの生成
  dBodySetPosition(sensor,sx,sy,sz);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,weight,size,size,size);
  dBodySetMass(sensor, &mass);

  sensor_joint = dJointCreateFixed(world, 0); // 固定ジョイントの生成
  dJointAttach(sensor_joint, rlink[NUM-1].body, sensor); // 先端リンクと結合
  dJointSetFixed(sensor_joint);
}

void makebox(int num, double x = 0.0, double y = 1.0, double z = lz/2 + 0.005)
{
  // double x = 0.0, y = 1.0, z = 0.54;
  dMass mass;

  box[num].body = dBodyCreate(world);
  dBodySetPosition(box[num].body, x, y, z);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, 0.05, lx, ly, lz);
  dBodySetMass(box[num].body, &mass);
  box[num].lx = lx;
  box[num].ly = ly;
  box[num].lz = lz;
  box[num].geom = dCreateBox(space, box[num].lx,box[num].ly,box[num].lz);
  dGeomSetBody(box[num].geom, box[num].body);
}

/*** ロボットアームの生成 ***/
void  makeArm()
{
  double a = 0;
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.05+a, 0.50+a, 1.50+a, 2.50+a};  // 重心 z
  dReal length[NUM] = {0.10, 0.90, 1.00, 1.00};  // 長さ
  dReal weight[NUM] = {9.00, 2.00, 2.00, 2.00};  // 質量
  dReal r[NUM]      = {0.20, 0.04, 0.04, 0.04};  // 半径
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00+a, 0.10+a, 1.00+a, 2.00+a};  // 関節中心点 z
  dReal axis_x[NUM] = {0, 0, 0, 0};              // 関節回転軸 x
  dReal axis_y[NUM] = {0, 0, 1, 1};              // 関節回転軸 y
  dReal axis_z[NUM] = {1, 1, 0, 0};              // 関節回転軸 z

  // リンクの生成

  rlink[0].body = dBodyCreate(world);
  dBodySetPosition(rlink[0].body, x[0], y[0], z[0]);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass, weight[0], 3, r[0], length[0]);
  dBodySetMass(rlink[0].body, &mass);
  rlink[0].geom = dCreateCylinder(space, r[0], length[0]);
  dGeomSetBody(rlink[0].geom, rlink[0].body);
  
  for (int i = 1; i < NUM; i++) {
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,weight[i],3,r[i],length[i]);
    dBodySetMass(rlink[i].body, &mass);
    rlink[i].geom = dCreateCapsule(space,r[i],length[i]);
    dGeomSetBody(rlink[i].geom,rlink[i].body);
  }

  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], rlink[0].body, 0);
  dJointSetFixed(joint[0]);
  for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
  }
}

class QTable {
    int num_actions;
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist;
public:
    std::map<int, std::vector<double>> table;
  QTable(int actions) : num_actions(actions), rng(std::time(0)), dist(-1.0, 1.0) {
      }

  std::vector<double>& checkState(int state) {
        // キーが見つからない場合
        if (table.find(state) == table.end()) {
            std::vector<double> new_q(num_actions);
            // 初期値をランダムに設定 (-1.0 ~ 1.0)
            for (int i = 0; i < num_actions; ++i) {
                new_q[i] = dist(rng);
            }
            table[state] = new_q;
        }
        // 参照を返す（これで table[state] と同じように使えます）
        return table[state];
    }

  int argmax(const std::vector<double>& values){
    return std::distance(values.begin(), std::max_element(values.begin(), values.end()));
  }  

  // int getAction(const std::vector<double>& state, int episode){
  //   int nextAction;
  //   double epsilon = 0.5*std::pow(0.99, episode);
  //   if (epsilon <= (rand() / static_cast<double>(RAND_MAX))){
  //     nextAction = argmax(table[digitizeState(state)]);
  //   } else {
  //     nextAction = rand() % table[0].size();
  //     // std::cout<<"nextAction = " << nextAction << std::endl;
  //   }
  //   return nextAction;
  // }
  int getAction(const std::vector<double>& state, int episode) {
        int nextAction;
        // epsilonの減少ロジック
        double epsilon = 0.5 * std::pow(0.99, episode); 
        // ★修正: table[digitizeState(state)] を checkState(...) に変更
        // これで「初めて見る状態」でもエラーになりません
        int state_idx = digitizeState(state);
        if (epsilon <= (rand() / static_cast<double>(RAND_MAX))) {
            // Greedy: Q値が最大の行動を選ぶ
            nextAction = argmax(checkState(state_idx));
        } else {
            // Random: 行動数の中からランダム
            // ★修正: table[0].size() だと table[0]がない場合に落ちるので num_actions を使用
            nextAction = rand() % num_actions;
        }
        return nextAction;
    }
  // int greedyAction(const std::vector<double>& state){
  //   int nextAction = argmax(table[digitizeState(state)]);
  //   return nextAction;
  // }
  int greedyAction(const std::vector<double>& state) {
        // ★修正: ここも checkState を経由させる
        return argmax(checkState(digitizeState(state)));
  }

  // void updateQtable(int state, int action, int nextState, int nextAction, double reward, double alpha, double ganma){
  //   table[state][action] = (1 - alpha) * table[state][action] + 
  //                             alpha * (reward + ganma * table[nextState][nextAction]);
  // }

  void updateQtable(int state, int action, int nextState, int nextAction, double reward, double alpha, double ganma) {
        // ★修正: 現在の状態と次の状態、両方について存在チェックを行う
        std::vector<double>& q_curr = checkState(state);
        std::vector<double>& q_next = checkState(nextState);

        // 更新式
        q_curr[action] = (1 - alpha) * q_curr[action] +
                         alpha * (reward + ganma * q_next[nextAction]);
  }

  int digitizeState(const std::vector<double>& state){
    auto bins = [](double values, double clip_min, double clip_max, int num_bins){
      if (values < clip_min){
        return 0;
      } else if (values >= clip_max){
        return num_bins -1;
      } else {
        double bin_width = (clip_max - clip_min) / num_bins;
        return static_cast<int>((values - clip_min) / bin_width);
      }
    };

    int num_bins = 4;
    int NUM_state = state.size();
    long long result = 0;
    long long weight = 1;
    
    for (int i = 0; i < NUM_state; i++){
      int d_val;
      if (i%2 == 0){
        d_val = bins(state[i], -1.0, 1.0, num_bins);
      } else {
        d_val = bins(state[i], 0, 2.0, num_bins);
      }
      result += d_val * weight;
      weight *= num_bins;
    }
    return static_cast<int>(result);
  }
};

double calculateMean(const std::vector<double>& data){
  double sum = 0.0;
  for (const double& value : data){
    sum += value;
  }
  double mean = 0.0;
  if (!data.empty()){
    mean = sum / data.size();
  }
  return mean;
}

static void destroybox(){
  dBodyDestroy(box[0].body);
  dGeomDestroy(box[0].geom);
  dBodyDestroy(box[1].body);
  dGeomDestroy(box[1].geom);
  dBodyDestroy(box[2].body);
  dGeomDestroy(box[2].geom);
  dBodyDestroy(box[3].body);
  dGeomDestroy(box[3].geom);
  dBodyDestroy(box[4].body);
  dGeomDestroy(box[4].geom);
  dBodyDestroy(box[5].body);
  dGeomDestroy(box[5].geom);
}

static void destroyarm(){
  for (int i = 0; i < NUM; i++){
    dJointDestroy(joint[i]);
    dBodyDestroy(rlink[i].body);
    dGeomDestroy(rlink[i].geom);
  }
  dJointDestroy(sensor_joint);
  dBodyDestroy(sensor);
}


double calculate_delta_theta(dBodyID body1, dBodyID body2){
  const dReal *R1 = dBodyGetRotation(body1);
  const dReal *R2 = dBodyGetRotation(body2);
  double theta1 = std::atan2(R1[4], R1[0]);
  double theta2 = std::atan2(R2[4], R2[0]);

  double delta_theta = abs(theta2 - theta1);
  return delta_theta;
}


int main(int argc, char *argv[]) 
{
  const int num_states = 4*4*4*4*4*4*4*4*4*4*4*4;
  const int num_actions = 50;
  dInitODE(); //ＯＤＥの初期化
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  dWorldSetERP(world, 1.0);
  dWorldSetCFM(world, 1e-5);
  ground = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成

  Stickness env;
  QTable qTable(num_actions);
  const int max_number_of_steps = 5000;
  const int num_consecutive_iterations = 100;
  const double alpha = 0.2;
  const double ganma = 0.99;
  std::vector<double> last_time_steps(num_consecutive_iterations, 0);
  std::vector<double> flags(num_consecutive_iterations, 0);
  std::vector<int> step_list;
  int episode =0;
  int success_flag =0;
  int final_flag = 0;
  double sumReward = 0.0;
  double eval_sumReward = 0.0;
  int nextAction;
  const int interval_episodes = 100;

  while(true){
    episode ++;
    contactgroup = dJointGroupCreate(0);
    std::vector<double> observation = env.reset();
    makeArm();
    makeSensor();
    const dReal *sensor_pos = dBodyGetPosition(sensor);
    double z = lz/2 + 0.02 +(rand() / (RAND_MAX +1.0)) * 0.15;
    makebox(0, observation[0]+sensor_pos[0], observation[1]+sensor_pos[1], z);
    makebox(1, observation[2]+sensor_pos[0], observation[3]+sensor_pos[1], z);
    makebox(2, observation[4]+sensor_pos[0], observation[5]+sensor_pos[1], z);
    makebox(3, observation[6]+sensor_pos[0], observation[7]+sensor_pos[1], z);
    makebox(4, observation[8]+sensor_pos[0], observation[9]+sensor_pos[1], z);
    makebox(5, observation[10]+sensor_pos[0], observation[11]+sensor_pos[1], z);
    P[0] = 0.0;
    P[1] = 0.0;
    P[2] = 3.0;

    int state = qTable.digitizeState(observation);

    // アクションの取得
    int action = qTable.getAction(observation, episode);
    int episode_reward = 0;

    double prevTotalSpread = env.calculate_total_spread(env.getcurrentState());//分散を計測
    for (int t = 0; t < max_number_of_steps; ++t){
      // if (t % 100 == 0) {
      //       std::cout << "Step: " << t << " / " << max_number_of_steps << std::endl;
      //   }
      success_flag = 0; // 成功フラグの初期化
      //報酬ゲット
      double reward = env.step(action);
      observation = env.getcurrentState();
      double currentTotalSpread = env.calculate_total_spread(env.getcurrentState());

      if (!env.terminate()){
        reward += -0.01;
        // sumReward += -0.01;
      }

      if (env.terminate() || t == max_number_of_steps -1){
        // std::cout << "x1 =" << observation[0] << ", x2 = " << observation[2] << ", xdistance =" << currentXDistance << std::endl;
        // std::cout << "y1 =" << observation[1] << ", y2 = " << observation[3] << ", ydistance =" << currentYDistance << std::endl;
        bool is_out_of_range = false;
        int num_particles = observation.size() /2;

        for (int i = 0; i < num_particles; ++i){
          double r2 = std::pow(observation[2*i], 2) + std::pow(observation[2*i +1] , 2);
          if (r2 > 6.25){
            is_out_of_range = true;
            break;
          }
        }
        bool is_separated = true;
        double threshold_sq = std::pow(1.5*lx, 2);

        if (!is_out_of_range){
          for (int i = 0; i < num_particles; ++i){
            for (int j = i +1; j < num_particles; ++j){
              double dx = observation[2*i] - observation[2*j];
              double dy = observation[2*i +1] - observation[2*j +1];
              if (dx *dx + dy* dy < threshold_sq){
                is_separated = false;
                break;
              }
            }
            if (!is_separated) break;
          }
        } else {
          is_separated = false;
        }

        if (is_out_of_range){
          reward += -2000000000000000.0;
          std::cout<<"fail: out of range" << std::endl;
        }else if(is_separated){
          reward += 200;
          success_flag++;
          std::cout<< "success: All separated" << std::endl;
        } else{
          // NEW: Reward partial separations even at episode end
          int separated_pairs = 0;
          for (int i = 0; i < num_particles; ++i){
            const dReal* pi = dBodyGetPosition(box[i].body);
            for (int j = i +1; j < num_particles; ++j){
              const dReal* pj = dBodyGetPosition(box[j].body);
              double dx = pi[0] - pj[0];
              double dy = pi[1] - pj[1];
              if (dx *dx + dy* dy >= threshold_sq){
                separated_pairs++;
              }
            }
          }
          if (separated_pairs > 0) {
            reward += separated_pairs * 10.0; // Reward each separated pair at episode end
            success_flag++; // Count as partial success
          } else {
            reward += -30;
          }
          // 追加: くっついているペア数に応じた罰則（粒の一辺 * 1.1 未満を「くっつき」と判定）
          int stuck_pairs = 0;
          double stick_threshold_sq = std::pow(box[0].lx * 1.1, 2);
          for (int i = 0; i < num_particles; ++i){
            const dReal* pi = dBodyGetPosition(box[i].body);
            for (int j = i +1; j < num_particles; ++j){
              const dReal* pj = dBodyGetPosition(box[j].body);
              double dx = pi[0] - pj[0];
              double dy = pi[1] - pj[1];
              if (dx*dx + dy*dy < stick_threshold_sq) {
                stuck_pairs++;
              }
            }
          }
          if (stuck_pairs > 0) {
            reward += - stuck_pairs * STICK_PENALTY_LEARN;
          }
        }
        // sumReward += reward;
      }
      else {
        // Prefer average pairwise spread (reduce single-outlier effect)
        int num_particles = observation.size() / 2;
        int num_pairs = std::max(1, num_particles * (num_particles - 1) / 2);
        double avgSpreadDiff = (currentTotalSpread - prevTotalSpread) / static_cast<double>(num_pairs);
        // 正規化してスケールを揃える（平均オーダー 1e-5 に基準化）
        reward += (avgSpreadDiff / B_AVG_SPREAD_DIFF) * 0.5; // scaled and normalized
        // std::cout<<"avgSpreadDiff: "<< avgSpreadDiff <<std::endl;

        // Reward fraction of particles that are separated
        double sep_score = env.calculate_separation_score(observation); // [0..1]
        reward += sep_score * 3.0; // scaled down
        // std::cout<<"separation score: "<< sep_score*3.0  <<std::endl;

        // NEW: Reward individual pair separations more aggressively
        int separated_pairs = 0;
        double threshold_sq = std::pow(1.5*lx, 2);
        for (int i = 0; i < num_particles; ++i){
          for (int j = i +1; j < num_particles; ++j){
            double dx = observation[2*i] - observation[2*j];
            double dy = observation[2*i+1] - observation[2*j+1];
            double dist_sq = dx*dx + dy*dy;
            if (dist_sq >= threshold_sq){
              separated_pairs++;
              reward += 2.0; // Reward each separated pair

            }
          }
        }
        // Bonus for having multiple separated pairs
        if (separated_pairs > 0) {
          reward += separated_pairs * 1.0;
          // std::cout<<"separated pairs: "<< separated_pairs <<std::endl;
        }

        // Penalize variance of nearest-neighbor distances to avoid one-outlier exploitation
        double nn_var = env.calculate_nearest_distance_variance(observation);
        reward -= nn_var * 0.2; // reduced (0.5x)
        // std::cout<<"nearest distance variance: "<< nn_var*2.0 <<std::endl;

        // Encourage every particle to increase its nearest-neighbor distance (sum of min distances)
        double sum_min_dists = 0.0;
        for (int i = 0; i < num_particles; ++i){
          double minDist = 1e9;
          for (int j = 0; j < num_particles; ++j){
            if (i == j) continue;
            double dx = observation[2*i] - observation[2*j];
            double dy = observation[2*i+1] - observation[2*j+1];
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < minDist) minDist = d;
          }
          sum_min_dists += minDist;
        }
        reward += sum_min_dists * 1.6; // increased (2x)
        // std::cout<<"sum of min distances: "<< sum_min_dists*1.6 <<std::endl;
        // 追加: 全粒について最小となる最近傍距離 (worst-case nearest-neighbor)
        // を大きくすることを促す項を入れる（1粒だけ遠ざけるのを抑制）
        double min_of_min = 1e9;
        for (int i = 0; i < num_particles; ++i){
          double minDist = 1e9;
          for (int j = 0; j < num_particles; ++j){
            if (i == j) continue;
            double dx = observation[2*i] - observation[2*j];
            double dy = observation[2*i+1] - observation[2*j+1];
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < minDist) minDist = d;
          }
          if (minDist < min_of_min) min_of_min = minDist;
        }
        // 学習時は控えめに加算
        reward += min_of_min * 5.0;
        // std::cout<<"min of min distances: "<< min_of_min*5.0 <<std::endl;
        // 追加: 非終端時にもくっつきペア数で罰則を与える
        {
          int stuck_pairs = 0;
          double stick_threshold_sq = std::pow(box[0].lx * 1.1, 2);
          for (int i = 0; i < num_particles; ++i){
            for (int j = i +1; j < num_particles; ++j){
              double dx = observation[2*i] - observation[2*j];
              double dy = observation[2*i+1] - observation[2*j+1];
              if (dx*dx + dy*dy < stick_threshold_sq) stuck_pairs++;
            }
          }
          if (stuck_pairs > 0) reward += - stuck_pairs * STICK_PENALTY_LEARN;
          // std::cout<<"stuck pairs penalty: "<< - stuck_pairs * STICK_PENALTY_LEARN <<std::endl;
        }
      }
      prevTotalSpread = currentTotalSpread;
      // sumReward += reward;

      int nextstate = qTable.digitizeState(env.getcurrentState());
      int nextAction = action;
      if (t%100 ==0){
        nextAction = qTable.getAction(env.getcurrentState(), episode);
      }

      qTable.updateQtable(state, action, nextstate, nextAction, reward, alpha, ganma);
      episode_reward += reward;
      sumReward += std::pow(ganma, t)*reward;

      if (env.terminate() || t == max_number_of_steps-1){
        std::cout << episode << "finished after " << t+1 << " time steps /mean" << calculateMean(last_time_steps) << std::endl;
        std::cout << "Episode:" << episode << "  sum of reward =" << sumReward << "flags =" << calculateMean(flags)<<std::endl;
        std::ofstream outputfile(F, std::ios::app);
        outputfile << episode << "," << sumReward <<"\n";
        sumReward = 0.0;
        std::cout << episode_reward << "\n" << std::endl;

        last_time_steps.erase(last_time_steps.begin());
        last_time_steps.push_back(t+1);
        step_list.push_back(t+1);
        flags.erase(flags.begin());
        flags.push_back(success_flag);

        dJointGroupDestroy(contactgroup);
        for (int i = 0; i < 6; i++){
          memset(&feedback[i], 0, sizeof(dJointFeedback));
        }
        destroyarm();
        destroybox();
        std::ofstream outFile(F2, std::ios::app);
        outFile << episode << "," << t+1 << "\n";
        outFile.close();
        outputfile.close();
        break;
      }
      state = nextstate;
      action = nextAction;
      
      observation = env.getcurrentState();

    }
    if (episode % 1000 == 0){
      std::ofstream qtablefile(F5 + std::to_string(episode) + ".csv");
      
      // map の要素は std::pair<int, std::vector<double>> です
      for (const auto& pair : qTable.table) {
          int state_id = pair.first;                 // 状態ID (Key)
          const std::vector<double>& q_vals = pair.second; // Q値のリスト (Value)

          // 先頭に状態IDを書き込む（これがないと復元できません）
          qtablefile << state_id << ",";

          // 続いてQ値を書き込む
          for (size_t i = 0; i < q_vals.size(); ++i){
              qtablefile << q_vals[i];
              if (i < q_vals.size() - 1){
                  qtablefile << ",";
              }
          }
          qtablefile << "\n";
      }
      qtablefile.close();
    }
    if (episode % interval_episodes == 0){
      contactgroup = dJointGroupCreate(0);
      std::vector<double> eval_observation = env.eval_reset();
      makeArm();
      makeSensor();
      const dReal *eval_sensor_pos = dBodyGetPosition(sensor);
      double z = lz/2 + 0.02;
      makebox(0, eval_observation[0]+eval_sensor_pos[0], eval_observation[1]+eval_sensor_pos[1], z);
      makebox(1, eval_observation[2]+eval_sensor_pos[0], eval_observation[3]+eval_sensor_pos[1], z);
      makebox(2, eval_observation[4]+eval_sensor_pos[0], eval_observation[5]+eval_sensor_pos[1], z);
      makebox(3, eval_observation[6]+eval_sensor_pos[0], eval_observation[7]+eval_sensor_pos[1], z);
      makebox(4, eval_observation[8]+eval_sensor_pos[0], eval_observation[9]+eval_sensor_pos[1], z);
      makebox(5, eval_observation[10]+eval_sensor_pos[0], eval_observation[11]+eval_sensor_pos[1], z);      P[0] = 0.0;
      P[0] = 0.0;
      P[1] = 0.0;
      P[2] = 3.0;

      int eval_state = qTable.digitizeState(eval_observation);

      // アクションの取得
      int eval_action = qTable.greedyAction(eval_observation);  //greeedyな行動選択になるように修正する
      double eval_prevTotalSpread = env.calculate_total_spread(eval_observation);
      double eval_sumReward = 0.0; // ローカル変数として初期化
    for (int eval_t = 0; eval_t < max_number_of_steps; ++eval_t){
      //報酬ゲット
      double eval_reward = env.step(eval_action);
      eval_observation = env.getcurrentState();
      double eval_currentTotalSpread = env.calculate_total_spread(eval_observation);

      if (!env.terminate()){
        eval_reward += -0.01;
        // eval_sumReward += -0.01;
      }

      if (env.terminate() || eval_t == max_number_of_steps -1){
        bool is_out_of_range = false;
        int num_particles = eval_observation.size() / 2;
        for (int i = 0; i < num_particles; ++i){
            double r2 = std::pow(eval_observation[2*i], 2) + std::pow(eval_observation[2*i+1], 2);
            if (r2 > 6.25) { is_out_of_range = true; break; }
        }
        bool is_separated = true;
        double threshold_sq = std::pow(1.5*lx, 2);
        if (!is_out_of_range){
          for (int i = 0; i < num_particles; ++i){
            for (int j = i+1; j < num_particles; ++j){
              double dx = eval_observation[2*i] - eval_observation[2*j];
              double dy = eval_observation[2*i+1] - eval_observation[2*j+1];
              if (dx*dx + dy*dy < threshold_sq) { is_separated = false; break; }
            }
            if (!is_separated) break;
          }
        } else {is_separated = false;}

            if (is_out_of_range) eval_reward += -2000.0;
            else if(is_separated) eval_reward += 200;
            else{
              // NEW: Reward partial separations even at episode end for evaluation
              int separated_pairs = 0;
              double threshold_sq = std::pow(1.5*lx, 2);
              for (int i = 0; i < num_particles; ++i){
                const dReal* pi = dBodyGetPosition(box[i].body);
                for (int j = i +1; j < num_particles; ++j){
                  const dReal* pj = dBodyGetPosition(box[j].body);
                  double dx = pi[0] - pj[0];
                  double dy = pi[1] - pj[1];
                  if (dx *dx + dy* dy >= threshold_sq){
                    separated_pairs++;
                  }
                }
              }
              if (separated_pairs > 0) {
                eval_reward += separated_pairs * 10.0; // Reward each separated pair at episode end
              } else {
                eval_reward += -30.0;
              }
              // 追加: 終端時のくっつきペア数ペナルティ（評価）
              {
                int eval_stuck_pairs = 0;
                double eval_stick_threshold_sq = std::pow(box[0].lx * 1.1, 2);
                for (int i = 0; i < num_particles; ++i){
                  const dReal* pi = dBodyGetPosition(box[i].body);
                  for (int j = i +1; j < num_particles; ++j){
                    const dReal* pj = dBodyGetPosition(box[j].body);
                    double dx = pi[0] - pj[0];
                    double dy = pi[1] - pj[1];
                    if (dx*dx + dy*dy < eval_stick_threshold_sq) eval_stuck_pairs++;
                  }
                }
                if (eval_stuck_pairs > 0) eval_reward += - eval_stuck_pairs * STICK_PENALTY_EVAL;
              }
            }
          }
          else {
            int num_particles = eval_observation.size() / 2;
            int num_pairs = std::max(1, num_particles * (num_particles - 1) / 2);
            double avgSpreadDiff = (eval_currentTotalSpread - eval_prevTotalSpread) / static_cast<double>(num_pairs);
            // 評価時も同様に正規化して重み付け（評価では強め）
            eval_reward += (avgSpreadDiff / B_AVG_SPREAD_DIFF) * 5.0;

            double eval_sep_score = env.calculate_separation_score(eval_observation);
            eval_reward += eval_sep_score * 30.0;

            // NEW: Reward individual pair separations more aggressively for evaluation
            int eval_separated_pairs = 0;
            double threshold_sq = std::pow(1.5*lx, 2);
            for (int i = 0; i < num_particles; ++i){
              for (int j = i +1; j < num_particles; ++j){
                double dx = eval_observation[2*i] - eval_observation[2*j];
                double dy = eval_observation[2*i+1] - eval_observation[2*j+1];
                double dist_sq = dx*dx + dy*dy;
                if (dist_sq >= threshold_sq){
                  eval_separated_pairs++;
                  eval_reward += 2.0; // Reward each separated pair
                }
              }
            }
            // Bonus for having multiple separated pairs
            if (eval_separated_pairs > 0) {
              eval_reward += eval_separated_pairs * 1.0;
            }

            double eval_nn_var = env.calculate_nearest_distance_variance(eval_observation);
            eval_reward -= eval_nn_var * 20.0;

            double sum_min_dists = 0.0;
            for (int i = 0; i < num_particles; ++i){
              double minDist = 1e9;
              for (int j = 0; j < num_particles; ++j){
                if (i == j) continue;
                double dx = eval_observation[2*i] - eval_observation[2*j];
                double dy = eval_observation[2*i+1] - eval_observation[2*j+1];
                double d = std::sqrt(dx*dx + dy*dy);
                if (d < minDist) minDist = d;
              }
              sum_min_dists += minDist;
            }
              eval_reward += sum_min_dists * 16.0;

              // 追加 (評価時): 最小の最近傍距離を重視して、worst-caseを大きくする
              double eval_min_of_min = 1e9;
              for (int i = 0; i < num_particles; ++i){
                double minDist = 1e9;
                for (int j = 0; j < num_particles; ++j){
                  if (i == j) continue;
                  double dx = eval_observation[2*i] - eval_observation[2*j];
                  double dy = eval_observation[2*i+1] - eval_observation[2*j+1];
                  double d = std::sqrt(dx*dx + dy*dy);
                  if (d < minDist) minDist = d;
                }
                if (minDist < eval_min_of_min) eval_min_of_min = minDist;
              }
              // 評価時は学習より強めに評価
              eval_reward += eval_min_of_min * 20.0;
              // 追加: 非終端時のくっつきペア数ペナルティ（評価）
              {
                int eval_stuck_pairs = 0;
                double eval_stick_threshold_sq = std::pow(box[0].lx * 1.1, 2);
                for (int i = 0; i < num_particles; ++i){
                  for (int j = i +1; j < num_particles; ++j){
                    double dx = eval_observation[2*i] - eval_observation[2*j];
                    double dy = eval_observation[2*i+1] - eval_observation[2*j+1];
                    if (dx*dx + dy*dy < eval_stick_threshold_sq) eval_stuck_pairs++;
                  }
                }
                if (eval_stuck_pairs > 0) eval_reward += - eval_stuck_pairs * STICK_PENALTY_EVAL;
              }
      }

      eval_prevTotalSpread = eval_currentTotalSpread;
      int eval_nextstate = qTable.digitizeState(env.getcurrentState());
      int eval_nextAction = eval_action;
      if (eval_t%100 ==0){
        eval_nextAction = qTable.greedyAction(env.getcurrentState());
      }
      eval_sumReward += std::pow(ganma, eval_t)*eval_reward;

      if (env.terminate() || eval_t == max_number_of_steps-1){  // ちょいファイルの記述の修正
      std::ofstream evalfile(F4, std::ios::app);
      evalfile << episode << "," << eval_sumReward <<"\n";
      eval_sumReward = 0.0;

      dJointGroupDestroy(contactgroup);
      for (int i = 0; i < 6; i++){
        memset(&feedback[i], 0, sizeof(dJointFeedback));
      }
      destroyarm();
      destroybox();
      evalfile.close();
      break;
    }
    eval_state = eval_nextstate;
    eval_action = eval_nextAction;
    
    eval_observation = env.getcurrentState();
    }
  }

    if (calculateMean(flags) >1.00){
      std::cout<<"Episode " << episode << " train agent successfully!" << std::endl;
      break;
    }
  }
  std::ofstream qtablefile(F3);
  // map の中身は pair<int, vector<double>> です
  for (const auto& pair : qTable.table) {
    int state_id = pair.first;                 // Key: 状態ID
    const auto& q_values = pair.second;        // Value: Q値リスト

    // 先頭に状態IDを書き込む
    qtablefile << state_id << ",";

    // Q値を書き込む
    for (size_t i = 0; i < q_values.size(); ++i){
      qtablefile << q_values[i];
      if (i < q_values.size() - 1){
        qtablefile << ",";
      }
    }
    qtablefile << "\n";
  }
  qtablefile.close();
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE(); //ＯＤＥの終了
  return 0;
}
