// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 6.4:  ３自由度ロボットアーム(逆運動学実装）
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// arm3.cpp by Kosei Demura (2007-5-18)
//
// 更新履歴　(change log)
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <iostream>
#include <random>

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

dReal Lx = 0.5, Ly = 0.5, Lz = 0.5;
typedef struct {
  dBodyID body;
  dGeomID geom;
  dReal Lx, Ly, Lz, m;
} Stand;
Stand stand[1];

MyObject rlink[NUM];                   // リンク

dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節
int           ANSWER = 1;              // 逆運動学の解
int ACTION = 0; //action番号

dReal P[3] = {0, 0, 3.0+Lz};             // 先端の位置
dReal a[3], b[3];                         // 有顔ベクトル(a,b)
dReal THETA[NUM] = {0.0, 0.0, 0.0, 0.0};  // 関節の目標角度[rad]
dReal l[4] = { 0.10+Lz, 0.40, 1.00, 1.50};   // リンクの長さ[m]
dVector3 target_pos;
dJointFeedback feedback[6];
dJointFeedback jcollide;

bool fast_mode = false;
int fast_mode_counter = 0;
const int FAST_MODE_DURATION = 100;

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

      // if (!isGround){
      //   if (i ==0){
      //     calculatestickness(b1,b2);
      //   }
      // }
      
      if ((o1 == box[1].geom && o2 == box[0].geom) ||(o2 == box[1].geom && o1 == box[0].geom)
        ||(o1 == box[1].geom && o2 == box[2].geom) ||(o2 == box[1].geom && o1 == box[2].geom)
        ||(o1 == box[0].geom && o2 == box[2].geom) ||(o2 == box[0].geom && o1 == box[2].geom)){
        dJointSetFeedback(c, &jcollide);
        // b1 = dGeomGetBody(o1);
        // b2 = dGeomGetBody(o2);
        // std::cout << "Force3: " << jcollide.f1[0] << ", " << jcollide.f1[1] << ", " << jcollide.f1[2] << std::endl;
      }

    }
      // std::cout << "Force1: " << feedback[0].f1[0] << ", " << feedback[0].f1[1] << ", " << feedback[0].f1[2] << std::endl;
  }

}

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

void makebox(int num, double x = 0.0, double y = 1.0, double z = lz/2+0.005)
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

void makestand(int num, double x = 0.0, double y = 0.0, double z = 0.005)
{
  // double x = 0.0, y = 1.0, z = 0.54;
  dMass mass;

  stand[num].body = dBodyCreate(world);
  dBodySetPosition(stand[num].body, x, y, z);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, 0.05, Lx, Ly, Lz);
  dBodySetMass(stand[num].body, &mass);
  stand[num].Lx = Lx;
  stand[num].Ly = Ly;
  stand[num].Lz = Lz;
  stand[num].geom = dCreateBox(space, stand[num].Lx,stand[num].Ly,stand[num].Lz);
  dGeomSetBody(stand[num].geom, stand[num].body);
}


// センサ位置の表示
void printSensorPosition()
{
  double *pos = (double *) dBodyGetPosition(sensor);
  printf("Current Position: x=%6.2f y=%6.2f z=%6.2f \n",pos[0],pos[1],pos[2]);
  // printf("P : x=%5.2f y=%5.2f z=%5.2f \n",P[0],P[1],P[2]);
}

/*** ロボットアームの生成 ***/
void  makeArm()
{
  double a = 0;
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.05+Lz, 0.25+Lz, 1.00+Lz, 2.25+Lz};  // 重心 z
  dReal length[NUM] = {0.10, 0.40, 1.00, 1.50};  // 長さ
  dReal weight[NUM] = {9.00, 2.00, 2.00, 2.00};  // 質量
  dReal r[NUM]      = {0.20, 0.04, 0.04, 0.04};  // 半径
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00+Lz, 0.10+Lz, 0.50+Lz, 1.50+Lz};  // 関節中心点 z
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
  dBodyDestroy(stand[0].body);
  dGeomDestroy(stand[0].geom);  
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

void restart(){
  dJointGroupDestroy(contactgroup);
  for (int i = 0; i < 6; i++){
    memset(&feedback[i], 0, sizeof(dJointFeedback));
  }
  destroyarm();
  destroybox();

  contactgroup = dJointGroupCreate(0);
  makeArm();
  makeSensor();
  makestand(0);
  double x = (rand() / (RAND_MAX +1.0)) * 0.5;
  double y = 1 + (rand() / (RAND_MAX +1.0)) * 0.5;
  double z = lz/2 + 0.05 +(rand() / (RAND_MAX +1.0)) * 0.15;
  // double x = 1;
  // double y = 1;
  // double z = lz/2 + 0.05;
  makebox(0, x, y, z);
  makebox(1, x, y -0.25, z);
  makebox(2, x, y -0.50, z);
  makebox(3, x+0.25, y, z);
  makebox(4, x+0.25, y-0.25, z);
  makebox(5, x+0.25, y-0.50, z);
  P[0]=0;
  P[1]=0;
  P[2]=3.0+Lz;
  ACTION = 0;
}

/*** ロボットアームの描画 ***/
void drawArm()
{
	dReal r,length;

  dGeomCylinderGetParams(rlink[0].geom, &r, &length);
  dsDrawCylinder(dBodyGetPosition(rlink[0].body),
  dBodyGetRotation(rlink[0].body),length,r);

	for (int i = 1; i < NUM; i++ ) {       // カプセルの描画
		dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
		if (i != NUM -1)
			dsDrawCapsule(dBodyGetPosition(rlink[i].body),
								    dBodyGetRotation(rlink[i].body),length,r);
		else
			dsDrawCylinder(dBodyGetPosition(rlink[i].body),
								     dBodyGetRotation(rlink[i].body),length,r);
	}
}

// 位置センサの描画
void drawSensor()
{
   dReal sides[] = {0.01,0.01,0.01};

   dsSetColor(1,0,0);
   dBodyGetRotation(sensor);
   dsDrawBox(dBodyGetPosition(sensor),dBodyGetRotation(sensor),sides);
}

void drawbox(int num)
{
  dReal sides[] = {lx, ly, lz};

  dsSetColor(0,1,0);
  dsDrawBox(dBodyGetPosition(box[num].body), dBodyGetRotation(box[num].body), sides);

}
void drawstand(int num)
{
  dReal sides[] = {Lx, Ly, Lz};

  dsSetColor(0,1,1);
  dsDrawBox(dBodyGetPosition(stand[num].body), dBodyGetRotation(stand[num].body), sides);

}

// 目標位置の描画
void drawP()
{
   dReal tmpP[3];
   dMatrix3 tmpR;

   tmpP[0] = P[0];
   tmpP[1] = P[1];
   tmpP[2] = P[2];

   dsSetColor(1,0,0);

   dRSetIdentity(tmpR);
   dsDrawSphere(tmpP, tmpR, 0.02);
   //printf("P= %f %f %f \n",tmpP[0],tmpP[1],tmpP[2]);
}

/*** 制御 ***/
void Pcontrol()
{
  dReal k =  80.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
		if (z >=   M_PI) z -= 2.0 * M_PI;
		if (z <= - M_PI) z += 2.0 * M_PI;
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}

/*** 視点と視線の設定 ***/
void start()
{
  // float xyz[3] = {    3.0f, 1.3f, 0.8f};          // 視点[m]
  // float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]

  float xyz[3] = {    -0, 0.9, 1.38};          // 視点[m]
  float hpr[3] = { 129, -90, 0.0f};          // 視線[°]

  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}



// 逆運動学
// 目標位置がアームの到達範囲以外のときは，逆運動学を計算できないので
// その処理を組み込む必要があります．
void  inverseKinematics()
{
  double Px, Py, Pz;
  double distance = sqrt(P[0] * P[0] + P[1] * P[1]
    + (P[2] - (l[0] + l[1])) * (P[2] - (l[0] + l[1])));
  double maxReach = l[2] + l[3];
  double minReach = fabs(l[2] - l[3]);

  if (distance > maxReach || distance < minReach){
    printf("Target  Position: x=%6.2f y=%6.2f z=%6.2f \n", P[0], P[1], P[2]);
    return;
  }

  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
 	printf("Target  Position: x=%6.2f y=%6.2f z=%6.2f \n", Px, Py, Pz);

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

//物体同士の距離を計算
dReal calculateDistance(dBodyID body1, dBodyID body2){
  const dReal*pos1 = dBodyGetPosition(body1);
  const dReal*pos2 = dBodyGetPosition(body2);
  
  dReal distance = sqrt(pow(pos2[0] - pos1[0], 2) 
                      + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));
  return distance;
}
dReal calculateX(dBodyID body1, dBodyID body2){
  const dReal*pos1 = dBodyGetPosition(body1);
  const dReal*pos2 = dBodyGetPosition(body2);
  
  dReal distance = pos1[0] - pos2[0];
  return distance;
}
dReal calculateY(dBodyID body1, dBodyID body2){
  const dReal*pos1 = dBodyGetPosition(body1);
  const dReal*pos2 = dBodyGetPosition(body2);
  
  dReal distance = pos1[1] - pos2[1];
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
    std::cout << "straigh " << i1 << "," << i2 << std::endl;
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

  dReal relvel[3] = {
    vel2[0] - vel1[0],
    vel2[1] - vel1[1],
    vel2[2] - vel1[2]
  };

  // フィードバックによる衝突力
  const dReal *f3 = jcollide.f1;

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
    std::cout << "over range!!!!" << std::endl;
  }
  double dist_ = calculateDistance(body1,body2);

  // 距離条件
  if (dist_ <= 1.40*ly){
  // if (fabs(relPos[0]) <= lx*1.1 && fabs(relPos[1]) <= ly*1.1) {
    // 緑側のY方向の力が強い場合
    if (force2[1] > flim) {
      if (force2[0] > 0) {
        // dBodyAddForceAtRelPos(body1, force2[0] + relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
        // dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, -0.5*lx, -0.5*ly, 0);
        printf("Mode change\n");
      } else {
        // dBodyAddForceAtRelPos(body1, -force2[0]/2 + relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
        // dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, -0.5*lx, -0.5*ly, 0);
        printf("Mode change_\n");
      }
    }

    if (force2[1] < -flim) {
      if (force2[0] > 0) {
        dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
        dBodyAddForceAtRelPos(body1, force2[0]/2 + relPos[0], force2[1]/2 + relPos[1] , 0, -0.5*lx, -0.5*ly, 0);
        printf("green jab1\n");
      } else {
        dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
        dBodyAddForceAtRelPos(body1, force2[0]/2 + relPos[0], force2[1]/2 + relPos[1] , 0, -0.5*lx, -0.5*ly, 0);
        printf("green jab2\n");
      }
    }

    // 青側（body1）からの力が大きいとき
    if (force1[1] > flim) {
      if (force1[0] > 0) {
        dBodyAddForceAtRelPos(body2, - relPos[0], force1[1]/2 - relPos[1] ,0, 0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body2, force1[0]/2 - relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, 0.5*ly, 0);
        printf("blue crash1\n");
      } else {
        dBodyAddForceAtRelPos(body2, force1[0]/2 - relPos[0], force1[1]/2 - relPos[1] , 0, 0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body2, -relPos[0], force1[1]/2 - relPos[1] ,0, -0.5*lx, 0.5*ly, 0);
        printf("blue crash2\n");
      }
    }
    if (force1[1] < -flim){
      printf("wow\n");
    }

    // if (force1[1] < -flim) {
    //   dBodyAddForceAtRelPos(body2, force1[0]/2 - relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, 0.5*ly, 0);
    //   dBodyAddForceAtRelPos(body2, -force1[0]/2 - relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, -0.5*ly, 0);
    //   printf("blue jab\n");
    // }
    
    dBodyAddForceAtRelPos(body1, relPos[0], relPos[1], 0, 0.5*lx, -0.5*ly, 0);
    dBodyAddForceAtRelPos(body1, relPos[0], relPos[1], 0, -0.5*lx, -0.5*ly, 0);
    dBodyAddForceAtRelPos(body2, -relPos[0], -relPos[1], 0, 0.5*lx, 0.5*ly, 0);
    dBodyAddForceAtRelPos(body2, -relPos[0], -relPos[1], 0, -0.5*lx, 0.5*ly, 0);  

    // 通常の引き寄せ
    if (force1[2] < -20){
      // dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2, -0.5*lx, 0.5*ly, 0);
      // dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2, -0.5*lx, -0.5*ly, 0);  
      dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2+feedback[i1].f2[2]/2, 0.5*lx, 0.5*ly, 0);
      dBodyAddForceAtRelPos(body2, 0, 0, force1[2]/2+feedback[i1].f2[2]/2, -0.5*lx, 0.5*ly, 0);  
    } else if (force2[2] < -20){
      dBodyAddForceAtRelPos(body1, 0, 0, force2[2]/2+feedback[i2].f2[2]/2, 0.5*lx, 0.5*ly, 0);
      dBodyAddForceAtRelPos(body1, 0, 0, force2[2]/2+feedback[i2].f2[2]/2, -0.5*lx, 0.5*ly, 0);
        // 押し付けられてる方は、アームから受ける力 - 反発力 してるはずだから、もう一方もそうする？？ 
    }
  } else {
    printf("NO sticking\n");
  }

  // 状態の表示（デバッグ用）
  // printf("青が出す力: (%.3f, %.3f, %.3f)\n", force1_raw[0], force1_raw[1], force1_raw[2]);
  // printf("緑が出す力: (%.3f, %.3f, %.3f)\n", force2_raw[0], force2_raw[1], force2_raw[2]);
  printf("物体にかかる力青: (%.3f, %.3f, %.3f)\n", force1[0], force1[1], force1[2]);
  printf("物体にかかる力緑: (%.3f, %.3f, %.3f)\n", force2[0], force2[1], force2[2]);
  // printf("collision: (%.3f, %.3f, %.3f)\n", f3[0], f3[1], f3[2]);
  printf("物体の位置青: (%.3f, %.3f, %.3f)\n", pos1[0], pos1[1], pos1[2]);
  printf("物体の位置緑: (%.3f, %.3f, %.3f)\n", pos2[0], pos2[1], pos2[2]);
  // printf("relPos: %.3f, %.3f, %.3f\n", relPos[0], relPos[1], relPos[2]);
}


void calculatestickness_side(dBodyID body1, dBodyID body2) { //body1が左側、body2が右側のとき成立する
  int i1 = getBodyIndex(body1);
  int i2 = getBodyIndex(body2);
  const dReal *pos1 = dBodyGetPosition(body1);
  const dReal *pos2 = dBodyGetPosition(body2);
  const dReal *force1_raw = dBodyGetForce(body1);
  const dReal *force2_raw = dBodyGetForce(body2);
  double flim = 10.0;
  std::cout << "side " << i1 << "," << i2 << std::endl;
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
    std::cout << "over range!!!!" << std::endl;
  }
  double dist_ = calculateDistance(body1,body2);


  // 距離条件
  if (dist_ <= 1.40*lx){
    if (force2[0] > flim){
      if(force2[1] > 0){
        dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body1, force2[0]/2 + relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
        printf("Mode change1\n");
      }else {
        dBodyAddForceAtRelPos(body1, force2[0]/2 +relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body1, relPos[0], force2[1]/2 + relPos[1] , 0, 0.5*lx, -0.5*ly, 0);
        printf("green jab1\n");
      }
    }

    if (force1[0] < -flim){
      if (force1[1] > 0){
        dBodyAddForceAtRelPos(body2, - relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body2, force1[0]/2 -relPos[0], force1[1]/2 - relPos[1] ,0, -0.5*lx, -0.5*ly, 0);
        printf("blue crash2\n");
      } else{
        dBodyAddForceAtRelPos(body2, force1[0]/2 -relPos[0], force1[1]/2 - relPos[1] , 0, -0.5*lx, 0.5*ly, 0);
        dBodyAddForceAtRelPos(body2, -relPos[0], force1[1]/2 - relPos[1] ,0, -0.5*lx, -0.5*ly, 0);
        printf("wow2\n");
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
    printf("NO sticking\n");
  }
  printf("物体にかかる力青: (%.3f, %.3f, %.3f)\n", force1[0], force1[1], force1[2]);
  printf("物体にかかる力緑: (%.3f, %.3f, %.3f)\n", force2[0], force2[1], force2[2]);

}


void get_goal_target(dBodyID body, double x, double y, double z, dVector3 target){
  const dReal *pos = dBodyGetPosition(body);
  target[0] = pos[0] + x;
  target[1] = pos[1] + y;
  target[2] = pos[2] + z;
}
void get_local_goal(dBodyID hand, double lx, double ly, double lz, dVector3 goal)
{
    dVector3 world_offset;
    // 手先ローカル座標の点 (lx, ly, lz) をワールド座標に変換
    dBodyGetRelPointPos(hand, lx, ly, lz, world_offset);
    std::cout<< "World offset" << world_offset[0] << ","<<world_offset[1] <<  ","<<world_offset[2] << std::endl;

    const dReal* hand_pos = dBodyGetPosition(hand);

    goal[0] = hand_pos[0] + world_offset[0];
    goal[1] = hand_pos[1] + world_offset[1];
    goal[2] = hand_pos[2] + world_offset[2];
}


void updatepath(){
  double radius = 0.5;
  double omega = 0.01;

  t += omega;

  P[0] = radius * cos(t);
  P[1] = 1.0;
  P[2] = 1.5 + radius * sin(t);
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
    // case 1:
    // dBodyGetRelPointPos(box[0].body, 0, -0.5*ly-0.5*r, 0.2*lz, target_pos); break;
    case 2:
    case 5:
    case 6:
    case 7:
    case 8:
    dBodyGetRelPointPos(box[0].body, 0, 0.5*ly+0.5*r, 0.2*lz, target_pos); break;
    case 3:
    case 9:
    case 10:
    case 11:
    case 12:
    dBodyGetRelPointPos(box[0].body, -0.5*lx-0.5*r, 0, 0.2*lz, target_pos); break;
    case 4:
    case 13:
    case 14:
    case 15:
    case 1:
    dBodyGetRelPointPos(box[0].body, 0.5*lx+0.5*r, 0, 0.2*lz, target_pos); break;
  }
  double dist = sqrt(pow(current_pos[0] - target_pos[0], 2) +
                     pow(current_pos[1] - target_pos[1], 2) +
                     pow(current_pos[2] - target_pos[2], 2));
    std::cout << "dist =" << dist << std::endl;

  if (!fast_mode && dist < 0.09 && ACTION!=0){
    fast_mode = true;
    fast_mode_counter = FAST_MODE_DURATION;
    std::cout << "enter fast mode" << std::endl;
  }

  if (fast_mode){
    switch (ACTION)
    {
    case 5:
      get_local_goal(box[0].body,0, 0.5*ly+0.5*r, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos);
      break;
    case 13:
      get_local_goal(box[0].body,0.5*lx+0.5*r, 0, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos);
      break;
    case 6:
      get_local_goal(box[0].body,0, 0.5*ly+0.5*r, 0.2*lz, -0.5*lx, -0.5*ly, 0.2*lz, target_pos);
      break;
    case 14:
      get_local_goal(box[0].body,0.5*lx+0.5*r, 0, 0.2*lz, 0.5*lx, 0.5*ly, 0.2*lz, target_pos);
      break;
    case 7:
      get_local_goal(box[0].body,0, 0.5*ly+0.5*r, 0.2*lz, -0.5*lx, 0.5*ly, 0.2*lz, target_pos);
      break;
    case 10:
      get_local_goal(box[0].body,-0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, -0.5*ly, 0.2*lz, target_pos);
      break;
    case 8:
      get_local_goal(box[0].body,0, 0.5*ly+0.5*r, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos);
      break;
    case 9:
      get_local_goal(box[0].body,-0.5*lx-0.5*r, 0, 0.2*lz, 0.5*lx, 0, 0.2*lz, target_pos);
      break;
    case 11:
      get_local_goal(box[0].body,-0.5*lx-0.5*r, 0, 0.2*lz, -0.5*lx, -0.5*ly, 0.2*lz, target_pos);
      break;
    case 12:
      get_local_goal(box[0].body,-0.5*lx-0.5*r, 0, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos);
      break;
    case 15:
      get_local_goal(box[0].body,0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0.5*ly, 0.2*lz, target_pos);
      break;
    case 1:
      get_local_goal(box[0].body,0.5*lx+0.5*r, 0, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos);
      break;
    case 2:
      get_local_goal(box[0].body,0, 0.5*ly+0.5*r, 0.2*lz, -0.5*lx, 0, 0.2*lz, target_pos);
      break;
    case 3:
      get_local_goal(box[0].body,-0.5*lx-0.5*r, 0, 0.2*lz, 0, -0.5*ly, 0.2*lz, target_pos);
      break;
    case 4:
      get_local_goal(box[0].body,0.5*lx+0.5*r, 0, 0.2*lz, 0, 0.5*ly, 0.2*lz, target_pos);
      break;
    
    }
    fast_mode_counter--;
    if (fast_mode_counter <=0){
      fast_mode = false;
      std::cout << "exit fast mode" << std::endl;
      // ACTION =0;
    }
  }

  

  double rate = fast_mode ? 0.025 : 0.016;

  for (int i = 0; i < 3; i++){
    P[i] += rate*(target_pos[i] - P[i]);
  }
}

int judge_relation_local(int i, int j)
{
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


/*** シミュレーションループ ***/
void simLoop(int pause)
{
if (!pause){
  for (int i = 0; i < 6; i++){
    memset(&feedback[i], 0, sizeof(dJointFeedback));
  }
  dSpaceCollide(space, 0, &nearCallback);
  // updatepath();
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
  printSensorPosition();
  Pcontrol();                                  // P制御
	dWorldStep(world, 0.01);                     // 動力学計算
  dJointGroupEmpty(contactgroup);
  for (int i = 0; i < 6; i++){
      printf("feedback[%d]: %f %f %f\n", i,
          feedback[i].f1[0],
          feedback[i].f1[1],
          feedback[i].f1[2]);
  }

}
  drawArm();                                   // ロボットの描画
	drawP();                                     // 目標位置の描画
	drawSensor();                                // 先端位置の描画
  drawstand(0);
  drawbox(0);
  drawbox(1);
  drawbox(2);
  drawbox(3);
  drawbox(4);
  drawbox(5);
}

/*** キー入力関数 ***/
void command2(int cmd)
{
  switch (cmd) {
  case '0':  ACTION = 0; break;
  case '1':  ACTION = 1; break;    
  case '2':  ACTION = 2; break;    
  case '3':  ACTION = 3; break;   
  case '4':  ACTION = 4; break;    
  case '5':  ACTION = 5; break;
  case '6':  ACTION = 6; break;    
  case '7':  ACTION = 7; break;    
  case '8':  ACTION = 8; break;    
  case '9':  ACTION = 9; break;    
  case 'q':  ACTION = 10; break;
  case 'w':  ACTION = 11; break;
  case 'e':  ACTION = 12; break;
  case 't':  ACTION = 13; break;
  case 'y':  ACTION = 14; break;
  case 'u':  ACTION = 15; break;
  case 'j':  P[0] += 0.1; break;   // jキーを押すと先端のx座標が増加
  case 'f':  P[0] -= 0.1; break;   // fキーを押すと先端のx座標が減少
  case 'k':  P[1] += 0.1; break;   // kキーを押すと先端のy座標が増加
  case 'd':  P[1] -= 0.1; break;   // dキーを押すと先端のy座標が減少
  case 'l':  P[2] += 0.1; break;   // lキーを押すと先端のz座標が増加
  case 's':  P[2] -= 0.1; break;   // sキーを押すと先端のz座標が減少
  case 'r': restart(); break;
  }
}

/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command2;                       // command関数
  fn.path_to_textures = "../../drawstuff/textures";
}

int main(int argc, char *argv[]) 
{
  dInitODE(); //ＯＤＥの初期化
  setDrawStuff();
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  dWorldSetERP(world, 1.0);
  dWorldSetCFM(world, 1e-5);
  ground = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  makeArm();                                      // アームの生成
  makeSensor();                                   // センサの生成
  makebox(0);
  makebox(1, 0, 0.75);
  makebox(2, 0,0.5);
  makebox(3, 0.25, 1);
  makebox(4, 0.25,0.75);
  makebox(5, 0.25, 0.50);
  makestand(0);
  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE(); //ＯＤＥの終了
  return 0;
}

