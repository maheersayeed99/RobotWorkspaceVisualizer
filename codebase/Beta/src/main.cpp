#include "Robot.h"

#include <iostream>

#include <chrono>

#include <fssimplewindow.h>
#include "ysclass.h"
#include <ysglfontdata.h>

const double PI = 3.14159276;

class ApplicationMain {
 private:
  bool terminate = false;

  /* PCD Viewer */
  double FOV = PI / 6.0;  // 30 degrees
  double viewDistance = 10.0;
  YsMatrix3x3 viewRotation;
  YsVec3 viewTarget;

  /* ROBOT */
  Robot robot;

 public:
  ApplicationMain(int argc, char* argv[]);
  bool MustTerminate(void) const;
  void RunOneStep(void);
  void Draw(void) const;
  std::vector<float> vtx, col;
  void boundingBox(double minmax[2][3], const std::vector<float>& vtx);
  void ResetViewDistance();

  decltype(std::chrono::high_resolution_clock::now()) lastT;
};

ApplicationMain::ApplicationMain(int argc, char* argv[]) {
  if (2 == argc) {
    if (argc == 2 && 0 == robot.read_urdf(argv[1])) {
      std::cout << "Loading URDF" << std::endl;
      robot.print_joints();
      std::vector<double> temp{0, 0, 0, 0, 0};
      std::vector<std::vector<double>> configs{temp, temp, temp, temp,
                                               temp, temp, temp, temp};
      robot.get_workspace(configs, 8);
      robot.print_map(robot.point_cloud_);
      // TODO: Use real point cloud here
      // robot.makePCD();        // uncomment when fk is working
      robot.makeTempPCD(5000);
      // TODO: Find a way to convert robot.point_cloud_ to vtx and col
      vtx = robot.vtx;
      col = robot.col;

      viewTarget = YsVec3::Origin();
      lastT = std::chrono::high_resolution_clock::now();
      ResetViewDistance();

      robot.savePCD("newTest.pcd");
    } else {
      std::cout << "Error: Failed to read a .urdf file." << std::endl;
    }
  } else {
    std::cout << "Usage: find_ws urdfFile-Name.urdf" << std::endl;
  }

  /*================ URDF ======================*/
}

void ApplicationMain::Draw(void) const {
  int wid, hei;
  FsGetWindowSize(wid, hei);  // get window size

  glClear(GL_COLOR_BUFFER_BIT |
          GL_DEPTH_BUFFER_BIT);  // clear screen and depth values
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);  // enables depth buffer

  glMatrixMode(GL_PROJECTION);  // ?
  glLoadIdentity();             // ?
  gluPerspective(FOV * 180.0 / PI, (double)wid / (double)hei,
                 viewDistance / 10.0, viewDistance * 3.0);  // ?

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  YsMatrix4x4 modelView;

  modelView.Translate(0, 0, -viewDistance);

  // viewRotationInverse = viewOrientation;
  // viewRotationInverse.Invert();

  modelView *= viewRotation;  // All reused from class notes

  modelView.Translate(-viewTarget);

  double modelViewGL[16];
  modelView.GetOpenGlCompatibleMatrix(modelViewGL);

  glMultMatrixd(modelViewGL);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  // glEnableClientState(GL_NORMAL_ARRAY);
  glColorPointer(4, GL_FLOAT, 0, col.data());
  glVertexPointer(3, GL_FLOAT, 0, vtx.data());
  // glNormalPointer(GL_FLOAT, 0, nom.data());
  // glDrawArrays(GL_TRIANGLES, 0, vtx.size() / 3);

  glPointSize(2.0);

  glDrawArrays(GL_POINTS, 0, vtx.size() / 3);
  // glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);

  // for (int i = 0; i < 7; ++i) {
  //   drawMenu(i, wid, hei, 1);
  //   if (i < 6) {
  //     drawMenu(i, wid, hei, 0);
  //   }
  // }

  FsSwapBuffers();
  FsSleep(25);
}

void ApplicationMain::RunOneStep(void) {
  FsPollDevice();
  auto deltaT = std::chrono::high_resolution_clock::now() - lastT;
  auto deltaTinMS =
      std::chrono::duration_cast<std::chrono::microseconds>(deltaT).count();
  double dt = (double)deltaTinMS / 1000000.0;
  lastT = std::chrono::high_resolution_clock::now();

  auto key = FsInkey();

  if (FSKEY_ESC == key) {
    terminate = true;
  }

  if (FsGetKeyState(FSKEY_LEFT)) {
    YsMatrix3x3 rot;
    rot.RotateXZ(dt * PI / 6);
    viewRotation = rot * viewRotation;
  }
  if (FsGetKeyState(FSKEY_RIGHT)) {
    YsMatrix3x3 rot;
    rot.RotateXZ(-dt * PI / 6);
    viewRotation = rot * viewRotation;
  }
  if (FsGetKeyState(FSKEY_UP)) {
    YsMatrix3x3 rot;
    rot.RotateYZ(-dt * PI / 6);
    viewRotation = rot * viewRotation;
  }
  if (FsGetKeyState(FSKEY_DOWN)) {
    YsMatrix3x3 rot;
    rot.RotateYZ(dt * PI / 6);
    viewRotation = rot * viewRotation;
  }
}

bool ApplicationMain::MustTerminate(void) const { return terminate; }

void ApplicationMain::boundingBox(
    double minmax[2][3],
    const std::vector<float>& vtx)  // Bounding Box Equation from class notes
{
  if (3 <= vtx.size()) {
    minmax[0][0] = vtx[0];
    minmax[0][1] = vtx[1];
    minmax[0][2] = vtx[2];
    minmax[1][0] = vtx[0];
    minmax[1][1] = vtx[1];
    minmax[1][2] = vtx[2];
    for (int i = 0; i + 3 <= vtx.size(); i += 3) {
      minmax[0][0] = std::min<double>(minmax[0][0], vtx[i + 0]);
      minmax[0][1] = std::min<double>(minmax[0][1], vtx[i + 1]);
      minmax[0][2] = std::min<double>(minmax[0][2], vtx[i + 2]);
      minmax[1][0] = std::max<double>(minmax[1][0], vtx[i + 0]);
      minmax[1][1] = std::max<double>(minmax[1][1], vtx[i + 1]);
      minmax[1][2] = std::max<double>(minmax[1][2], vtx[i + 2]);
    }
  } else {
    minmax[0][0] = 0;
    minmax[0][1] = 0;
    minmax[0][2] = 0;
    minmax[1][0] = 0;
    minmax[1][1] = 0;
    minmax[1][2] = 0;
  }
}

void ApplicationMain::ResetViewDistance(void) {
  double minmax[2][3];
  double diag;
  boundingBox(minmax, vtx);

  diag = pow(minmax[0][0] - minmax[1][0], 2) +
         pow(minmax[0][1] - minmax[1][1], 2) +
         pow(minmax[0][2] - minmax[1][2], 2);  // calculate diagonal
  diag = pow(diag, 0.5);

  viewDistance = (diag * 0.5) / (sin(0.5 * FOV));
}

int main(int argc, char* argv[]) {
  FsOpenWindow(0, 0, 800, 600, 1);
  ApplicationMain app(argc, argv);

  while (true != app.MustTerminate()) {
    app.RunOneStep();
    app.Draw();
  }

  return 0;
}