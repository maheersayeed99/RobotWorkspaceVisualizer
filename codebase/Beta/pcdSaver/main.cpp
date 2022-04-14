#include <iostream>
#include <fstream>
#include <time.h> 
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <chrono>

#include <fssimplewindow.h>
#include "ysclass.h"

//using namespace std;

const double PI = 3.14159276;

class pcd {
public:
    float rgb = 4.808e+06;
    int numP;
    float x, y, z;
    std::vector<float> xvec, yvec, zvec;
    std::vector<float> vtx;

    bool term = false;

    double FOV = PI / 6.0;  // 30 degrees
    double viewDistance = 10.0;

    YsMatrix3x3 viewRotation;
    YsVec3 viewTarget;

    decltype(std::chrono::high_resolution_clock::now()) lastT;


    pcd();
    void makePCD(int numPoints);
    void savePCD(std::string fileName);
    void drawPCD();
    void RunOneStep();
    void boundingBox(double minmax[2][3], const std::vector <float>& vtx);
    void ResetViewDistance();
    bool terminateStep();
};



pcd::pcd(){

    viewTarget = YsVec3::Origin();
    lastT = std::chrono::high_resolution_clock::now();
}





void pcd::makePCD(int numPoints) {
    numP = numPoints;
    float theta, phi, rho;

    for (int i = 0; i < numPoints; ++i) {

        theta = ((rand() % 628) / 100.0);
        phi = ((rand() % 628) / 100.0);
        rho = ((rand() % 200) / 100.0) - 1;
        rho = rho;
        /*
        x = ((rand() % 200) / 100.0) - 1;
        y = ((rand() % 200) / 100.0) - 1;
        z = ((rand() % 200) / 100.0) - 1;
        */

        x = rho * sin(phi) * cos(theta);
        y = rho * sin(phi) * sin(theta);
        z = rho * cos(phi);


        //x = ((rand() % 200) / 100.0) - 1;
        //y = ((rand() % 200) / 100.0) - 1;
        //z = ((rand() % 200) / 100.0) - 1;


        xvec.push_back(x);
        yvec.push_back(y);
        zvec.push_back(z);



        vtx.push_back(x);
        vtx.push_back(y);
        vtx.push_back(z);
    }

}




void pcd::savePCD(std::string fileName) {

    std::ofstream PCD;

    PCD.open(fileName, std::fstream::out);

    if (PCD.fail()) {
        std::cout << "could not open" << std::endl;
    }

    else {
        PCD << "FIELDS x y z rgb" << std::endl;
        PCD << "SIZE 4 4 4 4" << std::endl;
        PCD << "TYPE F F F F" << std::endl;
        PCD << "COUNT 1 1 1 1" << std::endl;
        PCD << "WIDTH " << numP << std::endl;
        PCD << "HEIGHT 1" << std::endl;
        PCD << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
        PCD << "POINTS " << numP << std::endl;
        PCD << "DATA ascii" << std::endl;

        for (int i = 0; i < xvec.size(); ++i) {
            PCD << xvec[i] << " " << yvec[i] << " " << zvec[i] << " " << rgb << std::endl;
        }

        PCD.close();


        std::cout << "File saved as newTest.pcd" << std::endl;
    }

}




void pcd::boundingBox(double minmax[2][3], const std::vector <float>& vtx)			// Bounding Box Equation from class notes
{
    if (3 <= vtx.size())
    {
        minmax[0][0] = vtx[0];
        minmax[0][1] = vtx[1];
        minmax[0][2] = vtx[2];
        minmax[1][0] = vtx[0];
        minmax[1][1] = vtx[1];
        minmax[1][2] = vtx[2];
        for (int i = 0; i + 3 <= vtx.size(); i += 3)
        {
            minmax[0][0] = std::min<double>(minmax[0][0], vtx[i + 0]);
            minmax[0][1] = std::min<double>(minmax[0][1], vtx[i + 1]);
            minmax[0][2] = std::min<double>(minmax[0][2], vtx[i + 2]);
            minmax[1][0] = std::max<double>(minmax[1][0], vtx[i + 0]);
            minmax[1][1] = std::max<double>(minmax[1][1], vtx[i + 1]);
            minmax[1][2] = std::max<double>(minmax[1][2], vtx[i + 2]);
        }
    }
    else
    {
        minmax[0][0] = 0;
        minmax[0][1] = 0;
        minmax[0][2] = 0;
        minmax[1][0] = 0;
        minmax[1][1] = 0;
        minmax[1][2] = 0;
    }
}




void pcd::ResetViewDistance(void)
{
    double minmax[2][3];
    double diag;
    boundingBox(minmax, vtx);

    diag = pow(minmax[0][0] - minmax[1][0], 2) + pow(minmax[0][1] - minmax[1][1], 2) + pow(minmax[0][2] - minmax[1][2], 2);		// calculate diagonal
    diag = pow(diag, 0.5);

    viewDistance = (diag * 0.5) / (sin(0.5 * FOV));	
    
}














void pcd::drawPCD() {


    int wid, hei;
    FsGetWindowSize(wid, hei);								// get window size

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// clear screen

    glEnable(GL_DEPTH_TEST);								// ?


    glMatrixMode(GL_PROJECTION);							// ?					
    glLoadIdentity();										// ?
    gluPerspective(FOV * 180.0 / PI, (double)wid / (double)hei, viewDistance / 10.0, viewDistance * 3.0);		// ?

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    // This is the correct timing to set up a head light (a light moves with the view point) >>
    // Also the light needs to be normalized.
    GLfloat lightDir[] = { 0,1 / sqrt(2.0f),1 / sqrt(2.0f),0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightDir);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    // This is the correct timing to set up a head light (a light moves with the view point) <<


    // Set up model-view transformation from viewDistance, viewRotation, and viewTarget


    YsMatrix4x4 modelView;

    modelView.Translate(0, 0, -viewDistance);

    //viewRotationInverse = viewOrientation;
    //viewRotationInverse.Invert();

    modelView *= viewRotation;											// All reused from class notes

    modelView.Translate(-viewTarget);

    double modelViewGL[16];
    modelView.GetOpenGlCompatibleMatrix(modelViewGL);

    glMultMatrixd(modelViewGL);

    glEnableClientState(GL_VERTEX_ARRAY);
    //glEnableClientState(GL_COLOR_ARRAY);
    //glEnableClientState(GL_NORMAL_ARRAY);
    //glColorPointer(4, GL_FLOAT, 0, col.data());
    glVertexPointer(3, GL_FLOAT, 0, vtx.data());
    //glNormalPointer(GL_FLOAT, 0, nom.data());
    //glDrawArrays(GL_TRIANGLES, 0, vtx.size() / 3);


    glPointSize(2.0);

    glDrawArrays(GL_POINTS, 0, vtx.size() / 3);
    //glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    //glDisableClientState(GL_COLOR_ARRAY);

    FsSwapBuffers();
    FsSleep(25);

}
void pcd::RunOneStep() {
    
    auto deltaT = std::chrono::high_resolution_clock::now() - lastT;
    auto deltaTinMS = std::chrono::duration_cast<std::chrono::microseconds>(deltaT).count();
    double dt = (double)deltaTinMS / 1000000.0;
    lastT = std::chrono::high_resolution_clock::now();
    
    
    auto key = FsInkey();


    if (FSKEY_ESC == key)
    {
        term = true;
    }

    if (FsGetKeyState(FSKEY_LEFT))
    {
        YsMatrix3x3 rot;
        rot.RotateXZ(dt * PI / 6);
        viewRotation = rot * viewRotation;
    }
    if (FsGetKeyState(FSKEY_RIGHT))
    {
        YsMatrix3x3 rot;
        rot.RotateXZ(-dt * PI / 6);
        viewRotation = rot * viewRotation;
    }
    if (FsGetKeyState(FSKEY_UP))
    {
        YsMatrix3x3 rot;
        rot.RotateYZ(-dt * PI / 6);
        viewRotation = rot * viewRotation;
    }
    if (FsGetKeyState(FSKEY_DOWN))
    {
        YsMatrix3x3 rot;
        rot.RotateYZ(dt * PI / 6);
        viewRotation = rot * viewRotation;
    }



}


bool pcd::terminateStep() {
    return term;

}


int main(int argc, char* argv[])
{
    int numPoints;

    if (argc <= 1) {
        numPoints = 5000;
    }
    else {
        numPoints = atoi(argv[1]);
    }
    
    srand(time(NULL));
    
    FsOpenWindow(0, 0, 800, 600, 1);
    pcd newPCD;
    newPCD.makePCD(numPoints);
    newPCD.ResetViewDistance();
    newPCD.savePCD("newTest.pcd");

    while (true != newPCD.terminateStep())
    {
        FsPollDevice();
        newPCD.RunOneStep();
        newPCD.drawPCD();
    }

    return (0);
}