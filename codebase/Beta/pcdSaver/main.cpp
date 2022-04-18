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
#include <ysglfontdata.h>

//using namespace std;

const double PI = 3.14159276;


class button {
public:
    float x, y, wid, hei, xoff, yoff;
    int val;
    int colorRed[3] = { 200,70,70 };
    int colorBlue[3] = { 100,100,255 };
    int colorWhite[3] = { 255,255,255 };
    char type;
    bool hover;


    button(float x1, float y1, int val1, char type1) {
        x = x1;
        y = y1;
        val = val1;
        wid = 100;
        hei = 40;
        xoff = 10;
        yoff = -10;
        type = type1;

    }

    void draw();
    bool isInButton(int x, int y);
    bool checkInButton(float x, float y);

};

void button::draw() {

    char str[256];

    if (type == 'l') {

        glColor3ub(0,0,0);
        glRasterPos2i(x+xoff, y+yoff);
        sprintf(str, "Link %d", val);
        YsGlDrawFontBitmap12x16(str);

        glBegin(GL_QUADS);

        if (hover) {
            glColor3ub(colorWhite[0], colorWhite[1], colorWhite[2]);
        }
        else {
            glColor3ub(colorRed[0], colorRed[1], colorRed[2]);
        }
        glVertex2f(x, y);
        glVertex2f(x + wid, y);
        glVertex2f(x + wid, y - hei);
        glVertex2f(x, y - hei);

        glEnd();

    }
    else if (type == 'j') {
        glColor3ub(0, 0, 0);
        glRasterPos2i(x+xoff, y+yoff);
        sprintf(str, "Joint %d", val);
        YsGlDrawFontBitmap12x16(str);

        glBegin(GL_QUADS);

        if (hover) {
            glColor3ub(colorWhite[0], colorWhite[1], colorWhite[2]);
        }
        else {
            glColor3ub(colorBlue[0], colorBlue[1], colorBlue[2]);
        }
        glVertex2f(x, y);
        glVertex2f(x + wid, y);
        glVertex2f(x + wid, y - hei);
        glVertex2f(x, y - hei);

        glEnd();
    }
    
}

bool button::isInButton(int x1, int y1) {
    if ((x1 > x) && (x1 < x + wid)) {
        if ((y1 < y) && (y1 > y + hei)) {
            return true;
        }
    }
    return false;
}





class pcd {
public:
    float rgb = 4.808e+06;
    int numP;
    float x, y, z;
    std::vector<float> xvec, yvec, zvec;
    std::vector<float> vtx , col;

    bool term = false;

    double FOV = PI / 6.0;  // 30 degrees
    double viewDistance = 10.0;

    YsMatrix3x3 viewRotation;
    YsVec3 viewTarget;

    decltype(std::chrono::high_resolution_clock::now()) lastT;


    std::vector<button*> linkVec;
    std::vector<button*> jointVec;

    pcd() {
        viewTarget = YsVec3::Origin();
        lastT = std::chrono::high_resolution_clock::now();
    }






    void makePCD(int numPoints);
    void savePCD(std::string fileName);
    void drawPCD();
    void RunOneStep();
    void boundingBox(double minmax[2][3], const std::vector <float>& vtx);
    void ResetViewDistance();
    bool terminateStep();

    void makeButtons(int links, int joints);

    //draw

    void drawMenu(int val, int wid, int hei, int type);


};





void pcd::makeButtons(int links, int joints) {


    int wid, hei;
    FsGetWindowSize(wid, hei);

    for (int i = 0; i < links; ++i) {
        button* currButton = new button(-(wid/2) + 10 , -(hei/2) + 60 + (60*i), (i + 1), 'l');
        linkVec.push_back(currButton);
    }

    for (int i = 0; i < joints; ++i) {
        button* currButton = new button(-(wid / 2) + 150, -(hei / 2) + 60 + (60 * i), (i + 1), 'j');
        jointVec.push_back(currButton);
    }

}




void pcd::makePCD(int numPoints) {
    numP = numPoints;
    float theta, phi, rho;
    
    for (int i = 0; i < numPoints; ++i) {

        //theta = ((rand() % 628) / 100.0);
        //phi = ((rand() % 628) / 100.0);

        if (i < numPoints / 3) {
            rho = 1;

            col.push_back(1);
            col.push_back(1);
            col.push_back(0);
            col.push_back(1);

        }
        else if (i < 2 * numPoints / 3) {
            rho = 2;

            col.push_back(1);
            col.push_back(0);
            col.push_back(1);
            col.push_back(1);
        }
        else {
            rho = 3;

            col.push_back(0);
            col.push_back(1);
            col.push_back(1);
            col.push_back(1);
        }


        //rho = ((rand() % 200) / 100.0) - 1;
        //rho = rho;
        
        x = ((rand() % 200) / 100.0) - 1;
        y = ((rand() % 200) / 100.0) - 1;
        z = ((rand() % 200) / 100.0) - 1;
        
        x *= rho;
        y *= rho;
        z *= rho;

        //x = rho * sin(phi) * cos(theta);
        //y = rho * sin(phi) * sin(theta);
        //z = rho * cos(phi);





        //x = ((rand() % 200) / 100.0) - 1;
        //y = ((rand() % 200) / 100.0) - 1;
        //z = ((rand() % 200) / 100.0) - 1;


        //xvec.push_back(x);
        //yvec.push_back(y);
        //zvec.push_back(z);



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


void pcd::drawMenu(int val, int wid, int hei, int type) {


    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(-wid/2.0, wid/2.0, hei/2.0, -hei/2.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    
    for (button* currButton : linkVec) {
        currButton->draw();
    }

    for (button* currButton : jointVec) {
        currButton->draw();
    }
    
    /*
    char str[256];

    if (type == 1) {
        glColor3ub(255, 0, 0);
        glRasterPos2i((-wid / 2 + 30), (-hei / 2 + 50 +(60 * (val))));										// position FPS draw									// define character arraw for drawing fps on screen
        sprintf(str, "Link %d", val+1);							// Make String
    }
    else if (type == 0) {
        glColor3ub(0, 0, 255);
        glRasterPos2i((-wid / 2 + 150), (-hei / 2 + 50+ (60 * (val))));										// position FPS draw										// define character arraw for drawing fps on screen
        sprintf(str, "Joint %d", val+1);							// Make String
    }
    
    YsGlDrawFontBitmap12x16(str);							// Draw String

    */

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();


}











void pcd::drawPCD() {


    int wid, hei;
    FsGetWindowSize(wid, hei);								// get window size

    

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// clear screen and depth values
    glClearColor(0, 0, 0, 0);
    glEnable(GL_DEPTH_TEST);								// enables depth buffer

    glMatrixMode(GL_PROJECTION);							// ?					
    glLoadIdentity();										// ?
    gluPerspective(FOV * 180.0 / PI, (double)wid / (double)hei, viewDistance / 10.0, viewDistance * 3.0);		// ?

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    // This is the correct timing to set up a head light (a light moves with the view point) >>
    // Also the light needs to be normalized.
    //GLfloat lightDir[] = { 0,1 / sqrt(2.0f),1 / sqrt(2.0f),0 };
    //glLightfv(GL_LIGHT0, GL_POSITION, lightDir);
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0);
    //glEnable(GL_COLOR_MATERIAL);
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
    glEnableClientState(GL_COLOR_ARRAY);
    //glEnableClientState(GL_NORMAL_ARRAY);
    glColorPointer(4, GL_FLOAT, 0, col.data());
    glVertexPointer(3, GL_FLOAT, 0, vtx.data());
    //glNormalPointer(GL_FLOAT, 0, nom.data());
    //glDrawArrays(GL_TRIANGLES, 0, vtx.size() / 3);


    glPointSize(2.0);

    glDrawArrays(GL_POINTS, 0, vtx.size()/3);
    //glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);



    for (int i = 0; i < 7; ++i) {
        drawMenu(i, wid, hei, 1);
        if (i < 6) {
            drawMenu(i, wid, hei, 0);
        }
    }
    
    


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
    
    FsOpenWindow(0, 0, 1200, 600, 1);
    pcd newPCD;
    newPCD.makePCD(numPoints);
    newPCD.makeButtons(7,6);
    newPCD.ResetViewDistance();
    newPCD.savePCD("newTest.pcd");

    std::cout << newPCD.linkVec.size() << "   " << newPCD.linkVec[0]->x << std::endl;

    while (true != newPCD.terminateStep())
    {
        FsPollDevice();
        newPCD.RunOneStep();
        newPCD.drawPCD();
    }

    return (0);
}