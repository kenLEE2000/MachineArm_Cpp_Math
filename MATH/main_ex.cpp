
#include <math.h>
#include <iostream>

float TriangleCountR(float x, float y);
float TriangleCountS(float r, float xORy);
float* Motor1AngleAndLen(float ax, float ay, float r);
float* BinaryLinearEquation(float A, float B, float C, float D);

using namespace std;

int main() {
    float X = 350, Y = 150, Z = 120, R = 105, l1 = 203, l2 = 175.6, l3 = 81.2;
    float value[2] = {0};
    value[0] = Motor1AngleAndLen(X, Y, R)[0];
    value[1] = Motor1AngleAndLen(X, Y, R)[1];
    cout << "第一軸角度為：" <<  ((180 * value[0]) / M_PI) << endl;

    float A = l1, B = l2, C = value[1] - l3, D = Z;

    float exb[4] = {0};
    exb[0] = BinaryLinearEquation(A, B, C, D)[0];
    //cout << exb[0] << endl;

    exb[2] = BinaryLinearEquation(A, B, C, D)[2];
    //cout << exb[2] << endl;

    float ra = atan(exb[0] / exb[2]);
    cout << "第二軸角度為：" << ((180 * ra) / M_PI) << endl;

    float rb = acos((C - exb[2]) / B);
    float rc = ra + rb;
    cout << "第三軸角度為：" << ((180 * rc) / M_PI) << endl;
    cout << "第四軸角度為：" << ((180 * rb) / M_PI) << endl;

}

float TriangleCountR(float x, float y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

float TriangleCountS(float r, float xORy) {
    return sqrt(pow(r, 2) - pow(xORy, 2));
}

float* Motor1AngleAndLen(float ax, float ay, float r) {
    float output[2];
    float lPM = TriangleCountR(ax, ay - r);
    float aPMb= asin(r / lPM);
    float aMPc= atan((ay - r) / ax);
    float acPb= (M_PI / 2) - aPMb - aMPc;
    float aOPb;
    if((int)(180 * acPb / M_PI) == 90) aOPb = 0;
    else aOPb= (M_PI / 2) - acPb;
    output[0] = aOPb;                                //第一軸旋轉值
    float lMB = TriangleCountS(lPM, r);
    output[1] = lMB;                                 //點與第二軸距離
    return output;
}

float* BinaryLinearEquation(float A, float B, float C, float D) {
    float ANS[4];

    float a = pow(2 * C, 2) + pow((2 * D), 2);
    float b = -1 * (4 * D) * (pow(C, 2) + pow(A, 2) + pow(D, 2) - pow(B, 2));
    float c = -1 * ((pow(2 * C, 2) * pow(A, 2)) - pow(pow(C, 2) + pow(A, 2) + pow(D, 2) - pow(B, 2), 2));

    float bANS1 = ((-1 * b) + sqrt(pow(b, 2) - (4 * a * c))) / (2 * a);
    float bANS2 = ((-1 * b) - sqrt(pow(b, 2) - (4 * a * c))) / (2 * a);
    ANS[0] = bANS1;                                   //B 第一值
    ANS[1] = bANS2;                                   //B 第二值

    float aANS1 = sqrt(pow(A, 2) - pow(ANS[0], 2));
    float aANS2 = sqrt(pow(A, 2) - pow(ANS[1], 2));
    ANS[2] = aANS1;                                   //A 第一值
    ANS[3] = aANS2;                                   //A 第二值
    return ANS;
}