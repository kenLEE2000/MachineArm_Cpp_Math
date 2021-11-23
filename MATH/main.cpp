#include "MATH_CLASS.h"
#include <math.h>

float TriangleCountR(float x, float y);
float TriangleCountS(float r, float xORy);
float* Motor1AngleAndLen(float ax, float ay, float r);
float* BinaryLinearEquation(float A, float B, float C, float D);

JNIEXPORT jdoubleArray JNICALL Java_MATH_1CLASS_T
        (JNIEnv * env, jobject obj, jdouble x, jdouble y, jdouble z, jdouble link1, jdouble link2, jdouble link3, jdouble r) {
    jdoubleArray result;
    result = (*env).NewDoubleArray(4);
    if(result == NULL) return NULL;

    double Axis_Value[4] = {0, 0, 0, 0};

    float X = x, Y = y, Z = z, Link1 = link1, Link2 = link2, Link3 = link3, R = r;
    Axis_Value[0] = (180 * Motor1AngleAndLen(X, Y, R)[0]) / M_PI;

    float A = Link1, B = Link2, C = Motor1AngleAndLen(X, Y, R)[1] - Link3, D = Z;
    float ANS[4] = {BinaryLinearEquation(A, B, C, D)[2],
                    BinaryLinearEquation(A, B, C, D)[0],
                    BinaryLinearEquation(A, B, C, D)[3],
                    BinaryLinearEquation(A, B, C, D)[1]};

    float a = 0, b = 0;
    if(ANS[1] < 0)  { a = ANS[2]; b = ANS[3] ;}
    else { a = ANS[0]; b = ANS[1];}

    float ra = atan(b / a);
    Axis_Value[1] = 90 - ((180 * ra) / M_PI) ;

    float rb = acos((C - a) / B);
    Axis_Value[3] = (180 * rb) / M_PI ;

    float rc = ra + rb;
    Axis_Value[2] = (180 * rc) / M_PI;

    (*env).SetDoubleArrayRegion(result, 0, 4, Axis_Value);
    return result;
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



