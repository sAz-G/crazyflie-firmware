#ifndef __VEC_H__
#define __VEC_H__

typedef struct _Vec3
{
    float x;
    float y;
    float z;
}Vector3;

typedef struct _Vec6
{
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
}Vector6;

float calcNormf(Vector3);
float calcMeanMatrixf(float**, float*, int);
Vector3  addVecf(Vector3, Vector3);
Vector3  substractVec(Vector3, Vector3);
Vector3 negateVec(Vector3 vec);
void  scaleVec(float*, float, int);
float dotProdf(float*, float*);
float  clipVal(float, float, float);
float calcDistf(Vector3, Vector3);
float normf(Vector3);


#endif