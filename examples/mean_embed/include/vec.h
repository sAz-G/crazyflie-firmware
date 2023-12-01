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

float calcNormf(float*);
float calcMeanMatrixf(float**, float*, int);
void  addVecf(float*, float*, int sz);
void  substractVec(float*, float*, float*);
void  scaleVec(float*, float, int);
float dotProdf(float*, float*);
float  clipVal(float, float, float);


#endif