#include "../include/vec.h"

void  addVecf(float* dst, float* src, int sz)
{

    /*perform addition of two vectors*/
    for(int k = 0; k < sz; k++)
    {
        dst[k] += src[k];
    }
}

void  scaleVec(float* dst, float scl, int len)
{
    for(int k = 0; k < len; k++)
    {
        dst[k] = dst[k]*scl;
    }   
}

float clipVal(float val, float min, float max)
{
    if(val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    else{
        return val;
    }
    
}