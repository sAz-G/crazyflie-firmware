#include "../include/vec.h"

void  addVecf(float* dst, float* src, int sz)
{

    /*perform addition of two vectors*/
    for(int k = 0; k < sz; k++)
    {
        dst[k] += src[k];
    }
}