#include "../include/vec.h"
#include <math.h>

Vector3  addVecf(Vector3 vec1, Vector3 vec2)
{
    Vector3 result = {
                    .x=vec1.x+vec2.x,         
                    .y=vec1.y+vec2.y,
                    .z=vec1.z+vec2.z
                    };

    return result;
}

Vector3 negateVec(Vector3 vec)
{
    return (Vector3){.x=-vec.x, .y=-vec.y, .z=-vec.z};
}   

Vector3  substractVec(Vector3 from, Vector3 to)
{
    return addVecf(from, negateVec(to));
}

void  scaleVec(float* dst, float scl, int len)
{
    for(int k = 0; k < len; k++)
    {
        dst[k] = dst[k]*scl;
    }   
}

float clipVal(float val, float min, float max)
{// checked
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

float calcNormf(Vector3 vec)
{
    return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

float calcDistf(Vector3 vec1, Vector3 vec2)
{
    return calcNormf(substractVec(vec1, vec2));
}
