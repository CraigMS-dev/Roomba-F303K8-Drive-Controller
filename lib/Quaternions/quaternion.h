#ifdef __cplusplus
extern "C" {
#endif

#ifndef QUATERNION_INCLUDED
#define QUATERNION_INCLUDED

#include "vector_3d.h"

typedef struct Quaternion {

    float a, b, c, d;
    // a=w
    // b=x
    // c=y
    // d=z
    // q = a + bi + cj + dk

} Quaternion;

typedef struct euler_angles {
    float roll, pitch, yaw;

} euler_angles;

Quaternion quaternion_initialize(float a, float b, float c, float d);
Quaternion quaternion_product(Quaternion q1, Quaternion q2);
Quaternion quaternion_conjugate(Quaternion q);
Quaternion quaternion_normalize(Quaternion q);
Quaternion quaternion_between_vectors(vector_ijk v1, vector_ijk v2);
vector_ijk quaternion_rotate_vector(vector_ijk v, Quaternion q);
euler_angles quaternion_to_euler_angles(Quaternion q);

#endif

#ifdef __cplusplus
}
#endif
