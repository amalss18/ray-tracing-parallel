#include "stdio.h"
#include "stdlib.h"
// #include "cuda_runtime.h"

#define NC  100
#define NR  100

#define MAX_DEPTH 3

struct opt_prop {
    float ambient[3];
    float specular[3];
    float diffuse[3];
    float shininess;
    float reflection;
};

struct sphere {
    float *center;
    float radius;
    // struct opt_prop opt;
    // float ambient[3];
    // float specular[3];
    // float diffuse[3];
    // float shininess;
    // float reflection;
};

void ray_direction(float *pos1, float *pos2);
// return normalised direction vector
void sphere_intersection(float *origin, float *ray_direction, float *center, float radius);
// return t (distance from origin in direction of ray) or -1 (no intersection)
void nearest_intersection_object(struct sphere *objects, float *origin, float *ray_direction, float* intersection_point, int* object_idx);
void normal_sphere(float *center, float radius, float *intersection_point);
// 
    
void illumination();
//  



int main(){
    float center[] = {1.0,2.0,3.0};
    struct sphere all[2];
    all[0].center = center;

    printf("%f test\n", (all[0].center[2]));
    return 0;
}

/* 

Code Structure:
n
- 1d matrix - pixels - length (NC*NR)
- 1d matrix - cam_pos - length(3)
- 1d matrix - obj-pos - length (8*n_obj)
- 1d matrix - image - length(3*NC*NR)
- 1d matrix - screen location - length(4)
*/

/*

- we have source
- we have objects
- we have screen, pixels, dx, dy
- we can find ray of hope (1d matrix length 3)
- 
*/

