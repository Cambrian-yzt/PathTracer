#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"
#include <limits>

using namespace std;

// Base class for all 3d entities.
enum ObjType {
    Sphere_T,
    Plane_T,
    Triangle_T,
    Mesh_T,
    Rectangle_T,
    Group_T,
    Transform_T,
    Curve_T,
    RevSufrace_T
};

enum BoxState {
    IN,
    ON,
    OUT
};

class Object3D {
public:
    Object3D() : material(nullptr) {}

    virtual ~Object3D() = default;

    ObjType object_type;
    Vector3f box_min = Vector3f(-1e10, -1e10, -1e10);
    Vector3f box_max = Vector3f(1e10, 1e10, 1e10);

    explicit Object3D(Material *material, ObjType obj_type, Vector3f motion = Vector3f::ZERO) {
        this->material = material;
        this->object_type = obj_type;
        this->motion = motion;
    }

    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray &r, Hit &h, double tmin) = 0;
    virtual BoxState check_box (Vector3f &bmin, Vector3f &bmax) { return ON; }
    virtual Vector3f get_texel (Vector3f hit_point, double time = 0.0) { return Vector3f(1., 1., 1.); }
    Material *material;
    Vector3f motion;
    
protected:

};

#endif

