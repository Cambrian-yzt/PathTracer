#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <vecmath.h>
#include "object3d.hpp"

// transforms a 3D point using a matrix, returning a 3D point
static Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point) {
    return (mat * Vector4f(point, 1)).xyz();
}

// transform a 3D directino using a matrix, returning a direction
static Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir) {
    return (mat * Vector4f(dir, 0)).xyz();
}

// TODO: implement this class so that the intersect function first transforms the ray
class Transform : public Object3D {
public:
    Transform(): Object3D(nullptr, Transform_T) {}

    Transform(const Matrix4f &m, Object3D *obj) : o(obj), Object3D(nullptr, Transform_T) {
        transform = m.inverse();
    }

    ~Transform() {
    }

    virtual bool intersect(const Ray &r, Hit &h, double tmin) {
        Vector3f trSource = transformPoint(transform, r.getOrigin());
        Vector3f trDirection = transformDirection(transform, r.getDirection());
        Ray tr(trSource, trDirection, r.get_time(), r.getWavelength());
        bool inter = o->intersect(tr, h, tmin);
        if (inter) {
            h.set(h.getT(), h.getMaterial(), transformDirection(transform.transposed(), h.getNormal()).normalized());
        }
        return inter;
    }

    Vector3f transformedPoint(Vector3f &pnt) {
        return transform.getSubmatrix3x3(0, 0).inverse() * (pnt - Vector3f(transform(0, 3), transform(1, 3), transform(2, 3)));
    }

    Vector3f transformedDirection(Vector3f &dir) {
        return transform.getSubmatrix3x3(0, 0).inverse() * dir;
    }

    Object3D* get_obj() {
        return o;
    }
    Object3D *o; //un-transformed object
    Matrix4f transform;
protected:
};

#endif //TRANSFORM_H
