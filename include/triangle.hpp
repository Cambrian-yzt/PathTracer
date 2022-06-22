#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include "plane.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0] = a; vertices[1] = b; vertices[2] = c;
		normal = Vector3f::cross(a - b, a - c);
	}

	bool intersect( const Ray& ray,  Hit& hit , double tmin) override {
		Vector3f rdir_norm = ray.getDirection().normalized();
		double rdir_len = ray.getDirection().length();
		if(Vector3f::dot(normal, rdir_norm) == 0)
            return false;
		Vector3f e1 = vertices[0] - vertices[1];
		Vector3f e2 = vertices[0] - vertices[2];
		Vector3f s = vertices[0] - ray.getOrigin();
		Matrix3f inv = Matrix3f(rdir_norm, e1, e2);
		double coe = 1 / inv.determinant();
		Matrix3f tm = Matrix3f(s, e1, e2);
		double t = coe * tm.determinant() / rdir_len;
		Matrix3f bm = Matrix3f(rdir_norm, s, e2);
		double beta = coe * bm.determinant();
		Matrix3f gm = Matrix3f(rdir_norm, e1, s);
		double gamma = coe * gm.determinant();
		if(t > 0 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1 && beta + gamma <= 1) {
			if(t > tmin && t < hit.getT()) {
				hit.set(t, material, normal);
				return true;
			} else
				return false;
		}
        return false;
	}
	Vector3f normal;
	Vector3f vertices[3];
protected:

};

#endif //TRIANGLE_H
