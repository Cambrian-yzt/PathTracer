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
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m, Triangle_T) {
		vertices[0] = a; vertices[1] = b; vertices[2] = c;
		normal = Vector3f::cross(a - b, a - c).normalized();
		Object3D::box_min = Vector3f(
			min(a.x(), min(b.x(), c.x())),
			min(a.y(), min(b.y(), c.y())),
			min(a.z(), min(b.z(), c.z()))
		);
		Object3D::box_max = Vector3f(
			max(a.x(), max(b.x(), c.x())),
			max(a.y(), max(b.y(), c.y())),
			max(a.z(), max(b.z(), c.z()))
		);
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

	BoxState check_box(Vector3f &bmin, Vector3f &bmax) override {
		// using separating axis theorem (SAT)
		Vector3f cen = (bmin + bmax) / 2.;
		Vector3f ext = (bmax - bmin) / 2.;
		Vector3f a = vertices[0] - cen;
		Vector3f b = vertices[1] - cen;
		Vector3f c = vertices[2] - cen;
		Vector3f e0 = b - a;
		Vector3f e1 = c - b;
		Vector3f e2 = a - c;
		Vector3f x = Vector3f(1., 0., 0.);
		Vector3f y = Vector3f(0., 1., 0.);
		Vector3f z = Vector3f(0., 0., 1.);
		// 13 axes are required in SAT
		Vector3f axes[13];
		axes[0] = Vector3f::cross(x, e0).normalized();
		axes[1] = Vector3f::cross(x, e1).normalized();
		axes[2] = Vector3f::cross(x, e2).normalized();
		axes[3] = Vector3f::cross(y, e0).normalized();
		axes[4] = Vector3f::cross(y, e1).normalized();
		axes[5] = Vector3f::cross(y, e2).normalized();
		axes[6] = Vector3f::cross(z, e0).normalized();
		axes[7] = Vector3f::cross(z, e1).normalized();
		axes[8] = Vector3f::cross(z, e2).normalized();
		axes[9] = x;
		axes[10] = y;
		axes[11] = z;
		axes[12] = Vector3f::cross(e0, e1).normalized();
		for (int i = 0; i < 13; i++) {
			// project the triangle onto the current axis
			double pa = Vector3f::dot(a, axes[i]);
			double pb = Vector3f::dot(b, axes[i]);
			double pc = Vector3f::dot(c, axes[i]);
			double box_ext_on_axis = - std::numeric_limits<double>().max();
			for (unsigned short j = 0; j < 8; j++) {
				double t0 = ((j & 1) ? cen[0] - ext[0] : cen[0] + ext[0]);
				double t1 = ((j & 2) ? cen[1] - ext[1] : cen[1] + ext[1]);
				double t2 = ((j & 4) ? cen[2] - ext[2] : cen[2] + ext[2]);
				Vector3f t = Vector3f(t0, t1, t2);
				box_ext_on_axis = max(fabs(Vector3f::dot(t, axes[i])), box_ext_on_axis);
			}
			// box_ext_on_axis = max(
			// 	max(
			// 		max(
			// 			fabs(Vector3f::dot(Vector3f(cen[0] - ext[0], cen[1] - ext[1], cen[2] - ext[2]), axes[i])),
			// 			fabs(Vector3f::dot(Vector3f(cen[0] - ext[0], cen[1] - ext[1], cen[2] + ext[2]), axes[i]))
			// 		),
			// 		max(
			// 			fabs(Vector3f::dot(Vector3f(cen[0] - ext[0], cen[1] + ext[1], cen[2] - ext[2]), axes[i])),
			// 			fabs(Vector3f::dot(Vector3f(cen[0] + ext[0], cen[1] - ext[1], cen[2] - ext[2]), axes[i]))
			// 		)
			// 	),
			// 	max(
			// 		max(
			// 			fabs(Vector3f::dot(Vector3f(cen[0] - ext[0], cen[1] + ext[1], cen[2] + ext[2]), axes[i])),
			// 			fabs(Vector3f::dot(Vector3f(cen[0] + ext[0], cen[1] - ext[1], cen[2] + ext[2]), axes[i]))
			// 		),
			// 		max(
			// 			fabs(Vector3f::dot(Vector3f(cen[0] + ext[0], cen[1] + ext[1], cen[2] - ext[2]), axes[i])),
			// 			fabs(Vector3f::dot(Vector3f(cen[0] + ext[0], cen[1] + ext[1], cen[2] + ext[2]), axes[i]))
			// 		)
			// 	)
			// );
			if (min(pa, min(pb, pc)) > box_ext_on_axis || max(pa, max(pb, pc)) < - box_ext_on_axis)
				return OUT;
		}

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				if (vertices[i][j] <= bmin[j] || vertices[i][j] >= bmax[j])
					return ON;
		return IN;
	}

	Vector3f normal;
	Vector3f vertices[3];
protected:

};

#endif //TRIANGLE_H
