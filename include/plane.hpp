#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <cstdio>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    
    Plane() {

    }

    Plane(const Vector3f &normal, double d, Material *m) : Object3D(m) {
        this -> norm = normal;
        this -> norm.normalize();
        this -> d = -d;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        Vector3f rdir_norm = r.getDirection().normalized();
        double rdir_len = r.getDirection().length();
        if(Vector3f::dot(norm, r.getDirection()) == 0)
            return false;
        Vector3f hit_norm = norm;
        double t = - (d + Vector3f::dot(norm, r.getOrigin())) / (Vector3f::dot(norm, rdir_norm)) / rdir_len; // 交点在光线上的参数值t
        if (Vector3f::dot(norm, r.getDirection()) > 0) {
            hit_norm = -hit_norm;
        }
        if(t > tmin && t < h.getT()) {
            h.set(t, material, hit_norm);
            // printf("intersect");
            return true;
        } else
            return false;
        
        return false;
    }

protected:
    Vector3f norm;
    double d;

};


// double min(double a, double b) {
//     return a < b ? a : b;
// }
// double max(double a, double b) {
//     return a > b ? a : b;
// }
class Rectangle : public Object3D {
public:

    Vector3f point, x, y;  // 约定：x对应texture的width，y对应texture的height
    Rectangle(const Vector3f &point, const Vector3f &x, const Vector3f &y, Material *m) : Object3D(m), point(point), x(x), y(y) {}

    ~Rectangle() override = default;

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        Vector3f normal = (Vector3f::cross(x, y)).normalized();
        double d = Vector3f::dot(normal, point);
        Plane plane = Plane(normal, d, Object3D::material);
        Hit plane_hit;
        if (plane.intersect(r, plane_hit, tmin)) {
            Vector3f hit_point = r.pointAtParameter(plane_hit.getT());
            double xmin = std::min(point.x(), (point + x + y).x());
            double ymin = std::min(point.y(), (point + x + y).y());
            double zmin = std::min(point.z(), (point + x + y).z());
            double xmax = std::max(point.x(), (point + x + y).x());
            double ymax = std::max(point.y(), (point + x + y).y());
            double zmax = std::max(point.z(), (point + x + y).z());
            if (hit_point.x() >= xmin && hit_point.x() <= xmax &&
                hit_point.y() >= ymin && hit_point.y() <= ymax &&
                hit_point.z() >= zmin && hit_point.z() <= zmax) {
                h.set(plane_hit.getT(), Object3D::material, plane_hit.getNormal(), get_texel((hit_point)));
                return true;
            }
            else
                return false;
        }
        return false;
    }

    Vector3f get_texel(Vector3f hit_point) override {
        if (Object3D::material->texture == nullptr)  // 没有材质
            return Vector3f(1., 1., 1.);
        
        double xlen = x.length();
        double ylen = y.length();
        double tex_width = double(Object3D::material->texture->width()) - 1;
        double tex_height = double(Object3D::material->texture->height()) - 1;
        double ratio;
        if (tex_width / xlen * ylen > tex_height)  // 计算比例尺，在不拉伸的情况下进�?�最大面�?利用
            ratio = tex_height / ylen;
        else
            ratio = tex_width / xlen;
        double u = (Vector3f::dot(hit_point - point, x)) / x.length() * ratio;
        double v = (Vector3f::dot(hit_point - point, y)) / y.length() * ratio;
        return Object3D::material->texture->get_texel(u, v);
    }
};

#endif //PLANE_H
		

