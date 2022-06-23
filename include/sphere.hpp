#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
public:
    Sphere() {
        // unit ball at the center
        c = Vector3f(0, 0, 0);
        r = 1;
    }

    Sphere(const Vector3f &center, double radius, Material *material) : Object3D(material, Sphere_T) {
        // 
        c = center;
        r = radius;
    }

    ~Sphere() override = default;

    bool intersect(const Ray &ray, Hit &hit, double tmin) override {
        //
        Vector3f l = c - ray.getOrigin();
        bool inside = (l.squaredLength() < r * r);
        bool on_surface = (l.squaredLength() == r * r);
        Vector3f rdir_norm = ray.getDirection().normalized();
        double rdir_len = ray.getDirection().length();
        if(!inside) {
            //不在内部
            if(on_surface) {
                //在表面
                return false;
            } else {
                //在外部
                double tp = Vector3f::dot(l, rdir_norm);
                if(tp < 0)
                    return false;
                double d = sqrt(l.squaredLength() - tp * tp);
                if(d > r)
                    return false;
                double t = (tp - sqrt(r * r - d * d)) / rdir_len;
                Vector3f norm = ray.pointAtParameter(t) - c;
                if(t > tmin && t < hit.getT()) {
                    hit.set(t, material, norm.normalized(), get_texel(ray.pointAtParameter(t)));
                    return true;
                } else
                    return false;
            }
        } else {
            //在内部
            Vector3f rdir_norm = ray.getDirection().normalized();
            double rdir_len = ray.getDirection().length();
            double tp = Vector3f::dot(l, rdir_norm);
            double d = sqrt(l.squaredLength() - tp * tp);
            double t = (tp + sqrt(r * r - d * d)) / rdir_len;
            Vector3f norm = c - ray.pointAtParameter(t);
            if(t > tmin && t < hit.getT()) {
                hit.set(t, material, norm.normalized(), get_texel(ray.pointAtParameter(t)));
                return true;
            } else
                return false;
        }
        return false;
    }

    Vector3f get_texel(Vector3f hit_point) override {
        // 墨卡托投影 Mercator Projection
        if (Object3D::material->texture == nullptr)
            return Vector3f(1., 1., 1.);
        Vector3f relative_pos = hit_point - c;
        Vector3f eqtr_projection = Vector3f(hit_point.x(), c.y(), hit_point.z()) - c;
        double latitude_cos = Vector3f::dot(relative_pos, Vector3f(0., -1., 0.)) / relative_pos.length();  // 纬度(0 deg, 180 deg)的余弦
        double latitude = 180 * acos(latitude_cos) / M_PI; 
        double longitude_cos = Vector3f::dot(eqtr_projection, Vector3f(0., 0., -1.)) / eqtr_projection.length();  // 经度(0 deg, 360 deg)的余弦
        double orientation = Vector3f::cross(Vector3f(0., 0., -1.), eqtr_projection).y();  // 判断经度是0-180还是180-360
        double longitude;
        if (orientation >= 0)
            longitude = 180.0 * acos(longitude_cos) / M_PI;
        else
            longitude = 180.0 + 180.0 * acos(-longitude_cos) / M_PI;
        
        // printf("%.4lf, %.4lf\n", latitude, longitude);
        double tex_width = double(Object3D::material->texture->width()) - 1;
        double tex_height = double(Object3D::material->texture->height()) - 1;
        // 完全利用材质，如果材质的比例不是2：1可能会出现错误拉伸
        double u = longitude / 360.0 * tex_width;
        double v = latitude / 180.0 * tex_height;
        return Object3D::material->texture->get_texel(u, v);
    }

protected:
    Vector3f c;
    double r;
};


#endif
