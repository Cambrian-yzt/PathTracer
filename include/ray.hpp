#ifndef RAY_H
#define RAY_H

#include <cassert>
#include <iostream>
#include <Vector3f.h>
#include <limits>
#include <cmath>


// Ray class mostly copied from Peter Shirley and Keith Morley
class Ray {
public:

    Ray() = delete;
    Ray(const Vector3f &orig, const Vector3f &dir, double _time, double _wavelength = -1.0) {
        origin = orig;
        direction = dir;
        wavelength = _wavelength;
        time = _time;
    }

    Ray(const Ray &r) {
        origin = r.origin;
        direction = r.direction;
        wavelength = r.wavelength;
        time = r.time;
    }

    const double get_time() const {
        return time;
    }

    const Vector3f &getOrigin() const {
        return origin;
    }

    const Vector3f &getDirection() const {
        return direction;
    }

    const double getWavelength() const {
        return wavelength;
    }

    Vector3f pointAtParameter(double t) const {
        return origin + direction * t;
    }

    bool check_box_collision(Vector3f &bmin, Vector3f &bmax) const {
        const double epsilon = 1e-8;
        const double MAX = std::numeric_limits<double>::max();
        double tx1 = -MAX, tx2 = MAX, ty1 = -MAX, ty2 = MAX, tz1 = -MAX, tz2 = MAX;
        if (fabs(getDirection().x()) > epsilon) {
            tx1 = (bmin.x() - (getOrigin().x())) / (getDirection().x());
            tx2 = (bmax.x() - (getOrigin().x())) / (getDirection().x());
            if (tx1 > tx2)
                std::swap(tx1, tx2);
        }
        if (fabs(getDirection().y()) > epsilon) {
            ty1 = (bmin.y() - (getOrigin().y())) / (getDirection().y());
            ty2 = (bmax.y() - (getOrigin().y())) / (getDirection().y());
            if (ty1 > ty2)
                std::swap(ty1, ty2);
        }
        if (fabs(getDirection().z()) > epsilon) {
            tz1 = (bmin.z() - (getOrigin().z())) / (getDirection().z());
            tz2 = (bmax.z() - (getOrigin().z())) / (getDirection().z());
            if (tz1 > tz2)
                std::swap(tz1, tz2);
        }
        double t_min = std::max(tx1, std::max(ty1, tz1));
        double t_max = std::min(tx2, std::min(ty2, tz2));
        return t_min <= t_max;
    }


private:

    Vector3f origin;
    Vector3f direction;
    double time;  // 用于计算运动模糊，取值为[0, 1]
    double wavelength;  // in meters

};

inline std::ostream &operator<<(std::ostream &os, const Ray &r) {
    os << "Ray <" << r.getOrigin() << ", " << r.getDirection() << ">";
    return os;
}

#endif // RAY_H
