#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"

class Material;

class Hit {
public:

    // constructors
    Hit() {
        material = nullptr;
        t = 1e38;
    }

    Hit(double _t, Material *m, const Vector3f &n, const Vector3f &tc = Vector3f(1., 1., 1.)) {
        t = _t;
        material = m;
        normal = n;
        texture_color = tc;
    }

    Hit(const Hit &h) {
        t = h.t;
        material = h.material;
        normal = h.normal;
    }

    // destructor
    ~Hit() = default;

    double getT() const {
        return t;
    }

    Material *getMaterial() const {
        return material;
    }

    const Vector3f &getNormal() const {
        return normal;
    }

    Vector3f &get_texture_color() {
        return texture_color;
    }

    void print() {
        std::cout << t;
        normal.print(); 
        std::cout << "============\n";
    }

    void set(double _t, Material *m, const Vector3f &n, const Vector3f &tc = Vector3f(1., 1., 1.) ) {
        t = _t;
        material = m;
        normal = n;
        texture_color = tc;
    }

private:
    double t;
    Material *material;
    Vector3f normal;
    Vector3f texture_color;

};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() <<">";
    return os;
}

#endif // HIT_H
