#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>
#include <iostream>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH, bool dispersion = false, double focal_length = -1.0, double aperture_size = -1.0) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
        this->focal_length = focal_length;
        this->aperture_size = aperture_size;
        this->dispersion = dispersion;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point, const double time, double wavelength = -1) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    double focal_length;
    double aperture_size;
    bool dispersion;
    bool scatter;
    double mist_extinction;
    double mist_scatter;
    double back_scatter;

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, double angle, bool dispersion = false, double focal_length = -1.0, double aperture_size = -1.0) : 
            Camera(center, direction, up, imgW, imgH, dispersion, focal_length, aperture_size) {
        // angle is in radian.
        this -> angle = angle;
        cx = imgW / 2;
        cy = imgH / 2;
        fx = imgW / (2 * tan(angle / 2));
        fy = fx;
    }

    Ray generateRay(const Vector2f &point, const double time, double wavelength = -1.0) override {
        // 
        Vector3f dir_rc = Vector3f((point[0] - cx) / fx, (cy - point[1]) / fy, 1.0);
        dir_rc.normalize();
        Matrix3f R = Matrix3f(this->horizontal, -this->up, this->direction);
        Vector3f dir_rw = R * dir_rc;
        dir_rw.normalize();
        // std::cout << wavelength << std::endl;
        if (wavelength < 0)
            return Ray(center, dir_rw, time);
        else return Ray(center, dir_rw, time, wavelength);
    }
protected:
    double angle;
    double cx;
    double cy;
    double fx;
    double fy;
};

#endif //CAMERA_H
