#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP
// ²ÎÊýÇúÃæ
#include <tuple>

#include "curve.hpp"
#include "object3d.hpp"
#include "triangle.hpp"
#include <iostream>

const int MAX_NEWTON_ITERATIONS = 10;
const double NEWTON_EPSILON = 1e-5;

class RevSurface : public Object3D {
private:
    BsplineCurve *pCurve;
    Vector3f box_min, box_max;

public:
    RevSurface(BsplineCurve *pCurve, Material *m) : pCurve(pCurve), Object3D(m, RevSufrace_T) {
        for (auto &ctrl_pnt: pCurve->getControls()) {
            assert(ctrl_pnt.z() == 0.0);
        }
        box_min = Vector3f(-pCurve->r, pCurve->ymin, -pCurve->r);
        box_max = Vector3f(pCurve->r, pCurve->ymax, pCurve->r);
    }

    ~RevSurface() override { delete pCurve; }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        // puts("revsurface intersect");
        double t_ray, theta, mu;

        const double epsilon = 1e-8;
        const double MAX = std::numeric_limits<double>::max();
        double tx1 = -MAX, tx2 = MAX, ty1 = -MAX, ty2 = MAX, tz1 = -MAX, tz2 = MAX;
        if (fabs(r.getDirection().x()) > epsilon) {
            tx1 = (box_min.x() - (r.getOrigin().x())) / (r.getDirection().x());
            tx2 = (box_max.x() - (r.getOrigin().x())) / (r.getDirection().x());
            if (tx1 > tx2)
                std::swap(tx1, tx2);
        }
        if (fabs(r.getDirection().y()) > epsilon) {
            ty1 = (box_min.y() - (r.getOrigin().y())) / (r.getDirection().y());
            ty2 = (box_max.y() - (r.getOrigin().y())) / (r.getDirection().y());
            if (ty1 > ty2)
                std::swap(ty1, ty2);
        }
        if (fabs(r.getDirection().z()) > epsilon) {
            tz1 = (box_min.z() - (r.getOrigin().z())) / (r.getDirection().z());
            tz2 = (box_max.z() - (r.getOrigin().z())) / (r.getDirection().z());
            if (tz1 > tz2)
                std::swap(tz1, tz2);
        }
        double t_min = std::max(tx1, std::max(ty1, tz1));
        double t_max = std::min(tx2, std::min(ty2, tz2));
        if (t_min > t_max || t_min > h.getT())
            return false;
        if (t_min < 0) t_min = 0;
        // cout << t_min << " " << t_max << endl;
        // box_min.print();
        // box_max.print();
        // r.getOrigin().print();
        // r.getDirection().print();
        // puts("===========================");

        // puts("guessing parameters");
        bool hit = false;
        for (int sample = 0; sample < 20; sample++) {
            if (t_min > epsilon)
                t_ray = (double(sample) / 19) * (t_min);
            else if (t_max > 0) {
                t_ray = 0;
            }
            else return false;
            // else if (t_max > 0)
            //     t_ray = (double(sample) / 19) * (t_max);
            // else
            //     return false;
            Vector2f params = guessParameters(r, t_ray);
            theta = params.x();
            mu = params.y();
            // if (!pCurve->check_valid(mu))
            //     return false;
            // cout << mu << " " << theta << endl;
            Vector3f dm, dt, pnt_surface, normal;
            for (int iter = 0; iter < MAX_NEWTON_ITERATIONS; iter++) {
                if (theta < 0.)
                    theta += 2000000. * M_PI;
                if (theta >= 2. * M_PI)
                    theta = fmod(theta, 2. * M_PI);
                if (mu >= 1.)
                    mu = 1. - epsilon;
                if (mu <= 0.)
                    mu = 0. + epsilon;
                pnt_surface = get_point(theta, mu, dt, dm);
                normal = Vector3f::cross(dm, dt);
                Vector3f err = r.pointAtParameter(t_ray) - pnt_surface;
                if (!isnormal(mu) || !isnormal(theta) || !isnormal(t_ray)) break;
                if (err.squaredLength() < NEWTON_EPSILON) {
                    if (t_ray < tmin || mu < pCurve->range_min || mu > pCurve->range_max || t_ray > h.getT())
                        break;
                    h.set(t_ray, Object3D::material, normal.normalized(), get_texel(Vector3f(theta / (2 * M_PI), mu, 0)));
                    hit = true;
                }
                double d = Vector3f::dot(r.getDirection(), normal);
                t_ray -= Vector3f::dot(dm, Vector3f::cross(dt, err)) / d;
                mu -= Vector3f::dot(r.getDirection(), Vector3f::cross(dt, err)) / d;
                theta += Vector3f::dot(r.getDirection(), Vector3f::cross(dm, err)) / d;
            }
        }

        // exit(0);
        return hit;
    }
    Vector2f guessParameters(const Ray &r, const double t_ray) {
        Vector3f pnt_ray = r.pointAtParameter(t_ray);
        return Vector2f(
            atan2(-pnt_ray.z(), pnt_ray.x()) + M_PI,
            (pCurve->ymax - pnt_ray.y()) / (pCurve->ymax - pCurve->ymin)
        ); // .x() -> theta, .y() -> mu
    }
    Vector3f get_point(const double theta, const double mu, Vector3f &dt, Vector3f &dm) {
        Vector3f pnt_surface;
        Quat4f rotation;
        rotation.setAxisAngle(theta, Vector3f(0., 1., 0.));
        CurvePoint pnt_curve = pCurve->getPoint(mu);
        Matrix3f rotation_mat = Matrix3f::rotation(rotation);
        pnt_surface = rotation_mat * pnt_curve.V;
        dm = rotation_mat * pnt_curve.T;
        dt = Vector3f(-pnt_curve.V.x() * sin(theta), 0, -pnt_curve.V.x() * cos(theta));
        // cout << "dt ="; dt.print();
        // cout << "dm ="; dm.print();
        // cout << "pnt =";pnt_surface.print();
        return pnt_surface;
    }

    Vector3f get_texel(Vector3f hit_point, double time = 0.0) override {
        // hit_point.print();
        // return Vector3f(1., 1., 1.);
        // puts("getting texel");
        if (Object3D::material->texture == nullptr)
            return Vector3f(1., 1., 1.);
        return Object3D::material->texture->get_texel(hit_point.x(), hit_point.y());
    }
};

#endif