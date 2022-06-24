#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>
#include <cmath>
#include <cstdio>

#include <algorithm>

// TODO (PA2): Implement Bernstein class to compute spline basis function.
//       You may refer to the python-script for implementation.

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent  (unit)
};

int comb[500][500] = {0};
int C(int n, int i) {
    if (i > n || i < 0 || n < 0)
        return 0;
    if (comb[n][i] != 0)
        return comb[n][i];
    if (i == 1 && n > 0)
        return comb[n][i] = n;
    if (n == 0)
        return comb[n][i] = 1;
    return comb[n][i] = C(n - 1, i - 1) + C(n - 1, i);
}


class Curve : public Object3D {
protected:
    std::vector<Vector3f> controls;
public:
    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {}

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }

    // virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;

};

class BezierCurve : public Curve {
public:
    double B(int i, int n, double t) {
        // printf("C(%d,%d) = %d\n", n, i, C(n, i));
        return double(C(n, i)) * pow(1 - t, n - i) * pow(t, i);
    }

    double B_derivitive(int i, int n, double t) {
        return double(n) * (B(i - 1, n - 1, t) - B(i, n - 1, t));
    }

    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }

    }

    // void discretize(int resolution, std::vector<CurvePoint>& data) override {
    //     data.clear();
    //     // TODO (PA2): fill in data vector
    //     int n = getControls().size() - 1;
    //     vector<Vector3f> P = getControls();
    //     for (double t = 0; t <= 1; t += (1.0 / resolution)) {
    //         CurvePoint cp;
    //         cp.V = Vector3f::ZERO;
    //         for (int i = 0; i <= n; i++)
    //             cp.V += P[i] * B(i, n, t);
    //         // printf("V %f %f %f\n", cp.V[0], cp.V[1], cp.V[2]);
    //         cp.T = Vector3f::ZERO;
    //         for (int i = 0; i <= n - 1; i++)
    //             cp.T += (n + 1) * B(i, n, t) * (P[i + 1] - P[i]);
    //         data.push_back(cp);
    //     }
    //     // printf("\n");
    // }

protected:

};

class BsplineCurve : public Curve {
public:
    double base_func[100][100];
    double T[100];
    double ymin, ymax, r;
    double range_min, range_max;
    void init() {
        int n = getControls().size() - 1;
        for (int i = 0; i < 100; i++) {
            for (int j = 0; j < 100; j++) {
                base_func[i][j] = 0;
             }
        }
        for (int i = 0; i <= n + k + 1; i++) {
            T[i] = double(i) / double(n + k + 1);
            // printf("%f ", T[i]);
        }
        // printf("\n");
        ymin = ymax = controls[0].y();
        r = 0;
        for (auto ctrl_pnt: controls) {
            r = max(r, double(max(fabs(ctrl_pnt.x()), fabs(ctrl_pnt.y()))));
            ymin = ctrl_pnt.y() < ymin? ctrl_pnt.y(): ymin;
            ymax = ctrl_pnt.y() > ymax? ctrl_pnt.y(): ymax;
        }
        range_min = T[k];
        range_max = T[n];
    }

    double B(int i, int p, double t) {
        int n = getControls().size() - 1;
        if (p == 0 && (t < T[i] || t >= T[i + 1]))
            return 0;
        if (p == 0)
            return 1;
        // if (base_func[i][p] != 0)
        //     return base_func[i][p];
        // else
        return (t - T[i]) / (T[i + p] - T[i]) * B(i, p - 1, t) + (T[i + p + 1] - t) / (T[i + p + 1] - T[i + 1]) * B(i + 1, p - 1, t);
    }
    double B_derivitive(int i, int p, int t) {
        return p * (B(i, p - 1, t) / (T[i + p] - T[i]) - B(i + 1, p - 1, t) / (T[i + p + 1] - T[i + 1]));
    }
    
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
        init();
    }

    CurvePoint getPoint(double mu) {
        CurvePoint ret;
        ret.T = ret.V = Vector3f::ZERO;
        int n = getControls().size() - 1;
        vector<Vector3f> P = getControls();
        for (int i = 0; i <= n; i++) {
            ret.V += P[i] * B(i, k, mu);
            ret.T += P[i] * B_derivitive(i, k, mu);
        }
        cout << mu;
        ret.T.print();
        ret.V.print();
        puts("======");
        return ret;
    }

    // void discretize(int resolution, std::vector<CurvePoint>& data) override {
    //     data.clear();
    //     // TODO (PA2): fill in data vector
    //     int n = getControls().size() - 1;
    //     vector<Vector3f> P = getControls();
    //     for (double t = T[k]; t <= T[n + 1]; t += ((T[n + 1] - T[n]) / double(resolution))) {
    //         init();
    //         CurvePoint cp;
    //         cp.V = Vector3f::ZERO;
    //         for (int i = 0; i <= n; i++)
    //             cp.V += P[i] * B(i, k, t);
    //         printf("V %f %f %f\n", cp.V[0], cp.V[1], cp.V[2]);
    //         cp.T = Vector3f::ZERO;
    //         for (int i = 0; i <= n; i++)
    //             cp.T += P[i] * B_derivitive(i, k ,t);
    //         data.push_back(cp);
    //     }
    //     printf("\n");
    // }

protected:
    const int k = 3;
};

#endif // CURVE_HPP
