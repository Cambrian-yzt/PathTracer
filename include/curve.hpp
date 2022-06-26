// #ifndef CURVE_HPP
// #define CURVE_HPP

// #include "object3d.hpp"
// #include <vecmath.h>
// #include <vector>
// #include <utility>
// #include <cmath>
// #include <cstdio>

// #include <algorithm>

// // TODO (PA2): Implement Bernstein class to compute spline basis function.
// //       You may refer to the python-script for implementation.

// // The CurvePoint object stores information about a point on a curve
// // after it has been tesselated: the vertex (V) and the tangent (T)
// // It is the responsiblility of functions that create these objects to fill in all the data.
// struct CurvePoint {
//     Vector3f V; // Vertex
//     Vector3f T; // Tangent  (unit)
//     CurvePoint() {}
//     CurvePoint(Vector3f V, Vector3f T, bool valid = true): V(V), T(T){}
// };

// int comb[500][500] = {0};
// int C(int n, int i) {
//     if (i > n || i < 0 || n < 0)
//         return 0;
//     if (comb[n][i] != 0)
//         return comb[n][i];
//     if (i == 1 && n > 0)
//         return comb[n][i] = n;
//     if (n == 0)
//         return comb[n][i] = 1;
//     return comb[n][i] = C(n - 1, i - 1) + C(n - 1, i);
// }


// class Curve : public Object3D {
// protected:
//     std::vector<Vector3f> controls;
// public:
//     explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {}

//     bool intersect(const Ray &r, Hit &h, double tmin) override {
//         return false;
//     }

//     std::vector<Vector3f> &getControls() {
//         return controls;
//     }

//     // virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;

// };

// class BezierCurve : public Curve {
// public:
//     double B(int i, int n, double t) {
//         // printf("C(%d,%d) = %d\n", n, i, C(n, i));
//         return double(C(n, i)) * pow(1 - t, n - i) * pow(t, i);
//     }

//     double B_derivitive(int i, int n, double t) {
//         return double(n) * (B(i - 1, n - 1, t) - B(i, n - 1, t));
//     }

//     explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
//         if (points.size() < 4 || points.size() % 3 != 1) {
//             printf("Number of control points of BezierCurve must be 3n+1!\n");
//             exit(0);
//         }

//     }

//     // void discretize(int resolution, std::vector<CurvePoint>& data) override {
//     //     data.clear();
//     //     // TODO (PA2): fill in data vector
//     //     int n = getControls().size() - 1;
//     //     vector<Vector3f> P = getControls();
//     //     for (double t = 0; t <= 1; t += (1.0 / resolution)) {
//     //         CurvePoint cp;
//     //         cp.V = Vector3f::ZERO;
//     //         for (int i = 0; i <= n; i++)
//     //             cp.V += P[i] * B(i, n, t);
//     //         // printf("V %f %f %f\n", cp.V[0], cp.V[1], cp.V[2]);
//     //         cp.T = Vector3f::ZERO;
//     //         for (int i = 0; i <= n - 1; i++)
//     //             cp.T += (n + 1) * B(i, n, t) * (P[i + 1] - P[i]);
//     //         data.push_back(cp);
//     //     }
//     //     // printf("\n");
//     // }

// protected:

// };

// class BsplineCurve : public Curve {
// public:
//     double t[100];
//     double ymin, ymax, r;
//     double range_min, range_max;
//     int n;

//     double B(int i, int p, double mu) {
//         if (p == 0)
//             return (mu < t[i] || mu >= t[i + 1]) ? 0 : 1;
//         return (mu - t[i]) / (t[i + p] - t[i]) * B(i, p - 1, mu) + (t[i + p + 1] - mu) / (t[i + p + 1] - t[i + 1]) * B(i + 1, p - 1, mu);
//     }
//     double B_derivitive(int i, int p, int mu) {
//         return p * (B(i, p - 1, mu) / (t[i + p] - t[i]) - B(i + 1, p - 1, mu) / (t[i + p + 1] - t[i + 1]));
//     }

//     bool check_valid(double mu) {
//         return mu >= t[k] && mu <= t[n + 1];
//     }
    
//     BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
//         if (points.size() < 4) {
//             printf("Number of control points of BspineCurve must be more than 4!\n");
//             exit(0);
//         }
//         this->n = getControls().size() - 1;
//         for (int i = 0; i <= n + k + 1; i++) {
//             t[i] = double(i) / double(n + k + 1);
//         }
//         ymin = ymax = controls[0].y();
//         r = 0;
//         for (auto ctrl_pnt: controls) {
//             r = max(r, double(max(fabs(ctrl_pnt.x()), fabs(ctrl_pnt.y()))));
//             ymin = ctrl_pnt.y() < ymin? ctrl_pnt.y(): ymin;
//             ymax = ctrl_pnt.y() > ymax? ctrl_pnt.y(): ymax;
//         }
//         range_min = t[k];
//         range_max = t[n];
//     }

//     CurvePoint getPoint(double mu) {
//         CurvePoint ret;
//         ret.T = ret.V = Vector3f::ZERO;
//         vector<Vector3f> P = getControls();
//         for (int i = 0; i <= n; i++) {
//             ret.V += P[i] * B(i, k, mu);
//             ret.T += P[i] * B_derivitive(i, k, mu);
//         }
//         // cout << mu << endl;
//         // ret.T.print();
//         // ret.V.print();
//         // puts("======");
//         return ret;
//     }

//     // void discretize(int resolution, std::vector<CurvePoint>& data) override {
//     //     data.clear();
//     //     // TODO (PA2): fill in data vector
//     //     int n = getControls().size() - 1;
//     //     vector<Vector3f> P = getControls();
//     //     for (double t = T[k]; t <= T[n + 1]; t += ((T[n + 1] - T[n]) / double(resolution))) {
//     //         init();
//     //         CurvePoint cp;
//     //         cp.V = Vector3f::ZERO;
//     //         for (int i = 0; i <= n; i++)
//     //             cp.V += P[i] * B(i, k, t);
//     //         printf("V %f %f %f\n", cp.V[0], cp.V[1], cp.V[2]);
//     //         cp.T = Vector3f::ZERO;
//     //         for (int i = 0; i <= n; i++)
//     //             cp.T += P[i] * B_derivitive(i, k ,t);
//     //         data.push_back(cp);
//     //     }
//     //     printf("\n");
//     // }

// protected:
//     const int k = 3;
// };

// #endif // CURVE_HPP



#ifndef CURVE_HPP
#define CURVE_HPP

#include <vecmath.h>

#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>
#include "object3d.hpp"
using namespace std;
struct CurvePoint {
    Vector3f V;  // Vertex
    Vector3f T;  // Tangent  (unit)
};

class Curve {
   protected:
    std::vector<Vector3f> controls;

   public:
    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {
        ymin = 1e20;
        ymax = -1e20;
        r = 0;
        for (auto pt : controls) {
            ymin = std::min(pt.y(), ymin);
            ymax = std::max(pt.y(), ymax);
            r = std::max(r, fabs(pt.x()));
            r = std::max(r, fabs(pt.z()));
        }
    }

    std::vector<Vector3f> &getControls() { return controls; }

    CurvePoint getPoint(double mu) {
        CurvePoint pt;
        int bpos = upper_bound(t.begin(), t.end(), mu) - t.begin() - 1;
        vector<double> s(k + 2, 0), ds(k + 1, 1);
        s[k] = 1;
        for (int p = 1; p <= k; ++p) {
            for (int ii = k - p; ii < k + 1; ++ii) {
                int i = ii + bpos - k;
                double w1, dw1, w2, dw2;
                if (tpad[i + p] == tpad[i]) {
                    w1 = mu;
                    dw1 = 1;
                } else {
                    w1 = (mu - tpad[i]) / (tpad[i + p] - tpad[i]);
                    dw1 = 1.0 / (tpad[i + p] - tpad[i]);
                }
                if (tpad[i + p + 1] == tpad[i + 1]) {
                    w2 = 1 - mu;
                    dw2 = -1;
                } else {
                    w2 = (tpad[i + p + 1] - mu) /
                         (tpad[i + p + 1] - tpad[i + 1]);
                    dw2 = -1 / (tpad[i + p + 1] - tpad[i + 1]);
                }
                if (p == k) ds[ii] = (dw1 * s[ii] + dw2 * s[ii + 1]) * p;
                s[ii] = w1 * s[ii] + w2 * s[ii + 1];
            }
        }
        s.pop_back();
        int lsk = k - bpos, rsk = bpos + 1 - n;
        if (lsk > 0) {
            for (int i = lsk; i < s.size(); ++i) {
                s[i - lsk] = s[i];
                ds[i - lsk] = ds[i];
            }
            s.resize(s.size() - lsk);
            ds.resize(ds.size() - lsk);
            lsk = 0;
        }
        if (rsk > 0) {
            if (rsk < s.size()) {
                s.resize(s.size() - rsk);
                ds.resize(ds.size() - rsk);
            } else {
                s.clear();
                ds.clear();
            }
        }
        for (int j = 0; j < s.size(); ++j) {
            pt.V += controls[-lsk + j] * s[j];
            pt.T += controls[-lsk + j] * ds[j];
        }
        return pt;
    }

    void discretize(int resolution, std::vector<CurvePoint> &data) {
        resolution *= n / k;
        data.resize(resolution);
        for (int i = 0; i < resolution; ++i) {
            double mu =
                ((double)i / resolution) * (range_max - range_min) + range_min;
            data[i] = getPoint(mu);
        }
    }

    void pad() {
        int tSize = t.size();
        tpad.resize(tSize + k);
        for (int i = 0; i < tSize; ++i) tpad[i] = t[i];
        for (int i = 0; i < k; ++i) tpad[i + tSize] = t.back();
    }

    int n, k;
    std::vector<double> t;
    std::vector<double> tpad;
    double ymin, ymax, r;
    double range_min, range_max;
};

class BezierCurve : public Curve {
   public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
        n = controls.size();
        k = n - 1;
        range_min = 0;
        range_max = 1;
        t.resize(2 * n);
        for (int i = 0; i < n; ++i) {
            t[i] = 0;
            t[i + n] = 1;
        }
        pad();
    }
};

class BsplineCurve : public Curve {
   public:
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf(
                "Number of control points of BspineCurve must be more than "
                "4!\n");
            exit(0);
        }
        n = controls.size();
        k = 3;
        t.resize(n + k + 1);
        for (int i = 0; i < n + k + 1; ++i) t[i] = (double)i / (n + k);
        pad();
        range_min = t[k];
        range_max = t[n];
    }
};

#endif  // CURVE_HPP

