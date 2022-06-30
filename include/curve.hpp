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



    int n;
    std::vector<double> t;
    double ymin, ymax, r;
};

class BezierCurve : public Curve {
   public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        assert(false);
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
        for (int i = 0; i < n + k + 1; i++) 
            t.push_back(double(i) / double(n + k));
        range_min = t[k];
        range_max = t[n];
    }
    
    void calc_base_func(vector<double> &N, vector<double> &dN, int base_pos, double mu) {
        N[k] = 1;
        for (int p = 1; p <= k; ++p) {  // p as in DeBoor-Cox Algorithm
            for (int iter = k - p; iter < k + 1; ++iter) {
                int i = iter + base_pos - k;
                double p1, d1, p2, d2;
                    p1 = (mu - t[i]) / (t[i + p] - t[i]);
                    d1 = 1.0 / (t[i + p] - t[i]);
                    p2 = (t[i + p + 1] - mu) / (t[i + p + 1] - t[i + 1]);
                    d2 = -1 / (t[i + p + 1] - t[i + 1]);
                if (p == k) 
                    dN[iter] = (d1 * N[iter] + d2 * N[iter + 1]) * p;
                N[iter] = p1 * N[iter] + p2 * N[iter + 1];
            }
        }
        N.pop_back();
    }

    CurvePoint getPoint(double mu) {
        CurvePoint pt;
        if (mu < range_min || mu >= range_max)
            return pt;
        int base_pos = upper_bound(t.begin(), t.end(), mu) - t.begin() - 1;
        vector<double> N(k + 2, 0), dN(k + 1, 1);
        calc_base_func(N, dN, base_pos, mu);
        for (int j = 0; j < N.size(); ++j) {
            pt.V += controls[j + base_pos - k] * N[j];
            pt.T += controls[j + base_pos - k] * dN[j];
        }
        return pt;
    }

    double range_min, range_max;
    const int k = 3;
};

#endif  // CURVE_HPP

