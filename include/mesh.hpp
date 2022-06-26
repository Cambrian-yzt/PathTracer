#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"


class Mesh : public Object3D {

public:
    Mesh(const char *filename, Material *m, bool smooth);

    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    std::vector<Vector3f> n;
    bool intersect(const Ray &r, Hit &h, double tmin) override;
    void update_extents(const Vector3f &vec);
    void computeNormal();
    void compute_bounding_sphere(void);
    void compute_vertex_normal(void);
    double xmin, xmax, ymin, ymax, zmin, zmax;
    double bounding_sphere_radius;
    bool smooth;  // 是否开启法向插值平滑效果
    std::vector<Vector3f> vnorms;
    Vector3f bounding_sphere_center;

private:

    // Normal can be used for light estimation
};

#endif
