#include "mesh.hpp"
#include "sphere.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>


bool Mesh::intersect(const Ray &r, Hit &h, double tmin) {
    // 构造包围盒的三对平面
    const double epsilon = 10e-8;
    const double MAX = std::numeric_limits<double>::max();
    double tx1 = MAX, tx2 = -MAX, ty1 = MAX, ty2 = -MAX, tz1 = MAX, tz2 = -MAX;
    if (r.getDirection().x() != 0) {
        tx1 = (xmin - (r.getOrigin().x())) / (r.getDirection().x());
        tx2 = (xmax - (r.getOrigin().x())) / (r.getDirection().x());
        if (tx1 > tx2)
            swap(tx1, tx2);
    }
    if (r.getDirection().y() != 0) {
        ty1 = (ymin - (r.getOrigin().y())) / (r.getDirection().y());
        ty2 = (ymax - (r.getOrigin().y())) / (r.getDirection().y());
        if (ty1 > ty2)
            swap(ty1, ty2);
    }
    if (r.getDirection().z() != 0) {
        tz1 = (zmin - (r.getOrigin().z())) / (r.getDirection().z());
        tz2 = (zmax - (r.getOrigin().z())) / (r.getDirection().z());
        if (tz1 > tz2)
            swap(tz1, tz2);
    }
    double t_min = std::max(tx1, std::max(ty1, tz1));
    double t_max = std::min(tx2, std::min(ty2, tz2));
    if (t_min >= t_max)  // 光线包围盒不相交，不用继续向下走了
    {
        // printf("missed bunny\n");
        return false;
    }
    Sphere bounding_sphere = Sphere(bounding_sphere_center, bounding_sphere_radius, material);
    Hit bshit;
    if (!bounding_sphere.intersect(r, bshit, epsilon)) {  // 光线与包围球不相交
        return false;
    }
    
    // Optional: Change this brute force method into a faster one.
    bool result = false;
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        Triangle triangle(v[triIndex[0]],
                          v[triIndex[1]], v[triIndex[2]], material);
        triangle.normal = n[triId];
        result |= triangle.intersect(r, h, tmin);
    }
    return result;
}

void Mesh::update_extents(const Vector3f &vec) {
    if (vec.x() > xmax)
        xmax = vec.x();
    if (vec.x() < xmin)
        xmin = vec.x();
    if (vec.y() > ymax)
        ymax = vec.y();
    if (vec.y() < ymin)
        ymin = vec.y();
    if (vec.z() > zmax)
        zmax = vec.z();
    if (vec.z() < zmin)
        zmin = vec.z();
}

Mesh::Mesh(const char *filename, Material *material, bool smooth) : Object3D(material, Mesh_T), smooth(smooth) {
    xmin = std::numeric_limits<double>::max();
    xmax = -std::numeric_limits<double>::max();
    ymin = std::numeric_limits<double>::max();
    ymax = -std::numeric_limits<double>::max();
    zmin = std::numeric_limits<double>::max();
    zmax = -std::numeric_limits<double>::max();
    // Optional: Use tiny obj loader to replace this simple one.
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    char bslash = '/', space = ' ';
    std::string tok;
    int texID;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;
        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            update_extents(vec);
            v.push_back(vec);
            vnorms.push_back(Vector3f::ZERO);
        } else if (tok == fTok) {
            if (line.find(bslash) != std::string::npos) {
                std::replace(line.begin(), line.end(), bslash, space);
                std::stringstream facess(line);
                TriangleIndex trig;
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> trig[ii] >> texID;
                    trig[ii]--;
                }
                t.push_back(trig);
            } else {
                TriangleIndex trig;
                for (int ii = 0; ii < 3; ii++) {
                    ss >> trig[ii];
                    trig[ii]--;
                }
                t.push_back(trig);
            }
        } else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
        }
    }
    computeNormal();
    compute_bounding_sphere();
    compute_vertex_normal();
    printf("extents: x(%.4lf, %.4lf), y(%.4lf, %.4lf), z(%.4lf, %.4lf)\n", xmin, xmax, ymin, ymax, zmin, zmax);
    printf("bounding sphere: c(%4lf, %4lf, %4lf), r = %4lf\n", bounding_sphere_center.x(), bounding_sphere_center.y(), bounding_sphere_center.z(), bounding_sphere_radius);
    f.close();
}

void Mesh::compute_bounding_sphere() {
    if (v.size() == 0) {
        bounding_sphere_center = Vector3f::ZERO;
        bounding_sphere_radius = 0.0;
        return;
    }
    // Ritter's Bounding Algorithm
    Vector3f x = v[0];
    Vector3f y = v[0];
    for (auto vertex: v) {
        if ((vertex - x).length() > (y - x).length())
            y = vertex;
    }
    Vector3f z = y;
    for (auto vertex: v) {
        if ((vertex - y).length() > (z - y).length())
            z = vertex;
    }
    bounding_sphere_center = (y + z) / 2;
    bounding_sphere_radius = (y - z).length() / 2;
    for (auto vertex: v) {
        if ((vertex - bounding_sphere_center).length() > bounding_sphere_radius) {
            bounding_sphere_radius = (vertex - bounding_sphere_center).length();
        }
    }
}

void Mesh::computeNormal() {
    n.resize(t.size());
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        // 计算法向量
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        n[triId] = b / b.length();
    }
}

void Mesh::compute_vertex_normal() {
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        vnorms[triIndex[0]] += n[triId];
        vnorms[triIndex[1]] += n[triId];
        vnorms[triIndex[2]] += n[triId];
    } 
    
    for (int i = 0; i < v.size(); i++) {
        vnorms[i].normalize();
    }
}