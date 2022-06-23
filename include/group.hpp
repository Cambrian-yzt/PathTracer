#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "mesh.hpp"
#include "triangle.hpp"
#include "plane.hpp"
#include "sphere.hpp"
#include "transform.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include "kd_tree.hpp"
#include <iostream>
#include <vector>


static Triangle* de_transform_ize(Triangle* original, std::vector<Transform*> *transform_stack);
// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group(): Object3D(nullptr, Group_T) {
        data.clear();
        kd_excluded_objs.clear();
        kd_tree = nullptr;
    }

    explicit Group (int num_objects): Object3D(nullptr, Group_T) {
        data.clear();
        kd_excluded_objs.clear();
        kd_tree = nullptr;
    }

    ~Group() override {
        for (auto obj: data) {
            delete obj;
        }
        data.clear();
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        // bool naive_doesIntersect = false;
        // for(auto obj : this -> data) {
        //     if(obj->intersect(r, h, tmin))
        //         naive_doesIntersect = true;
        // }
        // for(auto obj : this -> data) {
        //     if(obj->object_type != Triangle_T)
        //     if(obj->intersect(r, h, tmin))
        //         naive_doesIntersect = true;
        // }
        // double nt = h.getT();

        // h.set(1e20, nullptr, Vector3f(0, 0, 1));
        bool does_intersect = false;
        // for(auto obj : this -> data) {
        //     if(obj->object_type != Triangle_T && obj->object_type != Transform_T && obj->object_type != Mesh_T) {
        //         if(obj->intersect(r, h, tmin))
        //             does_intersect = true;
        //         // cout << "good: " << obj << endl;
        //     }
        // }
        // for (auto excluded_obj: kd_excluded_objs) {
        //     if (excluded_obj->intersect(r, h, tmin))
        //         does_intersect = true;
        // }
        // h.print();
        // if (kd_tree != nullptr && kd_tree->intersect(r, h, tmin))
        //     does_intersect = true;
        // h.print();

        // double nt = h.getT();

        // h.set(1e20, nullptr, Vector3f(0, 0, 1));

        for (auto excluded_obj: kd_excluded_objs) {
            if (excluded_obj->intersect(r, h, tmin))
                does_intersect = true;
        }
                
        if (kd_tree != nullptr && kd_tree->intersect(r, h, tmin))
            does_intersect = true;
        // h.print();

        // h.print();

        // if (fabs(nt - h.getT()) > 1e-8) {
        //     cout << nt << "  "<< h.getT() << endl;
        //     assert(false);
        // }

        return does_intersect;
    }

    void addObject(int index, Object3D *obj) {
        data.insert(data.begin() + index, obj);
    }

    int getGroupSize() {
        return data.size();
    }

    void discretize(std::vector<Object3D*> *discretized_objs) {
        std::vector<Transform*> transform_stack;
        // cout << this->data.size() << endl;
        discretize(this, discretized_objs, &transform_stack);
        // cout << discretized_objs->size() << endl;
        cout << kd_excluded_objs.size() << endl;
        // for (auto obj: kd_excluded_objs) {
        //     cout << obj << endl;
        // }
        assert(transform_stack.size() == 0);
    }

    std::vector<Object3D*> data;
    std::vector<Object3D*> kd_excluded_objs;
    KdTree *kd_tree;

    void build_kd_tree() {
        std::vector<Object3D*> discretized_objs;
        this->discretize(&discretized_objs);
        if (discretized_objs.size() != 0)
            this->kd_tree = new KdTree(discretized_objs);
    }

private:
    
    
    void discretize(Object3D *complex_obj, std::vector<Object3D*> *discretized_objs, std::vector<Transform*> *transform_stack) {
        if (complex_obj->object_type == Group_T) {
            puts("from group");
            auto group = dynamic_cast<Group*>(complex_obj);
            for(auto obj: group->data) {
                if (obj->object_type == Group_T) {
                    puts("gr");
                    discretize(obj, discretized_objs, transform_stack);
                } else if (obj->object_type == Mesh_T) {
                    puts("ms");
                    auto mesh = dynamic_cast<Mesh*>(obj);
                    for (auto triIndex: mesh->t) {
                        Vector3f a = mesh->v[triIndex[0]];
                        Vector3f b = mesh->v[triIndex[1]];
                        Vector3f c = mesh->v[triIndex[2]];
                        if (transform_stack->size() > 0) {
                            discretized_objs->push_back(de_transform_ize(new Triangle(a, b, c, mesh->material), transform_stack));
                        } else
                            discretized_objs->push_back(new Triangle(a, b, c, mesh->material));
                    }
                } else if (obj->object_type == Transform_T) {
                    puts("tran");
                    auto transform = dynamic_cast<Transform*>(obj);
                    transform_stack->push_back(transform);
                    discretize(transform, discretized_objs, transform_stack);
                    transform_stack->pop_back();
                } else if (obj->object_type == Triangle_T) {
                    puts("tri");
                    discretized_objs->push_back(obj);
                } else if (obj->object_type == Plane_T) {
                    puts("pl");
                    kd_excluded_objs.push_back(obj);
                } else if (obj->object_type == Rectangle_T) {
                    puts("re");
                    kd_excluded_objs.push_back(obj);
                } else if (obj->object_type == Sphere_T) {
                    puts("sph");
                    kd_excluded_objs.push_back(obj);
                } else {
                    assert(false);
                }
            }
        } else if (complex_obj->object_type == Transform_T) {
            //puts("from transform");
            auto obj = dynamic_cast<Transform*>(complex_obj)->get_obj();
            if (obj->object_type == Group_T) {
                puts ("tg");
                discretize(obj, discretized_objs, transform_stack);
            } else if (obj->object_type == Mesh_T) {
                puts ("tm");
                auto mesh = dynamic_cast<Mesh*>(obj);
                for (auto triIndex: mesh->t) {
                    Vector3f a = mesh->v[triIndex[0]];
                    Vector3f b = mesh->v[triIndex[1]];
                    Vector3f c = mesh->v[triIndex[2]];
                    if (transform_stack->size() > 0) {
                        discretized_objs->push_back(de_transform_ize(new Triangle(a, b, c, mesh->material), transform_stack));
                    } else
                        discretized_objs->push_back(new Triangle(a, b, c, mesh->material));
                }
            } else if (obj->object_type == Transform_T) {
                puts ("tt");
                auto transform = dynamic_cast<Transform*>(obj);
                transform_stack->push_back(transform);
                discretize(transform, discretized_objs, transform_stack);
                transform_stack->pop_back();
            } else if (obj->object_type == Triangle_T) {
                puts ("ttr");
                discretized_objs->push_back(obj);
            } else if (obj->object_type == Plane_T) {
                puts ("tp");
                kd_excluded_objs.push_back(obj);
            } else if (obj->object_type == Rectangle_T) {
                puts ("trec");
                kd_excluded_objs.push_back(obj);
            } else if (object_type == Sphere_T) {
                puts ("tsph");
                kd_excluded_objs.push_back(obj);
            } else {
                assert(false);
            }
        }
        else {
            assert(false);
        }
    }
};

// de-transform-ization currently available for triangles
// returns a new triangle, remember to delete!
static Triangle* de_transform_ize(Triangle* original, std::vector<Transform*> *transform_stack) {
    Triangle *tri = new Triangle(original->vertices[0], original->vertices[1], original->vertices[2], original->material);
    for (int i = transform_stack->size() - 1; i >= 0; i--) {
        Transform *t = transform_stack->at(i);
        (*tri) = Triangle(
            t->transformedPoint(tri->vertices[0]),
            t->transformedPoint(tri->vertices[1]),
            t->transformedPoint(tri->vertices[2]),
            tri->material
        );
    }
    return tri;
}

#endif
	
