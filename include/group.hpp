#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>


// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {
        data.clear();
    }

    explicit Group (int num_objects) {
        data.clear();
    }

    ~Group() override {

    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        bool doesIntersect = false;
        for(auto obj : this -> data) {
            if(obj->intersect(r, h, tmin))
                doesIntersect = true;
        }
        return doesIntersect;
    }

    void addObject(int index, Object3D *obj) {
        data.insert(data.begin() + index, obj);
    }

    int getGroupSize() {
        return data.size();
    }

private:
    std::vector<Object3D*> data;
};

#endif
	
