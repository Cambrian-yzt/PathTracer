#pragma once

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <limits>

using namespace std;

const int MIN_RECURSION_NODE_SIZE = 8;


struct KdNode {
    void update_min_element(Vector3f &base, const Vector3f &candidate) {
        base.x() = min(base.x(), candidate.x());
        base.y() = min(base.y(), candidate.y());
        base.z() = min(base.z(), candidate.z());
    }

    void update_max_element(Vector3f &base, const Vector3f &candidate) {
        base.x() = max(base.x(), candidate.x());
        base.y() = max(base.y(), candidate.y());
        base.z() = max(base.z(), candidate.z());
    }
    int l, r, m1, m2;
    Vector3f box_min, box_max;
    KdNode *lchild = nullptr, *rchild = nullptr;
    KdNode() {}
    KdNode(vector<Object3D*> *tree_data, KdNode *parent, int l_bound, int r_bound, int depth) {
        // [l, m1), [m1, m2), [m2, r)
        printf("building node: %d %d\n", l_bound, r_bound);
        // cout << "tree_data = " << tree_data << endl;
        if (l_bound == r_bound - 1) {
            l = m1 = l_bound;
            m2 = r = r_bound;
            box_min = tree_data->at(l_bound)->box_min;
            box_max = tree_data->at(l_bound)->box_max;
        } else {
            this->box_min = tree_data->at(l_bound)->box_min;
            this->box_max = tree_data->at(l_bound)->box_max;
            // puts("getting min and max for this node");
            for (int i = l_bound + 1; i < r_bound; i++) {
                this->update_min_element(box_min, tree_data->at(i)->box_min);
                this->update_max_element(box_max, tree_data->at(i)->box_max);
            }
            // puts("finished getting min and max for this node:");
            Vector3f lchild_max = box_max;
            double divider = 0.5;
            double heuristic = 1e20;
            unsigned short rndstate[3] = {l * l, r * r, l * r};
            for (int rand_try = 0; rand_try < 30; rand_try++) {
                double rnd = erand48(rndstate);
                lchild_max[depth % 3] = (box_min[depth % 3] + box_max[depth % 3]) * rnd;
                vector<Object3D*> lbuf, mbuf, rbuf;
                lbuf.clear(); mbuf.clear(); rbuf.clear();
                // puts("dividing nodes");
                for (int i = l_bound; i < r_bound; i++) {
                    BoxState bs = tree_data->at(i)->check_box(box_min, lchild_max);
                    switch (bs) {
                        case IN: lbuf.push_back(tree_data->at(i)); break;
                        case ON: mbuf.push_back(tree_data->at(i)); break;
                        case OUT: rbuf.push_back(tree_data->at(i)); break;
                        default: assert(false);
                    };
                }
                double ls = lbuf.size() + 1, ms = mbuf.size() + 1, rs = rbuf.size() + 1;
                if (max(ls, rs) * 5 + ms * 95 < heuristic) {
                    heuristic = max(ls, rs) * 5 + ms * 95;
                    divider = rnd;
                }
            }
            lchild_max[depth % 3] = (box_min[depth % 3] + box_max[depth % 3]) * divider;
            vector<Object3D*> lbuf, mbuf, rbuf;
            lbuf.clear(); mbuf.clear(); rbuf.clear();
            for (int i = l_bound; i < r_bound; i++) {
                BoxState bs = tree_data->at(i)->check_box(box_min, lchild_max);
                switch (bs) {
                    case IN: lbuf.push_back(tree_data->at(i)); break;
                    case ON: mbuf.push_back(tree_data->at(i)); break;
                    case OUT: rbuf.push_back(tree_data->at(i)); break;
                    default: assert(false);
                };
            }
            l = l_bound;
            m1 = l_bound + lbuf.size();
            m2 = l_bound + lbuf.size() + mbuf.size();
            r = r_bound;
            assert(l_bound + lbuf.size() + mbuf.size() + rbuf.size() == r_bound);
            for (int i = l_bound; i < r_bound; i++) {
                if (i < m1)
                    (*tree_data)[i] = lbuf[i - l_bound];
                else if (i < m2)
                    (*tree_data)[i] = mbuf[i - m1];
                else
                    (*tree_data)[i] = rbuf[i - m2];
            }
        }
    }
};

class KdTree {
public:
    vector<Object3D*> discretized_objs;
    KdNode *root;
    void build(KdNode *cur_node, int depth) {
        if (cur_node->l + 1 >= cur_node->r)
            return;
        if (cur_node->l + 1 < cur_node->m1) {
            cur_node->lchild = new KdNode(&(this->discretized_objs), cur_node, cur_node->l, cur_node->m1, depth + 1);
            build(cur_node->lchild, depth + 1);
        }
        if (cur_node->m2 + 1 < cur_node->r) {
            cur_node->rchild = new KdNode(&(this->discretized_objs), cur_node, cur_node->m2, cur_node->r, depth + 1);
            build(cur_node->rchild, depth + 1);
        }
    }
    
    bool intersect(const Ray &ray, Hit &h, double tmin, KdNode* cur_node, int depth) {
        if (!ray.check_box_collision(cur_node->box_min, cur_node->box_max)) {
            return false;
        }
        if (cur_node->r - cur_node->l <= MIN_RECURSION_NODE_SIZE) {
            // printf("minimum node size reached, using linear check");
            bool ret = false;
            for (int i = cur_node->l; i < cur_node->r; i++) {
                if (discretized_objs[i]->intersect(ray, h, tmin))
                    ret = true;
            }
            return ret;
        }
        bool l_result = false, m_result = false, r_result = false;
        if (ray.getDirection()[depth % 3] > 0) {
            if (cur_node->lchild != nullptr)
                l_result = intersect(ray, h, tmin, cur_node->lchild, depth + 1);
            if (!l_result && cur_node->rchild != nullptr)
                r_result = intersect(ray, h, tmin, cur_node->rchild, depth + 1);
        }
        else if (ray.getDirection()[depth % 3] < 0){
            if (cur_node->rchild != nullptr)
                r_result = intersect(ray, h, tmin, cur_node->rchild, depth + 1);
            if (!r_result && cur_node->lchild != nullptr)
                l_result = intersect(ray, h, tmin, cur_node->lchild, depth + 1);
        }
        else {
            // ray is parallel to dividing plane, is this really possible, with all the randomization in Path Tracing and with floating point numbers?
            assert(false);
        }
        // printf("[%d, %d)\n", cur_node->l, cur_node->r);
        for (int i = cur_node->m1; i < cur_node->m2; i++) {
            if (discretized_objs[i]->intersect(ray, h, tmin))
                m_result = true;
        }
        return l_result || r_result || m_result;
    }

public:
    KdTree(vector<Object3D*> &disc_objs) {
        puts("begin building");
        assert(disc_objs.size() > 0);
        this->discretized_objs = disc_objs;
        this->root = new KdNode(&(this->discretized_objs), nullptr, 0, this->discretized_objs.size(), 0);
        build(root, 0);
    }

    ~KdTree() {
        for (auto obj: discretized_objs) {
            delete obj;
        }
        discretized_objs.clear();
    }
    
    bool intersect(const Ray &r, Hit &h, double tmin) {
        // puts("started kdtree intersect");
        assert(root != nullptr);
        return intersect(r, h, tmin, root, 0);
    }

};

