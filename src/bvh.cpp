#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>
#include <algorithm>

using namespace std;

namespace CMU462 { namespace StaticScene {
    
    /*struct compare{
        compare(size_t axis) {this->axis = axis;}
        bool operator()(const Primitive* m, const Primitive* n){
            return (((m->get_bbox()).centroid())[axis] < ((n->get_bbox()).centroid())[axis]);
        }
        size_t axis;
    };*/
    
    struct part{
        part(size_t axis, double partPos) {this->axis = axis; this->partPos = partPos;}
        bool operator()(const Primitive* m){
            return (((m->get_bbox()).centroid())[axis] < partPos);
        }
        size_t axis;
        double partPos;
    };
    
    size_t bucketNum = 10;
    
    void BVHAccel::buildBVH(const BBox bb, BVHNode *top, const size_t bucketNum, const size_t max_leaf_size){
        
        size_t start = top->start;
        size_t range = top->range;
        
        if ((range <= max_leaf_size)) return;
        
        int isAxis = -1; //determine the axis
        Vector3D min = bb.min;
        
        double s_area = INF_D;
        double a_area, b_area;
        
        BBox lb, rb, lbb, rbb;

        double partPos; // final partition position
        
        size_t lrange, mid; // temporary and final left range
        
        
        //compute buckets for every axis
        for(size_t axis = 0; axis < 3; axis++){
            
            double bucketLen = bb.extent[axis] / (double)bucketNum;
            
            //for every partition
            for(size_t i=0; i<bucketNum-1; i++){
                double partitionPos = min[axis] + bucketLen * (i+1);
                
                lrange = 0;
                a_area = INF_D;
                
                lbb = BBox();
                rbb = BBox();
                
                //for every primitive
                for(size_t j=0; j<range; j++){
                    BBox currentBBox = primitives[j+start]->get_bbox();
                    if(currentBBox.centroid()[axis] < partitionPos){
                        lbb.expand(currentBBox);
                        lrange++;
                    }else{
                        rbb.expand(currentBBox);
                    }
                }
                
                if(lrange!=0 && (range-lrange)!=0){
                    a_area = lrange * lbb.surface_area();
                    b_area = (range-lrange) * rbb.surface_area();
                    if((a_area+b_area) < s_area){
                        s_area = a_area + b_area;
                        lb = lbb;
                        rb = rbb;
                        isAxis = axis;
                        mid = lrange;
                        partPos = partitionPos;
                    }
                }
            }
        }
        
        if(isAxis==(-1)) return;
        
        std::vector<Primitive *>::iterator m = std::partition(primitives.begin()+start, primitives.begin()+start+range, part(isAxis, partPos));
        
        top->l = new BVHNode(lb, start, mid);
        top->r = new BVHNode(rb, mid + start, range - mid);
                
        buildBVH( lb, top->l, bucketNum, max_leaf_size);
        buildBVH( rb, top->r, bucketNum, max_leaf_size);
    }
    
BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  this->primitives = _primitives;

  // TODO:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bb;
  for (size_t i = 0; i < primitives.size(); ++i) {
    bb.expand(primitives[i]->get_bbox());
  }

  root = new BVHNode(bb, 0, primitives.size());

    buildBVH( bb, root, bucketNum, max_leaf_size);
}
    

    
void deleteNode(BVHNode* node){
    if(node->isLeaf()) free(node);
    else {
        if(node->l) deleteNode(node->l);
        if(node->r) deleteNode(node->r);
    }
}

BVHAccel::~BVHAccel() {

  // TODO:
  // Implement a proper destructor for your BVH accelerator aggregate
    deleteNode(root);
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}
    
bool test(const Ray &ray, BVHNode* node, const std::vector<Primitive *>& primitives){
    
    if(node->isLeaf()){
        for(size_t m=0; m<node->range; m++){
                if(primitives[m+node->start]->intersect(ray)) return true;
        }
        return false;
    }else if(node->l == NULL) return test(ray, node->r, primitives);
    else if(node->r == NULL) return test(ray, node->l, primitives);
    else{
        double lt0 = ray.min_t, lt1 = ray.max_t;
        double rt0 = ray.min_t, rt1 = ray.max_t;
        bool lhit = node->l->bb.intersect(ray, lt0, lt1);
        bool rhit = node->r->bb.intersect(ray, rt0, rt1);
        if(!lhit && !rhit) return false;
        else if(lhit) return test(ray, node->l, primitives);
        else if(rhit) return test(ray, node->r, primitives);
        else return test(ray, node->l, primitives) || test(ray, node->r, primitives);
    }
    return false;
}
    
bool test(const Ray &ray, Intersection *i, BVHNode* node, const std::vector<Primitive *>& primitives){
    
    if(node->isLeaf()){
        bool hit = false;
        for(size_t m=0; m<node->range; m++){
            if(primitives[m+node->start]->intersect(ray, i)) hit = true;
        }
        return hit;
    }else if(node->l == NULL) return test(ray, i, node->r, primitives);
    else if(node->r == NULL) return test(ray, i, node->l, primitives);
    else{
        double lt0 = ray.min_t, lt1 = ray.max_t;
        double rt0 = ray.min_t, rt1 = ray.max_t;
        
        bool lhit = node->l->bb.intersect(ray, lt0, lt1);
        bool rhit = node->r->bb.intersect(ray, rt0, rt1);
        
        if(!lhit && !rhit) return false;
        else if(!lhit) return test(ray, i, node->r, primitives);
        else if(!rhit) return test(ray, i, node->l, primitives);
        else{
            BVHNode* first = (lt0 <= rt0) ? (node->l):(node->r);
            BVHNode* second = (lt0 <= rt0) ? (node->r):(node->l);
            bool l_hit, r_hit;
            l_hit = test(ray, i, first, primitives);
            r_hit = false;
            if(!l_hit || std::max(lt0, rt0) < i->t){
                r_hit = test(ray, i, second, primitives);
            }
            return l_hit || r_hit;
        }
    }
    return false;
}

bool BVHAccel::intersect(const Ray &ray) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.
    
    return test(ray, root, primitives);

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

    return test(ray, i, root, primitives);

}

}  // namespace StaticScene
}  // namespace CMU462
