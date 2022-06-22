#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include "texture.hpp"
#include <iostream>

// TODO: Implement Shade function that computes Phong introduced in class.
enum TYPE {SPEC, DIFF, REFR};
class Material {
public:

    TYPE type;
    double refractive_rate;
    Vector3f color, emission;
    Texture* texture;
    
    explicit Material(const TYPE _type, const Vector3f &_color = Vector3f::ZERO, const Vector3f &_emission = Vector3f::ZERO, double _refractive_rate = 0, Texture *_tex = nullptr) :
            type(_type), color(_color), emission(_emission), refractive_rate(_refractive_rate), texture(_tex){

    }

    virtual ~Material(){
        delete texture;
    };

    // virtual Vector3f getDiffuseColor() const {
    //     return diffuseColor;
    // }

    double clamp(double a) {
        return a > 0 ? a : 0;
    }

    // Vector3f Shade(const Ray &ray, const Hit &hit,
    //                const Vector3f &dirToLight, const Vector3f &lightColor) {
    //     Vector3f shaded = Vector3f::ZERO;
    //     // 
    //     Vector3f norm = hit.getNormal();
    //     Vector3f reflectionDir = (2 * (Vector3f::dot(norm, dirToLight))) * norm - dirToLight;
    //     shaded = diffuseColor * clamp(Vector3f::dot(dirToLight, norm))
    //            + specularColor * pow(clamp(Vector3f::dot(-ray.getDirection(), reflectionDir)), shininess);
    //     return lightColor * shaded;
    // }

protected:
    // Vector3f diffuseColor;
    // Vector3f specularColor;
    // double shininess;

    

};


#endif // MATERIAL_H
