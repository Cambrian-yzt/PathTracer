#pragma once

#include <vecmath.h>
#include "image.hpp"
#include <fstream>

class Texture {
private:
    Image *image;
    Image *normal_map;
    bool _is_normal;
public:
    Texture(const char *filename, bool _is_normal = false, const char *_normal_map = nullptr): _is_normal(_is_normal) {
        image = Image::LoadBMP(filename);
        if (_is_normal)
            normal_map = Image::LoadBMP(_normal_map);
    }
    ~Texture() {
        delete image;
    }
    int width() { return image->Width(); }
    int height() { return image->Height(); }
    bool is_normal() { return this->_is_normal; }
    Vector3f get_texel(int u, int v) {
        assert(u >= 0 && u < width() && v >= 0 && v < height());
        return image->GetPixel(u, v); 
    }
    Vector3f get_normal(int u, int v) {
        assert(_is_normal && u >= 0 && u < width() && v >= 0 && v < height());
        return normal_map->GetPixel(u, v);
    }
};