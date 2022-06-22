#pragma once

#include <vecmath.h>
#include "image.hpp"
#include <fstream>

class Texture {
private:
    Image *image;
public:
    Texture(const char *filename) {
        image = Image::LoadBMP(filename);
    }
    ~Texture() {
        delete image;
    }
    int width() { return image->Width(); }
    int height() { return image->Height(); }
    Vector3f get_texel(int u, int v) {
        assert(u >= 0 && u < width() && v >= 0 && v < height());
        return image->GetPixel(u, v); 
    }
};