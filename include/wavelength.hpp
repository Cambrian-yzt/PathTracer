#pragma once
/*
    进行波长到rgb的转换
*/

#include <vecmath.h>
#include <cmath>

double nm(double m) {
    return m * 1e9;
}

Vector3f wavelength_to_rgb(double wavelength, double gamma = 0.8) {
    // wavelength in meters, need to convert to nanometers
    double wl = nm(wavelength);
    double attenuation, r, g, b;
    if (wl >= 380 && wl <= 440) {
        attenuation = 0.3 + 0.7 * (wl - 380) / (440 - 380);
        r = pow(((-(wl - 440) / (440 - 380)) * attenuation) , gamma);
        g = 0.;
        b = pow(1. * attenuation, gamma);
    } else if (wl >= 440 && wl <= 490) {
        r = 0.;
        g = pow((wl - 440) / 50., gamma);
        b = 1.;
    } else if (wl >= 490 && wl <= 510) {
        r = 0.;
        g = 1.;
        b = pow((510 - wl) / 20, gamma);
    } else if (wl >= 510 && wl <= 580) {
        r = pow((wl - 510) / 70, gamma);
        g = 1.;
        b = 0.;
    } else if (wl >= 580 && wl <= 645) {
        r = 1.;
        g = pow((645 - wl) / 65, gamma);
        b = 0.;
    } else if (wl >= 645 && wl <= 750) {
        attenuation = 0.3 + 0.7 * (750 - wl) / 105;
        r = pow(attenuation, gamma);
        g = 0.;
        b = 0.;
    } else {
        r = 0.;
        g = 0.;
        b = 0.;
    }
    if (r > 1.) r = 1.;
    if (g > 1.) g = 1.;
    if (b > 1.) b = 1.;
    return Vector3f(r, g, b);
}

double generate_wavelength(unsigned short *randState) {
    // 近似得到正午阳光的光谱分布
    if (erand48(randState) < 0.5) {
        return (erand48(randState) * (580 - 380) + 380) * 1e-9;
    } else {
        return (erand48(randState) * (750 - 380) + 380) * 1e-9;
    }
}