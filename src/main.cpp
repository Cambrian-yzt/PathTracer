#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "ray.hpp"

#include <string>
#include <chrono>

using namespace std;

const double epsilon = 1e-8;

double max_element(Vector3f vec) {
    return max(vec.x(), max(vec.y(), vec.z()));
}

// Vector3f to_int(Vector3f mc_radiance) {
//     mc_radiance.normalize();
//     return Vector3f(mc_radiance.x() * 255, mc_radiance.y() * 255, mc_radiance.z() * 255);
// }

Vector3f radiance(SceneParser* scene, const Ray &ray, int depth, unsigned short *randState) {
    depth++;
    double t;  // 交点在光线上的参数t
    Group *baseGroup = scene->getGroup(); 
    Hit hit;
    if (!(baseGroup->intersect(ray, hit, epsilon))) {  // 不与任何物体相交，返回黑色
        return Vector3f::ZERO;
    }
    t = hit.getT();
    Vector3f point = ray.pointAtParameter(t);  // 交点
    Vector3f normal = hit.getNormal();  // 法向量
    Vector3f nl = normal;
    if (Vector3f::dot(normal, ray.getDirection()) >= 0)  // 区分内外
        nl = -nl;  // oriented normal vector
    Material *hit_mat = hit.getMaterial();
    Vector3f f = hit_mat->color;
    double max_refl = max_element(f);
    if (depth > 5) {
        if (erand48(randState) >= max_refl)  // Russian Roulette
            return hit_mat->emission;
        f = f * (1. / max_refl);
    }
    if (hit_mat->type == DIFF) {  // 漫反射 diffuse
        double rnd1 = 2 * M_PI * erand48(randState);
        double rnd2 = erand48(randState);
        double rnd2_sqrt = sqrt(rnd2);
        Vector3f w = nl;
        Vector3f u = (Vector3f::cross((fabs(w.x()) > .1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w)).normalized();  // 浮点数精度
        Vector3f v = Vector3f::cross(w, u);
        //  dir: 合理均匀的漫反射采样：在半球面上单位表面积均匀分布
        Vector3f dir = (u * cos(rnd1) * rnd2_sqrt + v * sin(rnd1) * rnd2_sqrt + w * sqrt(1 - rnd2)).normalized();
        return (hit_mat->emission + f * radiance(scene, Ray(point, dir), depth, randState)) * hit.get_texture_color();
    } else if (hit_mat->type == SPEC) {  // 镜面反射 specular
        //  dir: 镜面反射直接算就行了，不需要采样
        Vector3f dir = (ray.getDirection() - normal * 2 * Vector3f::dot(normal, ray.getDirection())).normalized();
        return (hit_mat->emission + f * (radiance(scene, Ray(point, dir), depth, randState))) * hit.get_texture_color();
    } else if (hit_mat->type == REFR) {  // 折射 refraction 玻璃、水等
        Vector3f refl_dir = (ray.getDirection() - normal * 2 * Vector3f::dot(normal, ray.getDirection())).normalized();
        Ray refl_ray(point, refl_dir);
        // TODO: 三角面片法向量真的是向几何体外的吗？
        bool into = Vector3f::dot(normal, nl) > 0;
        const double refr_air = 1.0, refr_mat = hit_mat->refractive_rate;
        double nnt;
        // TODO: 从介质射入介质怎么办？
        if (into) {
            nnt = refr_air / refr_mat;
        } else {
            nnt = refr_mat / refr_air;
        }
        double ddn = Vector3f::dot(ray.getDirection(), nl);
        double cos2t = 1 - nnt * nnt * (1 - ddn * ddn);
        if (cos2t < 0){  // 全反射
            return (hit_mat->emission + f * radiance(scene, refl_ray, depth, randState)) * hit.get_texture_color();
        }
        Vector3f refr_dir = (ray.getDirection() * nnt - normal * ((into? 1: -1) * (ddn * nnt + sqrt(cos2t)))).normalized();
        Ray refr_ray = Ray(point, refr_dir);
        // 计算菲涅尔项（使用Schlick近似）
        double a = refr_mat - refr_air;
        double b = refr_mat + refr_air;
        double R0 = a * a / (b * b);
        double c = 1 - (into? -ddn: Vector3f::dot(refr_dir, normal));
        double Re = R0 + (1 - R0) * c * c * c * c * c;
        double Tr = 1 - Re;
        double P = 0.25 + 0.5 * Re;
        double RP = Re / P;
        double TP = Tr / (1 - P);
        Vector3f ret = hit_mat->emission;
        if (depth > 2) {
            if (erand48(randState) < P) {
                ret += (f * radiance(scene, refl_ray, depth, randState)) * RP;
            } else {
                ret += (f * radiance(scene, refr_ray, depth, randState)) * TP;
            }
        } else {
            ret += (f * radiance(scene, refl_ray, depth, randState)) * Re;
            ret += (f * radiance(scene, refr_ray, depth, randState)) * Tr;
        }
        return ret * hit.get_texture_color();
    }
    return Vector3f::ZERO;
}

struct coord {
    double x;
    double y;
    coord(double _x, double _y): x(_x), y(_y) {}
}sampling_bases[4] = {coord(0, 0), coord(0, 1), coord(1, 0), coord(1, 1)};

int SAMPLES_PER_SAMPLING;

void shift(SceneParser *scene, Ray &r, unsigned short *randState) {
    double f = scene->getCamera()->focal_length;
    double ap_size = scene->getCamera()->aperture_size;
    if (f < 0 || ap_size < 0)
        return;
    Vector3f p = r.pointAtParameter(f);
    Vector3f shift = Vector3f(erand48(randState) - 0.5, erand48(randState) - 0.5, erand48(randState) - 0.5) * ap_size;
    Vector3f shifted_origin = r.getOrigin() + shift;
    Vector3f shifted_dir = (p - shifted_origin).normalized();
    r = Ray(shifted_origin, shifted_dir);
}

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }
    if (argc != 4) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file> <SAMPLES_PER_PIXEL>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.
    SAMPLES_PER_SAMPLING = atoi(argv[3]);
    auto starting_time = chrono::high_resolution_clock::now();
    SceneParser scene = SceneParser(inputFile.data());
    Image image = Image(scene.getCamera()->getWidth(), scene.getCamera()->getHeight());
    for (int x = 0; x < image.Width(); x++) {
        // cout << "rendering row " << x << "...\n";
        #pragma omp parallel for schedule(dynamic)      // OpenMP 
        for (int y = 0; y < image.Height(); y++) {
            unsigned short randState[3] = {x * y, x * x, y * y * y};
            Vector3f monte_carlo_radiance_sum = Vector3f::ZERO;
            for (auto sampling_base: sampling_bases) {
                const double sx = sampling_base.x;
                const double sy = sampling_base.y;
                for (int sample_cnt = 0; sample_cnt < SAMPLES_PER_SAMPLING / 4; sample_cnt++) {
                    double x_smp_bias, y_smp_bias, x_bias, y_bias;
                    double rnd1 = 2 * erand48(randState);
                    double rnd2 = 2 * erand48(randState);
                    if (rnd1 < 1)
                        x_smp_bias = sqrt(rnd1) - 1;
                    else
                        x_smp_bias = 1 - sqrt(2 - rnd1);
                    if (rnd2 < 1)
                        y_smp_bias = sqrt(rnd2) - 1;
                    else
                        y_smp_bias = 1 - sqrt(2 - rnd2);
                    x_bias = (sx + 0.5 + x_smp_bias) / 2.0;
                    y_bias = (sy + 0.5 + y_smp_bias) / 2.0;
                    // if (x % 8 == 0 && y == 128) {
                    //     printf("samples: (%lf, %lf)\n", x + x_bias,  y + y_bias);
                    // } 
                    Ray cam_ray = scene.getCamera()->generateRay(Vector2f(x + x_bias, y + y_bias));
                    // printf("============\n");
                    // cam_ray.getOrigin().print();
                    shift(&scene, cam_ray, randState);
                    // cam_ray.getOrigin().print();
                    monte_carlo_radiance_sum += radiance(&scene, cam_ray, 0, randState);
                }
            }
            image.SetPixel(x, y, monte_carlo_radiance_sum / SAMPLES_PER_SAMPLING);
        }
        auto notification_time = chrono::high_resolution_clock::now();
        auto time_elapsed = chrono::duration_cast<chrono::duration<double>>(notification_time - starting_time);
        fprintf(stderr,"\rRendering (%d spp) %5.2f%%, est. (%.4lf)", SAMPLES_PER_SAMPLING, 100. * x / (scene.getCamera()->getWidth() - 1), time_elapsed / ((x == 0? 1: x)) * (scene.getCamera()->getWidth() - 1 - x)); 
    }
    auto notification_time = chrono::high_resolution_clock::now();
    auto time_elapsed = chrono::duration_cast<chrono::duration<double>>(notification_time - starting_time);
    printf("\ntotal time elapsed during render %.4lfs\n", time_elapsed);
    image.SaveBMP(outputFile.data());
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

