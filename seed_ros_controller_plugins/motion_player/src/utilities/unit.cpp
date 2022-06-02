#include "unit.hpp"

double m2pix(double meter){
    return (meter / meterPerPixel_def);
}

double pix2m(double pixel){
    return pixel * meterPerPixel_def;
}

double mm2m(double mmeter){
    return mmeter / 1000.0;
}

double m2mm(double meter){
    return meter * 1000.0;
}
