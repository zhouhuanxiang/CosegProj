#ifndef SENS_IO_H_
#define SENS_IO_H_

#include <string>

void Image2Sens(const char* folder, const char* name, bool noise);

void Sens2Image(const char* name, const char* folder);

void Sens2SensNoise(const std::string& outputFile, const std::string& inputFile, float noise = 0.005);

#endif