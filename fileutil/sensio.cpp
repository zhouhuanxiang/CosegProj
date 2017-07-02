#include "sensio.h"
#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif

#include "mlibutil.h"
#include <fstream>

void Image2Sens(const char* folder, const char* name)
{
	char buffer[200];
	sprintf(buffer, "%s\\poses.txt", folder);
	std::ifstream is(buffer);
	int nImages;
	float fx, fy, cx, cy;
	is >> nImages >> fx >> fy >> cx >> cy;
	ml::SensorData* input = new ml::SensorData();
	ml::mat4f intrinsic = ml::mat4f::identity();
	intrinsic(0, 0) = fx;
	intrinsic(1, 1) = fy;
	intrinsic(0, 2) = cx;
	intrinsic(1, 2) = cy;
	std::vector<ml::mat4f> poses;
	poses.resize(nImages);
	for (int i = 0; i < nImages; ++i) {
		for (int j = 0; j < 4; ++j) {
			for (int k = 0; k < 4; ++k) {
				is >> poses[i](j, k);
			}
		}
		poses[i] = poses[i].getInverse();
	}
	for (int i = 0; i < nImages; ++i) {
		printf("read %d of %d\n", i, nImages);
		sprintf(buffer, "%s\\color%d.jpg", folder, i);
		auto m = (cv::imread(buffer, CV_LOAD_IMAGE_UNCHANGED));
		sprintf(buffer, "%s\\depth%d.png", folder, i);
		auto n(cv::imread(buffer, CV_LOAD_IMAGE_UNCHANGED));
		if (i == 0) {
			input->initDefault(m.cols, m.rows, n.cols, n.rows, intrinsic, intrinsic);
		}
		input->addFrame((ml::vec3uc*)m.data, (unsigned short*)n.data, poses[i], i, i);
	}
	for (int i = 0; i < nImages; ++i) {
	}
	input->saveToFile(name);
}

void Sens2Image(const char* name, const char* folder)
{

}