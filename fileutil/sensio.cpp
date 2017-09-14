#include "sensio.h"
#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif

#include "mlibutil.h"

#include <fstream>
#include <random>

#define STANDARD_DEVIATION 0.015

void AddGaussNoise2Pose(ml::mat4f& pose){
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> distribution(0.0, STANDARD_DEVIATION);
  float params[6];
  for (int i = 0; i < 6; i++){
	params[i] = distribution(rd);
  }
  ml::mat4f increment = {
	1.0,			-1 * params[2],	params[1],		params[3],
	params[2],		1.0,			-1 * params[0],	params[4],
	-1 * params[1], params[0],		1.0,			params[5],
	0.0,			0.0,			0.0,			1.0
  };
  pose = increment * pose;
}

void Image2Sens(const char* folder, const char* name, bool noise)
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
		if (noise)
		  AddGaussNoise2Pose(poses[i]);
	}
	for (int i = 0; i < nImages; ++i) {
		//printf("read %d of %d\n", i, nImages);
		sprintf(buffer, "%s\\color%d.jpg", folder, i);
		auto m = (cv::imread(buffer, CV_LOAD_IMAGE_UNCHANGED));
		sprintf(buffer, "%s\\depth%d.png", folder, i);
		auto n(cv::imread(buffer, CV_LOAD_IMAGE_UNCHANGED));
		cv::Mat nn;
		n.convertTo(nn, CV_16UC1);
		if (i == 0) {
		  input->initDefault(m.cols, m.rows, n.cols, n.rows, intrinsic, intrinsic);
		}
		input->addFrame((ml::vec3uc*)m.data, (unsigned short*)nn.data, poses[i], i, i);
	}
	for (int i = 0; i < nImages; ++i) {
	}
	input->saveToFile(name);
}

void Sens2SensNoise(const std::string& outputFile, const std::string& inputFile, float noise)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<float> distribution(0.0, STANDARD_DEVIATION);

	ml::SensorData* input = new ml::SensorData(inputFile.c_str());
	const unsigned int imageWidth = input->m_depthWidth;
	const unsigned int imageHeight = input->m_depthHeight;

	ml::SensorData* output = new ml::SensorData();

	for (int iter = 0; iter < input->m_frames.size(); iter += 1) {
		printf("read %d of %d\n", iter, input->m_frames.size());
		unsigned short* depth_data = input->decompressDepthAlloc(iter);
		ml::vec3uc* color_data = input->decompressColorAlloc(iter);
		for (int i = 0; i < input->m_depthHeight; ++i) {
			for (int j = 0; j < input->m_depthWidth; ++j) {
				depth_data[i * input->m_depthWidth + j] *= distribution(rd) + 1;
			}
		}
		if (iter == 0)
			output->initDefault(imageWidth, imageHeight, imageWidth, imageHeight, input->m_calibrationColor.m_intrinsic, input->m_calibrationColor.m_intrinsic);
		output->addFrame((ml::vec3uc*)color_data, (unsigned short*)depth_data, input->m_frames[iter].getCameraToWorld(), iter, iter);
		::free(depth_data);
		::free(color_data);
	}
	output->saveToFile(outputFile);
}

void Sens2Image(const char* name, const char* folder)
{

}