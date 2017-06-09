#include "mlibutil.h"

#include <fstream>
#include <string>
#include <sstream>

void int2str(const int &int_temp, std::string &string_temp)
{
	std::stringstream stream;
	stream << int_temp;
	string_temp = stream.str();
}

void print_ith_frame_2_obj(ml::SensorData* input, int ith, 
							unsigned short* depth_data, ml::vec3uc* color_data, 
							ml::mat4f& pose, ml::mat4f& intrinsic){
	std::ofstream ofs;
	std::string filename;
	int2str(ith, filename);
	filename.append("th.obj");
	ofs.open(filename);

	for (int y = 0; y < input->m_colorHeight; y++){
		for (int x = 0; x < input->m_colorWidth; x++){
			ml::vec3uc rgb = (ml::vec3uc) *(color_data + x + y*input->m_colorWidth);
			float	     Z = (float)*(depth_data + x + y*input->m_colorWidth);
			float        X = Z * (x - intrinsic(0, 2)) / intrinsic(0, 0);
			float        Y = Z * (y - intrinsic(1, 2)) / intrinsic(1, 1);
			ml::vec4f coord_C(X, Y, Z, 1);
			ml::vec4f coord_W = pose * coord_C;
			ofs << "v " << coord_W[0] << " " << coord_W[1] << " " << coord_W[2] << " "
				<< rgb[0] / 256.0 << " " << rgb[1] / 256.0 << " " << rgb[2] / 256.0 << "\n";
		}
	}

	ofs.close();
}

// code src
// http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html#feature-homography
// 2017/6/8
void get_homography_martix(ml::SensorData* input, int ith,
						unsigned short* depth_data, ml::vec3uc* color_data,
						ml::mat4f& homography_pose, ml::mat4f& intrinsic){
	if (ith <= 0)
		return;
	ml::vec3uc* color_data_prev     = input->decompressColorAlloc(ith - 1);
	unsigned short* depth_data_prev = input->decompressDepthAlloc(ith - 1);
	cv::Mat colorimg(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);
	cv::Mat colorimg_prev(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data_prev);

	//-- Step 1: Detect the keypoints using SURF Detector

	cv::SiftFeatureDetector detector;
	//int minHessian = 200;
	//cv::SurfFeatureDetector detector(minHessian);

	std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

	detector.detect(colorimg, keypoints_object);
	detector.detect(colorimg_prev, keypoints_scene);

	//-- Step 2: Calculate descriptors (feature vectors)
	cv::SurfDescriptorExtractor extractor;

	cv::Mat descriptors_object, descriptors_scene;

	extractor.compute(colorimg, keypoints_object, descriptors_object);
	extractor.compute(colorimg_prev, keypoints_scene, descriptors_scene);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match(descriptors_object, descriptors_scene, matches);

	double max_dist = 0; double min_dist = 1000;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< cv::DMatch > good_matches;

	for (int i = 0; i < descriptors_object.rows; i++)
	{
		if (matches[i].distance < 2 * min_dist)
		{
			good_matches.push_back(matches[i]);
		}
	}

	cv::Mat img_matches;
	drawMatches(colorimg, keypoints_object, colorimg_prev, keypoints_scene,
		good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow("Good Matches & Object detection", img_matches);
	cv::waitKey(0);

	//-- Localize the object
	std::vector<cv::Point3f> obj;
	std::vector<cv::Point3f> scene;
	//std::vector<cv::Point3f> inliers;
	std::vector<uchar> inliers;

	//float	       Z = (float)*(depth_data + x + y*input->m_colorWidth);
	//float        X = Z * (x - intrinsic(0, 2)) / intrinsic(0, 0);
	//float        Y = Z * (y - intrinsic(1, 2)) / intrinsic(1, 1);

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		cv::Point3f obj_pt3, scene_pt3;
		cv::Point2f obj_pt2, scene_pt2;
		obj_pt2   = keypoints_object[good_matches[i].queryIdx].pt;
		scene_pt2 = keypoints_scene[good_matches[i].trainIdx].pt;

		std::cout << obj_pt2 << " ## " << scene_pt2 << "\n";
		
		int xmin = (int)obj_pt2.x;
		int ymin = (int)obj_pt2.y;
		int xmax = (int)(obj_pt2.x + 1);
		int ymax = (int)(obj_pt2.y + 1);
		float d1 = (float)*(depth_data + xmin + ymin*input->m_colorWidth);
		float d2 = (float)*(depth_data + xmax + ymin*input->m_colorWidth);
		float d3 = (float)*(depth_data + xmin + ymax*input->m_colorWidth);
		float d4 = (float)*(depth_data + xmax + ymax*input->m_colorWidth);
		obj_pt3.z = (d1*(-obj_pt2.x - obj_pt2.y + xmax + ymax) +
			d2*(obj_pt2.x - obj_pt2.y - xmin + ymax) +
			d3*(-obj_pt2.x + obj_pt2.y + xmax - ymin) +
			d4*(obj_pt2.x + obj_pt2.y - xmin - ymin)) / 4;
		obj_pt3.x = obj_pt3.z * (obj_pt2.x - intrinsic(0, 2)) / intrinsic(0, 0);
		obj_pt3.y = obj_pt3.z * (obj_pt2.y - intrinsic(1, 2)) / intrinsic(1, 1);

		//scene_pt3.z = (float)*(depth_data_prev + ((int)(scene_pt2.x+0.5) + (int)(scene_pt2.y+0.5)*input->m_colorWidth));
		xmin = (int)scene_pt2.x;
		ymin = (int)scene_pt2.y;
		xmax = (int)(scene_pt2.x + 1);
		ymax = (int)(scene_pt2.y + 1);
		d1 = (float)*(depth_data_prev + xmin + ymin*input->m_colorWidth); 
		d2 = (float)*(depth_data_prev + xmax + ymin*input->m_colorWidth);
		d3 = (float)*(depth_data_prev + xmin + ymax*input->m_colorWidth);
		d4 = (float)*(depth_data_prev + xmax + ymax*input->m_colorWidth);
		scene_pt3.z = (d1*(-obj_pt2.x - obj_pt2.y + xmax + ymax) +
			d2*(obj_pt2.x - obj_pt2.y - xmin + ymax) +
			d3*(-obj_pt2.x + obj_pt2.y + xmax - ymin) +
			d4*(obj_pt2.x + obj_pt2.y - xmin - ymin)) / 4;
		scene_pt3.x = scene_pt3.z * (scene_pt2.x - intrinsic(0, 2)) / intrinsic(0, 0);
		scene_pt3.y = scene_pt3.z * (scene_pt2.y - intrinsic(1, 2)) / intrinsic(1, 1);

		std::cout << obj_pt3 << " # " << scene_pt3 << "\n";

		obj.push_back(obj_pt3);
		scene.push_back(scene_pt3);
	}

	//cv::Mat H = findHomography(obj, scene, CV_RANSAC);
	cv::Mat H(3, 4, CV_64F);
	cv::estimateAffine3D(obj, scene, H, inliers, 3, 0.99);

	for (int i = 0; i < inliers.size(); i++){
		if (inliers[i] == 1)
			std::cout << obj[i] << " * " << scene[i] << "\n";
	}

	//cv::Mat img_matches;
	//std::vector< cv::DMatch > new_good_matches;
	//for (int i = 0; i < inliers.size(); i++){
	//	if (inliers[i] == 1)
	//		new_good_matches.push_back(good_matches[i]);
	//}
	//drawMatches(colorimg, keypoints_object, colorimg_prev, keypoints_scene,
	//new_good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
	//std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//imshow("Good Matches & Object detection", img_matches);
	//cv::waitKey(0);

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			homography_pose(i, j) = H.at<double>(i, j);
		}
	}

	std::cout << homography_pose;
}

void test_sens()
{
	// Load RGB-D Data
	ml::SensorData* input = new ml::SensorData("../../data/test.sens");
	// Basic Information
	std::cout << "Color Frame Size (w, h) = (" << input->m_colorWidth << ", " << input->m_colorHeight << ")\n";
	std::cout << "Depth Frame Size (w, h) = (" << input->m_depthWidth << ", " << input->m_depthHeight << ")\n";
	std::cout << "Color Intrinsic Matrix:\n";
	std::cout << input->m_calibrationColor.m_intrinsic << "\n";
	std::cout << "Depth Intrinsic Matrix:\n";
	std::cout << input->m_calibrationDepth.m_intrinsic << "\n";
	std::cout << "Frame number:\n";
	std::cout << input->m_frames.size() << "\n";
	// Frame Information¡®

	for (int i = 2400; i < input->m_frames.size(); ++i) {
		std::cout << i << "\n";
		// decompress depth and color data
		// depth (mm)
		unsigned short* depth_data = input->decompressDepthAlloc(i);
		// color (rgb [0,255],[0,255],[0,255])
		ml::vec3uc* color_data = input->decompressColorAlloc(i);
		// extrinsic (camera2world)
		ml::mat4f pose = input->m_frames[i].getCameraToWorld();

		std::cout << pose;
	
		ml::mat4f intrinsic = input->m_calibrationColor.m_intrinsic;
		ml::mat4f homography_pose;
		//get_homography_martix(input, i, depth_data, color_data, homography_pose);
		//print_ith_frame_2_obj(input, i, depth_data, color_data, pose, intrinsic);

		if (i == 2400){
			ml::vec3<float> v0 = { 1, 0, 0 };
			ml::vec3<float> v1 = { 0, 1, 0 };
			ml::vec3<float> v2 = { 0, 0, 1 };
			ml::mat4f m4(v0, v1, v2);
			print_ith_frame_2_obj(input, i, depth_data, color_data, m4, intrinsic);
			continue;
		}
		else {
			//ml::vec3<float> v0 = { 1, 0, 0 };
			//ml::vec3<float> v1 = { 0, 1, 0 };
			//ml::vec3<float> v2 = { 0, 0, 1 };
			//ml::mat4f m4(v0, v1, v2);
			//print_ith_frame_2_obj(input, i, depth_data, color_data, m4, intrinsic);
			get_homography_martix(input, i, depth_data, color_data, homography_pose, intrinsic);
			print_ith_frame_2_obj(input, i, depth_data, color_data, homography_pose, intrinsic);
			break;
		}

		// Visualize color and depth
		cv::Mat colorimg(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);
		cv::Mat depthimg(input->m_depthHeight, input->m_depthWidth, CV_16UC1, depth_data);
		cv::Mat colorVis, depthVis;
		cv::cvtColor(colorimg, colorVis, CV_RGB2BGR);
		depthimg.convertTo(depthVis, CV_8U, 0.064f);
		cv::imshow("color", colorVis);
		//cv::waitKey(100000);
		//break;
		//cv::imshow("depth", depthVis);
		cv::waitKey((i == 0) ? 0 : 1);
		// remember to free the data
		::free(color_data);
		::free(depth_data);
	}

	std::cout << "pause";
	int pause;
	std::cin >> pause;
}