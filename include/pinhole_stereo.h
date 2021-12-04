//
// Created by warren on 2021/2/22.
//

#ifndef STEREO_CALIB_PINHOLE_STEREO_H
#define STEREO_CALIB_PINHOLE_STEREO_H

#endif //STEREO_CALIB_PINHOLE_STEREO_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include "util.h"
#include <stdio.h>

using namespace cv;
using namespace std;

#define W 11
#define H 8
#define S 15
#define AVG_ERR_TOLERANCE 0.203294*1.5



class Pinhole
{
public:
    //Constructor
    Pinhole();
    Pinhole(const string img_dir, const string yaml_dir, const string yaml_name):
            yaml_dir(yaml_dir), yaml_name(yaml_name){
        getStereoSortedImages(img_dir, img_paths);
    }

    //Variables
    std::string yaml_dir;
    std::string yaml_name;
    vector<std::string> img_paths;
    int n_boards = 0;
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;
    Mat cameraMatrix;
    Mat distCoeffs;
    Size imageSize;
    Size patternSize = Size(W, H);
    Size chessboardSize = Size(S, S);
    vector<double> errors;
    double avgErr = 0;
    bool isCali = false;

    //Functions
    void setParameter();

    void printInfo();

    void writeYaml();
};


/*
双目标定:计算两摄像机相对旋转矩阵 R,平移向量 T, 本征矩阵E, 基础矩阵F
参数：
	stereoCalibrateResult	存放立体标定结果的txt
	objectPoints			三维点
	imagePoints				二维图像上的点
	cameraMatrix			相机内参数
	distCoeffs				相机畸变系数
	imageSize				图像尺寸
	R		左右相机相对的旋转矩阵
	T		左右相机相对的平移向量
	E		本征矩阵
	F		基础矩阵
*/
class Stereo
{
public:
    //Constructor
    Stereo();
    Stereo(Pinhole &Pinhole_l, Pinhole &Pinhole_r, const std::string &yaml_name):
    pinhole_l(Pinhole_l), pinhole_r(Pinhole_r), yaml_dir(Pinhole_l.yaml_dir), yaml_name(yaml_name){}

    //Variables
    std::string yaml_dir;
    std::string yaml_name;
    Pinhole &pinhole_l;
    Pinhole &pinhole_r;
    Mat R, T, E, F, R1, R2, P1, P2, Q;
    Rect validRoi[2];
    Mat map_l1, map_l2, map_r1, map_r2;
    bool isCali = 0;

    void setParameter();

    void printInfo();

    void writeYaml();



    //Functions
};