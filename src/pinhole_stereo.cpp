//
// Created by warren on 2021/2/22.
//

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include "util.h"
#include <stdio.h>

#include "pinhole_stereo.h"

using namespace cv;
using namespace std;

//Pinhole类成员函数
void Pinhole::setParameter(){
    vector<Point2f> corners; // 存放一张图片的角点坐标
    // 对每张图片，计算棋盘内格点像素坐标，并存入corners_seq
    for (auto imageName:img_paths) {
//        cout<<"img_name"<<imageName<<endl;
        Mat imageInput = imread(imageName);
        cvtColor(imageInput, imageInput, CV_RGB2GRAY);
        imageSize.width = imageInput.cols; // 获取图片的宽度
        imageSize.height = imageInput.rows; // 获取图片的高度
        // 查找标定板的角点
        bool found = findChessboardCorners(imageInput, patternSize,
                                           corners); // 最后一个参数int flags的缺省值为：CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
        // 亚像素精确化。在findChessboardCorners中自动调用了cornerSubPix，为了更加精细化，我们自己再调用一次。
        if (found) // 当所有的角点都被找到
        {
            TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40,
                                                 0.001); // 终止标准，迭代40次或者达到0.001的像素精度
            cornerSubPix(imageInput, corners, Size(11, 11), Size(-1, -1),
                         criteria);// 由于我们的图像只存较大，将搜索窗口调大一些，（11， 11）为真实窗口的一半，真实大小为（11*2+1， 11*2+1）--（23， 23）
            corners_seq.push_back(corners); // 存入角点序列
            // 绘制角点
            drawChessboardCorners(imageInput, patternSize, corners, true);
//            imshow("cornersframe", imageInput);
            n_boards++;
            //cout<<"img_name"<<imageName<<endl;
//
//            if (waitKey(0) == 27)
//            {
//                destroyWindow("cornersframe");
//            }
        } else
            cout << "fail to detect. img_name " << imageName << endl;
    }
    // 计算棋盘内格点世界坐标（都一样的，但为了满足calibrateCamera输入，所以弄成vector of vector；为了模块化，所以每个Pinhole都算一遍）
    for (int pic = 0; pic < n_boards; pic++) {
        vector<Point3f> realPointSet;
        for (int i = 0; i < patternSize.height; i++) {
            for (int j = 0; j < patternSize.width; j++) {
                Point3f realPoint;
                // 假设标定板位于世界坐标系Z=0的平面
                realPoint.x = j * chessboardSize.width;
                realPoint.y = i * chessboardSize.height;
                realPoint.z = 0;
                realPointSet.push_back(realPoint);
            }
        }
        objectPoints.push_back(realPointSet);
    }
    // 执行标定程序
    cout << "start to calib" << endl;
    vector<Mat> rvec; // 旋转向量
    vector<Mat> tvec; // 平移向量
    cout << "object point num" << endl << objectPoints.size() << endl;
    cout << "corners_seq num" << endl << corners_seq.size() << endl;
    calibrateCamera(objectPoints, corners_seq, imageSize, cameraMatrix, distCoeffs, rvec, tvec, 0);
    // 计算重投影点，与原图角点比较，得到误差
    double totalErr = 0.; // 误差总和
    vector<Point2f> projectImagePoints; // 重投影点
    for (int i = 0; i < n_boards; i++) {
        vector<Point3f> tempObjectPoints = objectPoints[i]; // 临时三维点
        // 计算重投影点
        projectPoints(tempObjectPoints, rvec[i], tvec[i], cameraMatrix, distCoeffs, projectImagePoints);
        // 计算新的投影点与旧的投影点之间的误差
        vector<Point2f> tempCornersPoints = corners_seq[i];// 临时存放旧投影点
        Mat tempCornersPointsMat = Mat(1, tempCornersPoints.size(), CV_32FC2); // 定义成两个通道的Mat是为了计算误差
        Mat projectImagePointsMat = Mat(1, projectImagePoints.size(), CV_32FC2);
        // 赋值
        for (int j = 0; j < tempCornersPoints.size(); j++) {
            projectImagePointsMat.at<Vec2f>(0, j) = Vec2f(projectImagePoints[j].x, projectImagePoints[j].y);
            tempCornersPointsMat.at<Vec2f>(0, j) = Vec2f(tempCornersPoints[j].x, tempCornersPoints[j].y);
        }
        // opencv里的norm函数其实把这里的两个通道分别分开来计算的(X1-X2)^2的值，然后统一求和，最后进行根号
        errors.push_back(norm(tempCornersPointsMat, projectImagePointsMat, NORM_L2) / (patternSize.width * patternSize.height));
        totalErr += errors[errors.size()-1];
        avgErr = totalErr / n_boards;
    }
    //判断标定是否成功
    if(avgErr < AVG_ERR_TOLERANCE){
        isCali = true;
    }
}

void Pinhole::printInfo(){
    // 保存标定结果
    cout << "相机内参数矩阵" << endl;
    cout << cameraMatrix << endl << endl;
    cout << "相机畸变系数" << endl;
    cout << distCoeffs << endl << endl;
    for (int i = 0; i < n_boards; i++) {
        cout << "第" << i + 1 << "张图像的平均误差为：" << errors[i] << endl;
    }
    cout << "全局平均误差为：" << avgErr << endl;
}

void Pinhole::writeYaml(){
    FileStorage storage(yaml_dir + yaml_name, FileStorage::WRITE);
    cerr << yaml_dir+yaml_name << endl;
    cerr << "1" << endl;
    storage << "Camera_fx" << cameraMatrix.at<double>(0,0);
    storage << "Camera_fy" << cameraMatrix.at<double>(1,1);
    storage << "Camera_cx" << cameraMatrix.at<double>(0,2);
    storage << "Camera_cy" << cameraMatrix.at<double>(1,2);
    storage << "Camera_k1" << distCoeffs.at<double>(0,0);
    storage << "Camera_k2" << distCoeffs.at<double>(0,1);
    storage << "Camera_p1" << distCoeffs.at<double>(0,2);
    storage << "Camera_p2" << distCoeffs.at<double>(0,3);
    storage << "Camera_k3" << distCoeffs.at<double>(0,4);

    storage << "Camera_width" << imageSize.width;
    storage << "Camera_height" << imageSize.height;
    storage.release();
    cerr << "2" << endl;
}

//Stereo类成员函数的子函数

bool stereoCalibrate(string stereoCalibrateResult, vector<vector<Point3f>> objectPoints,
                     vector<vector<Point2f>> imagePoints1, vector<vector<Point2f>> imagePoints2,
                     Mat &cameraMatrix1, Mat &distCoeffs1, Mat &cameraMatrix2, Mat &distCoeffs2, Size &imageSize,
                     Mat &R, Mat &T, Mat &E, Mat &F) {
    ofstream stereoStore(stereoCalibrateResult);



    stereoStore << "image size" << endl;
    stereoStore << imageSize << endl;
    stereoStore << "cameraMatrix_L" << endl;
    stereoStore << cameraMatrix1 << endl;
    stereoStore << "cameraMatrix_R" << endl;
    stereoStore << cameraMatrix2 << endl;
    stereoStore << "distCoeffs_L" << endl;
    stereoStore << distCoeffs1 << endl;
    stereoStore << "distCoeffs_R" << endl;
    stereoStore << distCoeffs2 << endl;
    stereoStore << "R" << endl;
    stereoStore << R << endl;
    stereoStore << "T" << endl;
    stereoStore << T << endl;
    stereoStore << "E" << endl;
    stereoStore << E << endl;
    stereoStore << "F" << endl;
    stereoStore << F << endl;
    stereoStore.close();
    return true;
}

/*
立体校正
参数：
	stereoRectifyParams	存放立体校正结果的txt
	cameraMatrix			相机内参数
	distCoeffs				相机畸变系数
	imageSize				图像尺寸
	R						左右相机相对的旋转矩阵
	T						左右相机相对的平移向量
	R1, R2					行对齐旋转校正
	P1, P2					左右投影矩阵
	Q						重投影矩阵
	map1, map2				重投影映射表
*/
Rect stereoRectification(string stereoRectifyParams, Mat &cameraMatrix1, Mat &distCoeffs1, Mat &cameraMatrix2,
                         Mat &distCoeffs2,
                         Size &imageSize, Mat &R, Mat &T, Mat &R1, Mat &R2, Mat &P1, Mat &P2, Mat &Q, Mat &mapl1,
                         Mat &mapl2, Mat &mapr1, Mat &mapr2) {
    Rect validRoi[2];
    ofstream stereoStore(stereoRectifyParams);
    //cout << "before:"<<endl<<" cameraMatrix1" << cameraMatrix1 << endl \
    //<< "distCoeffs1" << distCoeffs1 << endl << "cameraMatrix2" << cameraMatrix2 << endl \
    //<< "distCoeffs2" << distCoeffs2 << endl << "imageSize" << imageSize \
    //<< endl << "R" << R << endl << "T" << T << endl;
    stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize,
                  R, T, R1, R2, P1, P2, Q, 0, -1, imageSize, &validRoi[0], &validRoi[1]);

    cerr << "立体校正后图像尺寸" << imageSize <<endl;

    // 计算左右图像的重投影映射表
    stereoStore << "R1：" << endl;
    stereoStore << R1 << endl;
    stereoStore << "R2：" << endl;
    stereoStore << R2 << endl;
    stereoStore << "P1：" << endl;
    stereoStore << P1 << endl;
    stereoStore << "P2：" << endl;
    stereoStore << P2 << endl;
    stereoStore << "Q：" << endl;
    stereoStore << Q << endl;
    stereoStore.close();
    //cout << "R1:" << endl;
    //cout << R1 << endl;
    //cout << "R2:" << endl;
    //cout << R2 << endl;
    //cout << "P1:" << endl;
    //cout << P1 << endl;
    //cout << "P2:" << endl;
    //cout << P2 << endl;
    //cout << "Q:" << endl;
    //cout << Q << endl;
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_32FC1, mapl1, mapl2);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_32FC1, mapr1, mapr2);

    cerr << "initUndistortRectifyMap后图像尺寸" << imageSize << endl;

    return validRoi[0], validRoi[1];
}
//Stereo类成员函数
void Stereo::setParameter() {
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
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 1e-6); // 终止条件
    stereoCalibrate(pinhole_l.objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1,
                    cameraMatrix2, distCoeffs2, imageSize, R, T, E, F, CALIB_FIX_INTRINSIC,
                    criteria); // 注意参数顺序，可以到保存的文件中查看，避免返回时出错
}