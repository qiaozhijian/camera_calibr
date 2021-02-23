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
    /*
    单目标定
    参数：
    	imageList		存放标定图片名称的txt
	    singleCalibrateResult	存放标定结果的txt
	    objectPoints	世界坐标系中点的坐标
    	corners_seq		存放图像中的角点,用于立体标定
    	cameraMatrix	相机的内参数矩阵
    	distCoeffs		相机的畸变系数
	    imageSize		输入图像的尺寸（像素）
	    patternSize		标定板每行的角点个数, 标定板每列的角点个数 (9, 6)
    	chessboardSize	棋盘上每个方格的边长（mm）
    注意：亚像素精确化时，允许输入单通道，8位或者浮点型图像。由于输入图像的类型不同，下面用作标定函数参数的内参数矩阵和畸变系数矩阵在初始化时也要数据注意类型。
    */
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
            imagePoints.push_back(corners); // 存入角点序列
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
    cout << "corners_seq num" << endl << imagePoints.size() << endl;
    calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvec, tvec, 0);
    // 计算重投影点，与原图角点比较，得到误差
    double totalErr = 0.; // 误差总和
    vector<Point2f> projectImagePoints; // 重投影点
    for (int i = 0; i < n_boards; i++) {
        vector<Point3f> tempObjectPoints = objectPoints[i]; // 临时三维点
        // 计算重投影点
        projectPoints(tempObjectPoints, rvec[i], tvec[i], cameraMatrix, distCoeffs, projectImagePoints);
        // 计算新的投影点与旧的投影点之间的误差
        vector<Point2f> tempCornersPoints = imagePoints[i];// 临时存放旧投影点
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
    // 输出标定结果
    cout << "单目标定结果:" << endl;
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
    storage << "#--------------------------------------------------------------------------------------------";
    storage << "# Camera Parameters. Adjust them!";
    storage << "#--------------------------------------------------------------------------------------------\n";

    storage << "# Camera calibration and distortion parameters (OpenCV)";
    storage << "Camera_fx" << cameraMatrix.at<double>(0,0);
    storage << "Camera_fy" << cameraMatrix.at<double>(1,1);
    storage << "Camera_cx" << cameraMatrix.at<double>(0,2);
    storage << "Camera_cy" << cameraMatrix.at<double>(1,2) << "\n";

    storage << "Camera_k1" << distCoeffs.at<double>(0,0);
    storage << "Camera_k2" << distCoeffs.at<double>(0,1);
    storage << "Camera_p1" << distCoeffs.at<double>(0,2);
    storage << "Camera_p2" << distCoeffs.at<double>(0,3);
    storage << "Camera_k3" << distCoeffs.at<double>(0,4);

    storage << "Camera_width" << imageSize.width;
    storage << "Camera_height" << imageSize.height << "\n";

    //以下参数还有待根据具体情况决定怎么算
    storage << "###-----STARTING FROM THIS LINE, PARAMETERS ARE NOT CALCULATED YET! TO BE MODIFIED!------";
    storage << "# Camera frames per second";
    storage << "Camera_fps" << 20.0 << "\n";

    storage << "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)";
    storage << "Camera_RGB" << 1 << "\n";

    storage << "#--------------------------------------------------------------------------------------------";
    storage << "# ORB Parameters";
    storage << "#--------------------------------------------------------------------------------------------" << "\n";

    storage << "# ORB Extractor: Number of features per image";
    storage << " ORBextractor_nFeatures" << 1000 << "\n";

    storage << "# ORB Extractor: Scale factor between levels in the scale pyramid";
    storage << "ORBextractor_scaleFactor" << 1.2 << "\n";

    storage << "# ORB Extractor: Number of levels in the scale pyramid";
    storage << "ORBextractor_nLevels" << 8 << "\n";

    storage << "# ORB Extractor: Fast threshold";
    storage << "# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.";
    storage << "# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST";
    storage << "# You can lower these values if your images have low contrast";
    storage << "ORBextractor_iniThFAST" << 20 << "";
    storage << "ORBextractor_minThFAST" << 7 << "\n";

    storage << "#--------------------------------------------------------------------------------------------";
    storage << "# Viewer Parameters";
    storage << "#---------------------------------------------------------------------------------------------\n";

    storage << "Viewer_KeyFrameSize" << 0.05;
    storage << "Viewer_KeyFrameLineWidth" << 1;
    storage << "Viewer_GraphLineWidth" << 0.9;
    storage << "Viewer_PointSize" << 2;
    storage << "Viewer_CameraSize" << 0.08;
    storage << "Viewer_CameraLineWidth" << 3;
    storage << "Viewer_ViewpointX" << 0;
    storage << "Viewer_ViewpointY" << -0.7;
    storage << "Viewer_ViewpointZ" << -1.8;
    storage << "Viewer_ViewpointF" << 500;

    storage.release();
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
    stereoCalibrate(pinhole_l.objectPoints, pinhole_l.imagePoints, pinhole_r.imagePoints, 
                    pinhole_l.cameraMatrix, pinhole_l.distCoeffs,pinhole_r.cameraMatrix, pinhole_r.distCoeffs, 
                    pinhole_l.imageSize, R, T, E, F, CALIB_FIX_INTRINSIC,
                    criteria); // 注意参数顺序，可以到保存的文件中查看，避免返回时出错

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
    
    //cout << "立体校正前:"<<endl<<"pinhole_l.cameraMatrix" << pinhole_l.cameraMatrix << endl \
    //<< "pinhole_l.distCoeffs" << pinhole_l.distCoeffs << endl << "pinhole_r.cameraMatrix" << pinhole_r.cameraMatrix << endl \
    //<< "pinhole_r.distCoeffs" << pinhole_r.distCoeffs << endl << "pinhole_l.imageSize" << pinhole_l.imageSize \
    //<< endl << "R" << R << endl << "T" << T << endl;
    stereoRectify(pinhole_l.cameraMatrix, pinhole_l.distCoeffs, 
                  pinhole_r.cameraMatrix, pinhole_r.distCoeffs, 
                  pinhole_l.imageSize,
                  R, T, R1, R2, P1, P2, Q, 
                  0, -1, pinhole_l.imageSize, &validRoi[0], &validRoi[1]);
    
    pinhole_r.imageSize = pinhole_l.imageSize;

    initUndistortRectifyMap(pinhole_l.cameraMatrix, pinhole_l.distCoeffs, 
                            R1, P1, pinhole_l.imageSize, CV_32FC1, map_l1, map_l2);
    initUndistortRectifyMap(pinhole_r.cameraMatrix, pinhole_r.distCoeffs, 
                            R2, P2, pinhole_r.imageSize, CV_32FC1, map_r1, map_r2);

}

void Stereo::printInfo() {
    //双目标定结果
    cout << "双目标定结果：" << endl;
    cout << "image size" << endl;
    cout << pinhole_l.imageSize << endl;
    cout << "cameraMatrix_L" << endl;
    cout << pinhole_l.cameraMatrix << endl;
    cout << "cameraMatrix_R" << endl;
    cout << pinhole_r.cameraMatrix << endl;
    cout << "distCoeffs_L" << endl;
    cout << pinhole_l.distCoeffs << endl;
    cout << "distCoeffs_R" << endl;
    cout << pinhole_r.distCoeffs << endl;
    cout << "R" << endl;
    cout << R << endl;
    cout << "T" << endl;
    cout << T << endl;
    cout << "E" << endl;
    cout << E << endl;
    cout << "F" << endl;
    cout << F << endl;
    
    //立体校正结果
    cout << "立体校正结果：" << endl;
    cout << "R1：" << endl;
    cout << R1 << endl;
    cout << "R2：" << endl;
    cout << R2 << endl;
    cout << "P1：" << endl;
    cout << P1 << endl;
    cout << "P2：" << endl;
    cout << P2 << endl;
    cout << "Q：" << endl;
    cout << Q << endl;
}

void Stereo::writeYaml() {
    FileStorage storage(yaml_dir + yaml_name, FileStorage::WRITE);
    storage << "#--------------------------------------------------------------------------------------------";
    storage << "# Camera Parameters. Adjust them!";
    storage << "#--------------------------------------------------------------------------------------------\n";

    storage << "# Camera calibration and distortion parameters (OpenCV)";
    storage << "Camera_fx" << pinhole_l.cameraMatrix.at<double>(0,0);
    storage << "Camera_fy" << pinhole_l.cameraMatrix.at<double>(1,1);
    storage << "Camera_cx" << pinhole_l.cameraMatrix.at<double>(0,2);
    storage << "Camera_cy" << pinhole_l.cameraMatrix.at<double>(1,2) << "\n";

    storage << "Camera_k1" << pinhole_l.distCoeffs.at<double>(0,0);
    storage << "Camera_k2" << pinhole_l.distCoeffs.at<double>(0,1);
    storage << "Camera_p1" << pinhole_l.distCoeffs.at<double>(0,2);
    storage << "Camera_p2" << pinhole_l.distCoeffs.at<double>(0,3);
    storage << "Camera_k3" << pinhole_l.distCoeffs.at<double>(0,4) << "\n";

    storage << "Camera_width" << pinhole_l.imageSize.width;
    storage << "Camera_height" << pinhole_l.imageSize.height << "\n";

    //以下参数还有待根据具体情况决定怎么算
    storage << "###-----STARTING FROM THIS LINE, EXCEPT STEREO RECTIFICATION, PARAMETERS ARE NOT CALCULATED YET! TO BE MODIFIED!------";
    storage << "# Camera frames per second";
    storage << "Camera_fps" << 47.90639384423901 << "\n";

    storage << "# stereo baseline times fx";
    storage << "Camera_bf" << 20.0 << "\n";

    storage << "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)";
    storage << "Camera_RGB" << 1 << "\n";

    storage << "# Close/Far threshold. Baseline times.";
    storage << "ThDepth" << 35 << "\n";

    storage << "#--------------------------------------------------------------------------------------------";
    storage << "# Stereo Rectification. Only if you need to pre-rectify the images.";
    storage << "# Camera.fx, .fy, etc must be the same as in LEFT.P";
    storage << "#--------------------------------------------------------------------------------------------" << "\n";
    storage << "Left_height" << pinhole_l.imageSize.height;
    storage << "Left_width" << pinhole_l.imageSize.width;
    storage << "Left_D" << pinhole_l.distCoeffs;
    storage << "Left_K" << pinhole_l.cameraMatrix;
    storage << "Left_R" << R1;
    storage << "Left_P" << P1 << "\n";

    storage << "Right_height" << pinhole_r.imageSize.height;
    storage << "Right_width" << pinhole_r.imageSize.width;
    storage << "Right_D" << pinhole_r.distCoeffs;
    storage << "Right_K" << pinhole_r.cameraMatrix;
    storage << "Right_R" << R2;
    storage << "Right_P" << P2 << "\n";
    
    storage << "#--------------------------------------------------------------------------------------------";
    storage << "# ORB Parameters";
    storage << "#--------------------------------------------------------------------------------------------" << "\n";

    storage << "# ORB Extractor: Number of features per image";
    storage << " ORBextractor_nFeatures" << 1200 << "\n";

    storage << "# ORB Extractor: Scale factor between levels in the scale pyramid";
    storage << "ORBextractor_scaleFactor" << 1.2 << "\n";

    storage << "# ORB Extractor: Number of levels in the scale pyramid";
    storage << "ORBextractor_nLevels" << 8 << "\n";

    storage << "# ORB Extractor: Fast threshold";
    storage << "# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.";
    storage << "# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST";
    storage << "# You can lower these values if your images have low contrast";
    storage << "ORBextractor_iniThFAST" << 20 << "";
    storage << "ORBextractor_minThFAST" << 7 << "\n";

    storage << "#--------------------------------------------------------------------------------------------";
    storage << "# Viewer Parameters";
    storage << "#---------------------------------------------------------------------------------------------\n";

    storage << "Viewer_KeyFrameSize" << 0.05;
    storage << "Viewer_KeyFrameLineWidth" << 1;
    storage << "Viewer_GraphLineWidth" << 0.9;
    storage << "Viewer_PointSize" << 2;
    storage << "Viewer_CameraSize" << 0.08;
    storage << "Viewer_CameraLineWidth" << 3;
    storage << "Viewer_ViewpointX" << 0;
    storage << "Viewer_ViewpointY" << -0.7;
    storage << "Viewer_ViewpointZ" << -1.8;
    storage << "Viewer_ViewpointF" << 500;

    storage.release();
}