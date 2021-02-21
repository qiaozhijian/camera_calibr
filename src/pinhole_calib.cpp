//
// function：进行单目标定,将参数存入指定文件夹下的指定yaml文件中
// usage：./pinhole_calib -w=<board_width default=11> -h=<board_height default=8> -s=<square_size default=15> -d=<dir default=/home/shenyl/Documents/sweeper/data/> -show=<if_show default=False>
//
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

static int print_help() {
    cout <<
         " Given a list of chessboard images, the number of corners (nx, ny)\n"
         " on the chessboards, the size of the square and the root dir of the calib image and the flag of if show the rectified results \n"
         " Calibrate and rectified the stereo camera, and save the result to txt \n" << endl;
    cout
            << "Usage:\n // usage：./stereo_calib -w=<board_width default=11> -h=<board_height default=8> -s=<square_size default=15> -d=<dir default=/home/shenyl/Documents/sweeper/data/> -show=<if_show default=False>\n"
            << endl;
    return 0;
}

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
class Pinhole
{
public:
    //Constructor
    Pinhole(const string img_dir, const string yaml_dir, const string yaml_name):
    img_dir(img_dir), yaml_dir(yaml_dir), yaml_name(yaml_name){
        getStereoSortedImages(img_dir, img_paths);
    }
    std::string img_dir;
    std::string yaml_dir;
    std::string yaml_name;
    vector<std::string> img_paths;
    int n_boards = 0;
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> corners_seq;
    Mat cameraMatrix;
    Mat distCoeffs;
    Size imageSize;
    Size patternSize = Size(W, H);
    Size chessboardSize = Size(S, S);
    vector<double> errors;
    double avgErr;
    bool isCali = false;
    //functions
    void setParameter(){
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

    void printInfo(){
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

    void writeYaml(){
        FileStorage storage(yaml_dir + yaml_name, FileStorage::WRITE);
        storage <<
                "Camera_fx", cameraMatrix.at<double>(0,0),
                "Camera_fy", cameraMatrix.at<double>(1,1),
                "Camera_cx", cameraMatrix.at<double>(0,2),
                "Camera_cy", cameraMatrix.at<double>(1,2),
                "Camera_k1", distCoeffs.at<double>(0,0),
                "Camera_k2", distCoeffs.at<double>(0,1),
                "Camera_p1", distCoeffs.at<double>(0,2),
                "Camera_p2", distCoeffs.at<double>(0,3),
                "Camera_k3", distCoeffs.at<double>(0,4),

                "Camera_width", imageSize.width,
                "Camera_height", imageSize.height;
        storage.release();
    }
};


int main(int argc, char *argv[]) {
    string root_dir = "/home/warren/Documents/CLionProj/Camra_Calibration/stereo_calibr/cali/";
    Pinhole pinhole(root_dir+"left/", root_dir+"result/", "PinholeCalibr.yaml");
    pinhole.setParameter();
    if(pinhole.isCali){
        pinhole.printInfo();
        pinhole.writeYaml();
    }
}