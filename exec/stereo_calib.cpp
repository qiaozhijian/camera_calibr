//
// function：进行双目内外的标定和双目矫正,将内外参数和矫正参数存入标定文件夹下的calib_result.txt文件中
// usage：./stereo_calib
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

int main(int argc, char *argv[]) {
    string root_dir = "/home/warren/Documents/CLionProj/Camra_Calibration/stereo_calibr/cali/";
    Pinhole pinhole_left(root_dir+"left/", root_dir+"result/", "PinholeLeft.yaml");
    Pinhole pinhole_right(root_dir+"right/", root_dir+"result/", "PinholeRight.yaml");
    //单目标定
    cout << "开始左目标定" << endl;
    pinhole_left.setParameter();
    cout << "开始右目标定" << endl;
    pinhole_right.setParameter();
    if(pinhole_left.isCali && pinhole_right.isCali){//单目标定成功
        cout << "左目标定结果:" << endl;
        pinhole_left.printInfo();
        pinhole_left.writeYaml();
        cout << "右目标定结果:" << endl;
        pinhole_right.printInfo();
        pinhole_right.writeYaml();
        //双目标定
        Stereo stereo(pinhole_left, pinhole_right, "Stereo.yaml");//
        stereo.setParameter();//
        if(stereo.isCali)//
        {
            stereo.printInfo();//
            stereo.writeYaml();//
        }else{
            cerr << "双目标定失败" << endl;
        }
    }
    else{//至少一目标定失败
        if(!pinhole_left.isCali){
            cerr << "左目标定失败" << endl;
        }
        if(!pinhole_right.isCali){
            cerr << "右目标定失败" << endl;
        }
    }
    return 0;
}