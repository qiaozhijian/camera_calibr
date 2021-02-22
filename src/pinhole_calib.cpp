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
    Pinhole pinhole(root_dir+"right/", root_dir+"result/", "PinholeRight.yaml");
    pinhole.setParameter();
    if(pinhole.isCali){
        pinhole.printInfo();
        pinhole.writeYaml();
    }
    else{
        cout << "单目标定失败" << endl;
    }
    return 0;
}