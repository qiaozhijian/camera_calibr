任务要求
+ 输入标定图片地址（./cali_dataset），输出单目标定文件，参考ORB-SLAM2里的euroc 单目配置文件
+ 输入标定左右目图片地址（例如./cali_dataset，此文件夹下有left和right两个子文件夹），输出双目标定文件，参考ORB-SLAM2里的euroc 双目目配置文件
+ 面向对象编程，一个单目标定的类，一个双目标定的类，可以参考 https://github.com/qiaozhijian/pose_graph

```
//单目类
class Pinhole{

}
int main()
{
  Pinhole pinhole("path_to_img");
  pinhole.setparameter("para", value);//在标定里可能会有一些参数，可有可无
  if(pinhole.cali())//标定
  {
    pinhole.printInfo();//如果标定成功，标定结果输出，内参，标定误差。。。
    pinhole.writeYaml();//输出参数到配置文件里
  }else{
    cout<<""<<endl;
  }
}
//双目类


class Stereo{

}
int main()
{
  Pinhole pinhole_left("path_to_img");
  Pinhole pinhole_right("path_to_img");
  ...//各自内参标定
  Stereo stereo(pinhole_left, pinhole_right);//在构造函数里传入单目参数
  stereo.setparameter("para", value);//在标定里可能会有一些参数，可有可无
  if(stereo.cali())//标定
  {
    stereo.printInfo();//如果标定成功，标定结果输出，内参，标定误差。。。
    stereo.writeYaml();//输出参数到配置文件里
  }else{
    cout<<""<<endl;
  }
}

```
+ 给每一个函数加注释，特别是opencv标定函数里输入输出参数的物理意义都是什么
+ 接受邀请
  + https://github.com/qiaozhijian/camera_calibr/invitations
+ 参考资料
  + https://github.com/qiaozhijian/camera_calibr/invitations
  + https://github.com/qiaozhijian/stereo_calibr
  + https://zhuanlan.zhihu.com/p/74133719
  + https://zhuanlan.zhihu.com/p/87185139
  + https://zhuanlan.zhihu.com/p/87605784
