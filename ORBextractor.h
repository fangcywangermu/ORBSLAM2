/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Ra煤l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>  //vector是一种顺序容器，和数组差不多
#include <list>    //list将元素按顺序存储在链表中，与vector相比，它运行快速的插入和删除，但是随机访问比较慢
#include <opencv/cv.h>




ORB特征：Orientede FAST and Rotated BRIEF   对FAST特征点和BRIEF特征描述子的一种结合与改进


namespace ORB_SLAM2
{
  
//定义一种四叉树节点类型的类，类里面定义了一些成员及函数
//四叉树是在二维图片中定位像素的唯一适合的算法
//将图层不停四叉树分割，然后取其中比较好的keypoints保留
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);  //分配节点函数

    std::vector<cv::KeyPoint> vKeys;    //节点keypoints容器
    cv::Point2i UL, UR, BL, BR;         //二维整数点类型数据u的上下左右像素
    std::list<ExtractorNode>::iterator lit;     //节点类型列表迭代器
    bool bNoMore;                               //确认是否只含有一个特征点
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

  

nfeatures  ORB特征点数量
scaleFactor  金字塔中相邻层图像的比例系数
nlevels      构造金字塔的层数
iniThFAST    提取FAST角点时初始阈值
minThFAST    提取FAST角点时更小的阈值
设置两个阈值的原因是在FAST提取角点进行分块后有可能在某一块中在原始阈值情况下提取不到角点，使用更小的阈值进一步提取
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}     

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    // 重载了()运算符，作为提取器的对外接口，传入引用参数keypoints、descriptors用于存储计算得到的特征点及描述子
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }
  //图像金字塔 存放各层的图片
    std::vector<cv::Mat> mvImagePyramid;

protected:
    //计算高斯金字塔
    void ComputePyramid(cv::Mat image);
    //计算特征点并用四叉树进行存储
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    //为特征点分配四叉树
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);
                                           
    //对影像金子塔中的每一层图像进行特征点的计算，具体的计算过程是将影像格网分割成为小区域，每个小区域独立使用FAST角点检测。
    //检测完成后使用DistributeOctTree函数对检测得到的所有角点进行筛选，使得角点分布均匀                                       

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;      //存储特征点附近patch的点对

    int nfeatures;                       //提取特征点的最大数量
    double scaleFactor;                 //每层之间的缩放比例
    int nlevels;                        //高斯金字塔的层数
    int iniThFAST;                      //iniThFAST提取FAST角点时初始阈值
    int minThFAST;                      //minThFAST提取FAST角点时更小的阈值

    std::vector<int> mnFeaturesPerLevel;  //每层的特征数量

    std::vector<int> umax;                //Patch圆的最大坐标

    std::vector<float> mvScaleFactor;         //每层的相对于原始图像的缩放比例
    std::vector<float> mvInvScaleFactor;      //每层的相对于原始图像的缩放比例的倒数
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif



