/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
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
/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

//vector是一种容器，用它可以实现动态数据的创建，如同数据的增加、删除（push_back或者pop_back)等操作，
//它可以使用迭代器iterator来进行操作，如对容器中数据的遍历


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "ORBextractor.h"


using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

const int PATCH_SIZE = 31;         //进行FAST角点提取时所用圆的直径，同时也是计算角点方向时所用圆的直径（用于计算描述子BRIEF的特征点邻域大小）
const int HALF_PATCH_SIZE = 15;    //进行FAST角点提取时所用圆的半径，同时也是计算角点方向时所用圆的半径
const int EDGE_THRESHOLD = 19;     //进行ORB特征点检测时去除边缘的部分，为什么取出边缘，显而易见，一方面描述子不好确定，
                                   //另一方面提取角点时是通过周围范围内颜色对比产生的，因此去除取出边缘点

// 灰度质心法计算特征点方向  
// Point2f等价Point_<float>,point类常用于表示2维坐标，主要是图像中的像素点坐标，point.x、point.y。
// &表示对此的引用，用来向函数里面传递数据
static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;
  
  //uchar表示无符号数  
  //image待提取特征点的图片  pt是特征点的位置
  //image.at <uchar>(x,y)=128是指将灰度图的第x行、第y列的置为灰度为128的灰色（0为黑色，255为白色）
  //如果是才是图则写为image.at<cv::Vec3b>(x,y)[0]
                    image.at<cv::Vec3b>(x,y)[1]
                    image.at<cv::Vec3b>(x,y)[2]分别表示红绿蓝（顺序不确定）3种颜色的色度
  //访问像素用image.at<uchar>(v+i,u+j)
  //cvRound返回跟参数最接近的整数值（四舍五入）
    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    //-15≤u≤15，横坐标，对于v=0这一行单独计算，v=0这一行是特殊的
    //m_10=SUM(u*I(u,v))    m_01=SUM(v*I(u,v))
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    //每行含有的元素个数，要注意图像的step不一定是图像的宽度
    int step = (int)image.step1();
    //v=1，2，3……15
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        //上下和左右两条线同时计算
        //我们要在一个圆域中计算出m10和m01，先算出中间红线的m10，然后再平行于x轴算出m01和m10，一次计算相当于图像中同个颜色的两个line
        int v_sum = 0;
        int d = u_max[v];
        //-d≤u≤d
        //u + v*step表示第v行第u个像素的像素值
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);                  //计算上下的时候是有符号的，所以这边是减（此时val_minus对应-v）
            m_10 += u * (val_plus + val_minus);               //将(u,v)和(u,-v)两个点的像素一起计算，这边加是由于u已经确定好了符号
        }
        m_01 += v * v_sum;                                    //将x=v这条直线上所有的坐标点的像素值求和在进行计算
    }

    return fastAtan2((float)m_01, (float)m_10);               //返回计算的角度 
}


const float factorPI = (float)(CV_PI/180.f);                  //弧度制与角度的转换


//计算kpt这个特征点的描述子  img是整个图片  pattern是按照一定概率分布随机得到的特征点周围的点对 此处取了256个点（128点对）
static void computeOrbDescriptor(const KeyPoint& kpt,
                                 const Mat& img, const Point* pattern,
                                 uchar* desc)
{
    float angle = (float)kpt.angle*factorPI;               //取特征点的方向角度并进一步转化为弧度
    float a = (float)cos(angle), b = (float)sin(angle);    

    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    const int step = (int)img.step;



  //取旋转后一个像素点的值（根据特征点的角度，特征点的位置，已获得表（pattern）来采样，获得采样点的灰度值）
  //使得该关键点具有旋转不变性
  //center[0]给出了特征点的坐标，偏差点表示是（y，x）T，乘以旋转矩阵后就是下列式子中的GET_VALUE(idx)获取相应的点。
  //描述子矩阵中的纵坐标的个数是特征点的个数，横坐标表示的是一个特征点的描述子总共有32*8（256）位
    #define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]          
  




    //对于单个特征点，其32位描述子计算
    //循环32次，pattern取值16*32=512，也即每次取16个点，共形成8个点对
    //8个点对比较可以形成8bit（1byte）长度的特征描述数据
    //最终得到的ORB描述符是256bit(特征点附近256个点对)，可以用256/8=32个int类型进行存储
    //所以每个特征点可以用一个32个bit空间来存储其描述符，所以分成32组，每组8个点对，每次取16个点。
    // 生成8*32 = 256维的向量代表这个关键点，并通过将点的坐标转换到以描述子主方向为X轴的坐标系下，从而使得描述子具有旋转不变性。
    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;                      // |=是位操作运算符的一种，a|=b代表a=a|b，即把a和b做按位或操作，结果赋值给a
                                                    // 先小于再左移再或
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

    #undef GET_VALUE  
}

//计算描述子的pattern、高斯分布(随机得到关键点周围的点)，也可以使用其他定义的pattern
//bit_pattern_31_是一个一维数组，里面放了512个偏差点（256个点对），并且把这些点给类的point类型的pattern数组，并且把这些点都经过相应的旋转，即S西塔=R西塔S
static int bit_pattern_31_[256*4] =α
{
    8,-3, 9,5/*mean (0), correlation (0)*/,
    4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
    -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
    7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
    2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
    1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
    -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
    -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
    -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
    10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
    -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
    -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
    7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
    -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
    -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
    -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
    12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
    -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
    -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
    11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
    4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
    5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
    3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
    -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
    -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
    -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
    -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
    -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
    -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
    5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
    5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
    1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
    9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
    4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
    2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
    -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
    -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
    4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
    0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
    -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
    -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
    -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
    8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
    0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
    7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
    -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
    10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
    -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
    10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
    -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
    -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
    3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
    5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
    -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
    3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
    2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
    -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
    -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
    -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
    -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
    6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
    -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
    -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
    -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
    3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
    -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
    -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
    2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
    -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
    -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
    5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
    -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
    -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
    -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
    10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
    7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
    -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
    -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
    7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
    -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
    -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
    -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
    7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
    -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
    1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
    2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
    -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
    -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
    7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
    1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
    9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
    -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
    -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
    7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
    12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
    6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
    5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
    2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
    3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
    2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
    9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
    -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
    -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
    1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
    6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
    2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
    6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
    3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
    7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
    -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
    -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
    -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
    -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
    8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
    4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
    -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
    4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
    -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
    -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
    7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
    -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
    -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
    8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
    -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
    1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
    7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
    -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
    11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
    -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
    3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
    5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
    0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
    -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
    0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
    -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
    5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
    3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
    -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
    -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
    -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
    6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
    -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
    -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
    1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
    4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
    -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
    2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
    -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
    4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
    -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
    -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
    7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
    4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
    -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
    7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
    7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
    -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
    -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
    -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
    2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
    10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
    -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
    8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
    2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
    -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
    -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
    -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
    5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
    -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
    -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
    -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
    -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
    -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
    2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
    -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
    -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
    -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
    -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
    6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
    -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
    11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
    7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
    -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
    -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
    -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
    -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
    -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
    -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
    -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
    -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
    1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
    1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
    9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
    5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
    -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
    -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
    -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
    -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
    8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
    2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
    7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
    -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
    -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
    4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
    3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
    -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
    5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
    4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
    -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
    0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
    -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
    3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
    -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
    8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
    -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
    2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
    10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
    6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
    -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
    -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
    -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
    -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
    -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
    4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
    2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
    6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
    3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
    11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
    -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
    4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
    2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
    -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
    -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
    -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
    6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
    0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
    -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
    -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
    -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
    5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
    2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
    -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
    9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
    11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
    3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
    -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
    3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
    -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
    5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
    8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
    7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
    -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
    7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
    9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
    7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
    -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
         int _iniThFAST, int _minThFAST):
    nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
    iniThFAST(_iniThFAST), minThFAST(_minThFAST)
{
    mvScaleFactor.resize(nlevels);      // mvScaleFactor计算每一层相对于原始图片的放大倍数,存储金字塔中每层图像对应的尺度因子的vector变量
    mvLevelSigma2.resize(nlevels);      // mvLevelSigma2是该层尺度因子的平方
    mvScaleFactor[0]=1.0f;              // 将vector中的第一个元素初始化为1
    mvLevelSigma2[0]=1.0f;
    
    
    //计算金子塔中每一层图像对应的尺度因子和尺度因子的平方
    //可以发现金子塔中的0层的尺度因子是1,然后每向上高一层,图像的尺度因子是在上一层图像的尺度因子上乘以scaleFactor，在本工程下该值为1.2
    //1      1*1.2    1*1.2*1.2      1*1.2*1.2*1.2
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;                  
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    
    //计算每一层想对于原始图片放大倍数的逆
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mvImagePyramid.resize(nlevels);      //mvImagePyramid是一个存储金字塔图像的vector，vector中每一个元素都是用Mat数据类型表示的图像 
                                         //std：：vector<cv::Mat> mvImagePyramid

    mnFeaturesPerLevel.resize(nlevels);   //mnFeaturesPerLevel是一个存储金字塔中每层图像应该提取的特征点个数
    float factor = 1.0f / scaleFactor;    //scaleFactor：1.2   nfeatures：1000   nlevels：8
    
    
    
    //为什么第一层的特征点数量为这些，这是由于
    //  第一层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))
    //  第二层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))*q
    //  第三层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))*q*q
    //   .........
    //  第nlevels层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))*q^nlevels
    //  其中q=factor
    //  那么前nlevels层的和为总的特征点数量nfeatures（等比数列的前n相和）
    //  可以看出ORB特征点是如何实现尺度不变性的，原始图像那一层特征点最多，依次递减
    //   主要是将每层的特征点数量进行均匀控制
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));    //计算第一层特征点的数量

    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);      //mnFeaturesPerLevel数组存储每层特征点的数量
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);  //最大层需要的特征点数=需要的特征点数-其他所有层的特征点总和
    //复制训练的模板
    const int npoints = 512;
    const Point* pattern0 = (const Point*)bit_pattern_31_;                 //指针指向首地址
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));  //pattern0 是要拷贝元素的首地址
                                                                           //pattern0 + npoints要拷贝元素的最后一个元素的下一个地址
                                                                           //pattern也是指针，拷贝到的目的地的首地址

    //This is for orientation
    // pre-compute the end of a row in a circular patch
    umax.resize(HALF_PATCH_SIZE + 1);                                     //用于计算特征方向时，每个v坐标对应最大的u坐标，定义一个vector
    
    
    // 将v坐标划分为两部分进行计算，主要为了确保计算特征主方向的时候，x,y方向对称
    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);       //cvFloor含义是取不大于参数的最大整数值
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);                   //cvCeil含义是取不小于参数的最小整数值
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;  
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));     //勾股定理算出v坐标的第一部分

    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)    //确保对称，即保证是一个圆
    {
        while (umax[v0] == umax[v0 + 1])          //算出v坐标的第二部分
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

//计算特征的方向，计算每个关键点的角度，方法是计算以特征点为中心，已像素为权值的圆形区域上的重心，以中心和重心的连线作为该特征点方向
static void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax)
{
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),              
         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)            //为每个关键点计算主方向
    {
        keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}


//节点代表区域，将每个新节点里用来存储特征点的向量的大小设置为母节点中所有特征点的个数
void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
{
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //Define boundaries of childs
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    //根据特征点的坐标来将特征点分配到不同的新节点区域
    for(size_t i=0;i<vKeys.size();i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }



  //最后根据每个节点中分得到的特征点的数目来设定bNomore变量的真假
  //bNomore表示每个节点中只有一个特征点
    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}



//vToDistributeKeys是由第level层的所有fast关键点组成的vector
//minX 最小x坐标    maxX 最大x坐标 minY 最小y坐标 maxY 最大y坐标
//N 第level层的特征点数量
vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
{
    // Compute how many initial nodes   
    //确定四叉树有多少个初始的节点数量（为了满足每个节点是正方形的，根节点的数量）  宽度除以高度
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));     //round四舍五入取整，水平划分格子的数量
    
    //节点在x方向上的长度（水平划分格子的宽度）
    const float hX = static_cast<float>(maxX-minX)/nIni;           //static_cast<type-id>(expression)功能是把expression转换为type-id类型

    list<ExtractorNode> lNodes;

    vector<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);
    
    
    //UL、UR、BL、BR是节点的四个角的坐标
    for(int i=0; i<nIni; i++)
    {
        ExtractorNode ni;           //类 定义四叉树节点变量ni
        ni.UL = cv::Point2i(hX*static_cast<float>(i),0);    //左上
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);  //右上
        ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
        ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);             //lNodes中存储生成的树节点
        vpIniNodes[i] = &lNodes.back();   //返回lNodes最后元素值，vpIniNodes变量中存储的是节点的地址
    }

    push与push_back都是STL中常见的方法，都是向数据结构中添加元素
    reserve（Container::size_type n）强制容器把它的容量改为至少为n
    coll.push_back()是把一个元素放入到这个容器的末尾，相当于末尾添加一个元素
    coll.back（）是获取最后一个元素的迭代器，可以理解为最后一个元素的指针



    //Associate points to childs
    //把关键点分配给初级节点中
    for(size_t i=0;i<vToDistributeKeys.size();i++)
    {
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);      //->成员访问运算符，获取A指针指向的B成员
    }

    list<ExtractorNode>::iterator lit = lNodes.begin();   //设计节点迭代器

    while(lit!=lNodes.end())       //遍历节点列表
    {
        if(lit->vKeys.size()==1)  //如果节点中只有一个特征点时，不在进行划分
        {
            lit->bNoMore=true;
            lit++;
        }
        else if(lit->vKeys.empty())  //如果该节点中无特征点时，就删除该节点
            lit = lNodes.erase(lit);
        else
            lit++;
    }

    bool bFinish = false;     //完结标志定义

    int iteration = 0;        //迭代计数

    vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;   //定义新数据类型，节点及其所包含的特征数
    vSizeAndPointerToNode.reserve(lNodes.size()*4);            //一共有几个子节点，从初始节点分叉

    while(!bFinish)            //对于每个Node而言，若其只有一个特征点，bNomore为true，表明其不用继续划分
    {
        iteration++;

        int prevSize = lNodes.size();  // 初始节点个数，用于判断是否节点再一次进行了划分（老节点的数量）

        lit = lNodes.begin();          //lit是节点迭代器

        int nToExpand = 0;             //表示节点分解次数

        vSizeAndPointerToNode.clear();

        while(lit!=lNodes.end())
        {
            if(lit->bNoMore)           
            {
                // If node only contains one point do not subdivide and continue
                lit++;          //表面只有一个特征点，不在进行划分
                continue;
            }
            else
            {
                // If more than one point, subdivide
                //如果超过一个特征点，则继续划分
                ExtractorNode n1,n2,n3,n4;
                lit->DivideNode(n1,n2,n3,n4);      //将该节点划分为4个节点

                // Add childs if they contain points
                //如果子节点中包含着特征点就添加该节点，否则删除节点
                if(n1.vKeys.size()>0)           //特征点数>0，节点插入list头部
                {
                    lNodes.push_front(n1);      //把子节点推到lNodes前面，变成lNodes的一部分               
                    if(n1.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));   
                        lNodes.front().lit = lNodes.begin();         //begin（）迭代器，赋给第一个元素的迭代器
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                
                
                //删除该节点，因为有字节点包含了这些特征点了，所以为了减少冗余节点就删除该父节点
                lit=lNodes.erase(lit);  
                continue;
            }
        }       

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        // 当节点个数大于需分配的特征数或者所有的节点只有一个特征点（节点不能划分）的时候，则结束。
        if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
        {
            bFinish = true;
        }
        
        //节点展开次数乘以3用于表明下一次的节点分解可能超过特征数，即为最后一次分解
        else if(((int)lNodes.size()+nToExpand*3)>N)       
        {

            while(!bFinish)
            {

                prevSize = lNodes.size();

                vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());      //sort（）是C++给的一种排序函数、语法描述:sort(begin、end、mp)默认升序排列
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)           //特征点数多的节点优先划分
                {
                    ExtractorNode n1,n2,n3,n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);      

                    if((int)lNodes.size()>=N)
                        break;          //break跳出for循环，if不算循环
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;

            }
        }
    }

    // Retain the best point in each node
    vector<cv::KeyPoint> vResultKeys;             //取出每个节点中响应最大的特征点，用keypoint数据类型
    vResultKeys.reserve(nfeatures);               //容器的容量为特征点数的大小
    for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;      //获取每个节点下的特征点
        cv::KeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;     

        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            if(vNodeKeys[k].response>maxResponse)          //作者选取的是opencv中keypoint类的response成员变量表征其质量的高低
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}




//对影像金子塔中的每一层图像进行特征点的计算。具体计算过程是将影像网格分割成小区域，每个小区域独立使用fast角点检测
//检测完成之后使用Distrute OcTree函数对检测到所有的角点进行筛选，使角点分布均匀
// allKeypoints表示所有图层图像上特征点容器，容器的容器


// 首先在图像四周去掉长度为EDGE_THRESHOLD-3个单位的像素点的边界，对去掉边界之后的图像网格化，每个窗口的大小为w个像素点的大小
//然后依次在划分好的窗口中提取FAST关键点，这样子的目的是为了使得每个网格都有特征，从而使得特征点在图像上的分布相对均匀点。


//如果在有的窗口中提取的特征点数为0，则降低阈值（EDGE_THRESHOLD）继续提取，然后对提取出来的关键点换算出其位于（level层的被裁掉边界图像）的
//位置，并将每个窗口中的关键点存入vToDistribute Keys容器中。vToDistribute Keys容器就暂时保存着level层图像的关键点。
//然后将这些特征点送入Distribute OctTree函数，剔除一些关键点。将剔除后留下的关键点存入allkeypoints[level]容器中
void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints)
{
  
  
    allKeypoints.resize(nlevels);
    
    
    //设定每个格子的大小，窗口的大小
    const float W = 30;

    for (int level = 0; level < nlevels; ++level)
    {
      
        //设定该层图像中检测的X，Y最大最小坐标   由于PATCH 15 和EDGE_THRESHOLD 19 相差4 所以应该减去3,从每副图像的第16个元素开始查询FAST关键点
        //为何要去除边缘呢，是因为边缘的特征点不具有代表性
        const int minBorderX = EDGE_THRESHOLD-3;       //得到每层图像进行特征检测区域上下两坐标
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;
        
        
        //每层待分配关键点数量
        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures*10);
        
        //计算总面积长宽
        const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);
        
        
        //将原始图片分割的行数和列数
        const int nCols = width/W;
        const int nRows = height/W;
        
        //重新计算每个格子（窗口）的大小
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);
        
        //遍历每个窗口，计算窗口内的四个坐标，在每个格子内进行fast特征检测
        for(int i=0; i<nRows; i++)
        {
            const float iniY =minBorderY+i*hCell;  //iniY、maxY为窗口的行上坐标和下坐标
            
            //为什么加6呢，是因为在FAST角点检测时是需要3为半径的圆周围的16个点进行比较的，因此会加6
            float maxY = iniY+hCell+6;
            
            //窗口的行上坐标超出边界，则放弃此行
            if(iniY>=maxBorderY-3)
                continue;
                
            //窗口的行下坐标超出边界，则将窗口的行下坐标设置为边界
            if(maxY>maxBorderY)
                maxY = maxBorderY;

            for(int j=0; j<nCols; j++)
            {
                const float iniX =minBorderX+j*wCell;    //iniX、 maxX为窗口的列左坐标和右坐标
                float maxX = iniX+wCell+6;               //这里注意窗口之间有6列重叠
                
                //窗口的列左坐标超出边界，则放弃此列
                if(iniX>=maxBorderX-6)                   
                    continue; 
                    
                //窗口的列右坐标超出边界，则将窗口的列右坐标设置为边界
                if(maxX>maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;         //每个小窗口里的关键点Keypoint将存在这里
                FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),       //level层的图片中行范围（iniY,maxY)、列范围（iniX,maxX）的截图
                     vKeysCell,iniThFAST,true);
                
                
                //如果检测为空就降低阈值在进行检测
                if(vKeysCell.empty())
                {
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,minThFAST,true);
                }
                
                //如果检测不为空就将检测到的特征点放到vToDistributeKeys中
                if(!vKeysCell.empty())
                {
                    for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                    {
                        (*vit).pt.x+=j*wCell;       //根据前面的行列计算实际的位置
                        (*vit).pt.y+=i*hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }

            }
        }

        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures);

        //vToDistributeKeys存储该层的关键点 minBorderX 最小x坐标    maxBorderX 最大x坐标 minBorderY最小y坐标 maxBorderY最大y坐标 
	      //mnFeaturesPerLevel[level]该层检测到的特征点数量 level当前特征点所在的层数
        keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                      minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);
        
        //计算特征点Patch的大小，根据每层的尺度的不同而不同
        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Add border to coordinates and scale information
        // 将边界信息考虑进去计算特征点的位置，得到特征点在图片中的真实坐标
        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;       //特征点的x坐标
            keypoints[i].pt.y+=minBorderY;       //特征点的y坐标     
            keypoints[i].octave=level;           //特征点所在的层数
            keypoints[i].size = scaledPatchSize; //特征点Patch的大小将来计算描述子时使用
        }
    }

    // compute orientations
    //计算特征点的方向
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}




//传统方法进行特征点提取与匹配
//传统方法的第一步计算将图像分割成多少个元包（cell），对于每个元包分别提取特征点。
//元包的计算方法为，根据需要提取的特征点数目，假设每个元包中需要提取5个特征点，以此来计算需要的cell数目。

//接着对上面计算好的元包分别进行特征点的提取。这里注意，由于FAST特征在计算角点时向内缩进了3个像素才开始计算，所以在
//使用FAST之前还需要对图像加一条宽度为3像素的边。
//然后就要用到之前初始化的两个阈值参数，首先使用阈值较大的参数作为FAST特征点检测的阈值。如果提取的特征点数目足够多，那么直接计算
//下一个元包即可，否则就要使用较小的参数重新提取。在本项目中，特征点数目的阈值设定为3
void ORBextractor::ComputeKeyPointsOld(std::vector<std::vector<KeyPoint> > &allKeypoints)
{
    allKeypoints.resize(nlevels);

    float imageRatio = (float)mvImagePyramid[0].cols/mvImagePyramid[0].rows;       //图像横纵比

    for (int level = 0; level < nlevels; ++level)
    {
        const int nDesiredFeatures = mnFeaturesPerLevel[level];    //每一个cell中特征点的个数

        const int levelCols = sqrt((float)nDesiredFeatures/(5*imageRatio));
        const int levelRows = imageRatio*levelCols;

        //得到每一层图像进行特征检测区域的上下两个坐标
        const int minBorderX = EDGE_THRESHOLD;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD;
        
        //将待检测区域划分为格子的行列个数
        const int W = maxBorderX - minBorderX;
        const int H = maxBorderY - minBorderY;
        const int cellW = ceil((float)W/levelCols);
        const int cellH = ceil((float)H/levelRows);

        const int nCells = levelRows*levelCols;
        const int nfeaturesCell = ceil((float)nDesiredFeatures/nCells);
        
        //vector<T>v(n,i)即是向量v中包含n个值为i的元素，所以cellKeyPoints有levelRows层，每一层中又有levelCols层，均初始化为0
        //vector<vector<int>>b实例化为一个名为b的vector。这个vector当中存的是一系列的vector<int>。实例化的同时，放入levelRows个默认值。
        //默认值的内容是vector<int>(levelCols,0),即长度为levelCols，里面填充值为0的vector<int>
        vector<vector<vector<KeyPoint> > > cellKeyPoints(levelRows, vector<vector<KeyPoint> >(levelCols));

        vector<vector<int> > nToRetain(levelRows,vector<int>(levelCols,0));           //
        vector<vector<int> > nTotal(levelRows,vector<int>(levelCols,0));              //
        vector<vector<bool> > bNoMore(levelRows,vector<bool>(levelCols,false));       //想象成一个表，表里全都是0
        vector<int> iniXCol(levelCols);
        vector<int> iniYRow(levelRows);
        int nNoMore = 0;
        int nToDistribute = 0;


        float hY = cellH + 6;     //关于3，每一行都留出3个像素的宽度

        for(int i=0; i<levelRows; i++)
        {
            const float iniY = minBorderY + i*cellH - 3;    //第i个cell的第一个Y
            iniYRow[i] = iniY;
            
            //如果循环到了最后一行
            if(i == levelRows-1)
            {
                hY = maxBorderY+3-iniY;     //hY=3+Ymax-iniY
                                                =3+Ymax-(Ymin+(levelRows-1)*cellH-3)
                                                =6+Ymax-Ymin-H+cellH
                                                =cellH+6
                                            //hY牵扯到后面cellimage的大小，范围从iniY到iniY+hY不可能为负值
                if(hY<=0)
                    continue;     //continue只管for、while，不管if，不管多少if都直接无视
            }

            float hX = cellW + 6;

            for(int j=0; j<levelCols; j++)
            {
                float iniX;

                if(i==0)
                {
                    iniX = minBorderX + j*cellW - 3;
                    iniXCol[j] = iniX;
                }
                else
                {
                    iniX = iniXCol[j];       //和第一行的x值
                }


                if(j == levelCols-1)
                {
                    hX = maxBorderX+3-iniX;  //同上
                    if(hX<=0)
                        continue;
                }


                Mat cellImage = mvImagePyramid[level].rowRange(iniY,iniY+hY).colRange(iniX,iniX+hX);     //Mat.rowRange（int x，int y）函数取的实际行数y-x，只取到范围的左边界，而不取右边界

                cellKeyPoints[i][j].reserve(nfeaturesCell*5);         //论文中每个网格（cell）中至少5个特征点，nfeaturesCell每个cell中特征点个数

                FAST(cellImage,cellKeyPoints[i][j],iniThFAST,true);   //FAST检测关键子

                if(cellKeyPoints[i][j].size()<=3)
                {
                    cellKeyPoints[i][j].clear();

                    FAST(cellImage,cellKeyPoints[i][j],minThFAST,true);  //降低阈值重新检测关键子
                }


                const int nKeys = cellKeyPoints[i][j].size();            //网格中到底有多少关键点
                nTotal[i][j] = nKeys;                                    //nTotal总点数

                if(nKeys>nfeaturesCell)
                {
                    nToRetain[i][j] = nfeaturesCell;                     //保存预先计算好的数目的点
                    bNoMore[i][j] = false;                               //nomore为假
                }
                else
                {
                    nToRetain[i][j] = nKeys;                             //否则先知道要保存的数目
                    nToDistribute += nfeaturesCell-nKeys;               //还有多少需要离散的点的数目
                    bNoMore[i][j] = true;
                    nNoMore++;
                }

            }
        }


        // Retain by score
        
        //如果总共的离散点数目大于0并且未达到阈值的cell数目比总共的格网数小
        //直到不需要离散，不需要加点为止
        while(nToDistribute>0 && nNoMore<nCells)
        {
            //((float)nToDistribute/(nCells-nNoMore))在分配各区域选取的特征点数目时，就要考虑前面提到的极可能均匀的问题。
            //所以采用的方法是循环将特征点数目不足的元包中的剩余数目分配到其他所有元包中，直到最后取得足够数量的特征点。
            //当然，如果最初提取的特征点数目就不足预期，那么直接全部选取即可。所以这种方法并不能保证最终得到的特征点数目移动能达到1000
            //对于那些特征点数目特别多的元包，采用的是对各角点质量进行排序，选择最好的前n个特征点
            int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute/(nCells-nNoMore));
            nToDistribute = 0;

            for(int i=0; i<levelRows; i++)
            {
                for(int j=0; j<levelCols; j++)
                {
                    if(!bNoMore[i][j])          //有足够点数的cell
                    {
                        if(nTotal[i][j]>nNewFeaturesCell)     //总数目甚至比新的要求的点数还要多（当所有cell都执行这个条件语句，while循环就可以终止了
                        {
                            nToRetain[i][j] = nNewFeaturesCell;    //只保存新要求的点的数目即可
                            bNoMore[i][j] = false;
                        }
                        else
                        {
                            nToRetain[i][j] = nTotal[i][j];
                            nToDistribute += nNewFeaturesCell-nTotal[i][j];    //还要离散的点的数目
                            bNoMore[i][j] = true;                              //还需要在加点
                            nNoMore++;   
                        }
                    }
                }
            }
        }

        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nDesiredFeatures*2);

        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Retain by score and transform coordinates
        //换算特征点真实位置（添加边界值），添加特征点的尺度信息
        for(int i=0; i<levelRows; i++)
        {
            for(int j=0; j<levelCols; j++)
            {
                vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
                KeyPointsFilter::retainBest(keysCell,nToRetain[i][j]);     //retainBest：根据响应保留指定数目的特征点
                if((int)keysCell.size()>nToRetain[i][j])
                    keysCell.resize(nToRetain[i][j]);


                for(size_t k=0, kend=keysCell.size(); k<kend; k++)
                {
                    keysCell[k].pt.x+=iniXCol[j];
                    keysCell[k].pt.y+=iniYRow[i];
                    keysCell[k].octave=level;
                    keysCell[k].size = scaledPatchSize;
                    keypoints.push_back(keysCell[k]);
                }
            }
        }

        if((int)keypoints.size()>nDesiredFeatures)      //特征点还多的话，再进行一次滤波
        {
            KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
            keypoints.resize(nDesiredFeatures);
        }
    }

    // and compute orientations                         //计算方向
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

//遍历该层的所有关键点，进行计算描述子
static void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors,
                               const vector<Point>& pattern)
{
  
  
    //CV_[每一项的位数][有符号或无符号][类型前缀]C[通道数]
    descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);   //初始化描述子

    for (size_t i = 0; i < keypoints.size(); i++)                   //计算每个关键点的描述子
        computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));     //ptr（指针）pointer缩写
}



//_image；获取的灰度图像    _mask：掩码     _keypoints：关键点   _descriptors：描述子
void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                      OutputArray _descriptors)
{ 
    if(_image.empty())    //如果没有获取图片，返回
        return;

    Mat image = _image.getMat();          //提取输入图像矩阵
    //判断通道是否为灰度图
    assert(image.type() == CV_8UC1 );     //assert()是一个调试程序是常用的宏，在程序运行时它计算括号内的表达式，如果表达式为FALSE(0),程序将报告错误，并终止执行
                                          //如果表达式不为0，则继续执行后面的语句

    // Pre-compute the scale pyramid
    ComputePyramid(image);                //计算图像金字塔

    vector < vector<KeyPoint> > allKeypoints;     //提取图像金字塔中各层图像的关键点并存储在allKeypoints里
    ComputeKeyPointsOctTree(allKeypoints);
    //ComputeKeyPointsOld(allKeypoints);

    Mat descriptors;

    int nkeypoints = 0;                                       //算出实际提取出来的关键点数量nkeypoints*32，新建descriptors对象，特征点的描述子将存在这里
                                                              //descriptors就好似nkeypoints*32维度矩阵，矩阵里存着一个uchar，其单位为8bit，这样对于每个关键点，就对应有32*8=256bit的二进制向量作为描述子
    for (int level = 0; level < nlevels; ++level)descriptors;
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();                  //InputArray是opencv中的一个接口类
                                                              //InputArray：：getMat（）函数将传入的参数转换为Mat结构
                                                              //OutputArray是InputArray的派生类‘
                                                              //OutputArray：：getMat（）在使用前一定要调用OutputArray：：create（）为矩阵分配空间
    }

    _keypoints.clear();
    _keypoints.reserve(nkeypoints);
    
    
    //计算每个关键点对应的描述子
    int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image
        //高斯滤波：void GaussianBlur(InputArray src,OutputArray dst,Size ksize,double sigmaX,double sigmaY,int border Type=BORDER_DEFAULT)
        //对输入图像src进行高斯滤波后用dst输出，ksize为高斯滤波模板大小，sigmaX和sigmaY分别为高斯滤波在横向和竖向的滤波系数，border Type为边缘点插值类型
        //进行高斯模糊，用BORDER_REFLECT_101方法处理边缘
        Mat workingMat = mvImagePyramid[level].clone();
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

        // Compute the descriptors
        //offset是偏移量，此处取出的是该层的描述子，desc是描述子
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        //计算描述子采用的是高斯分布取点，就是上面一长串的pattern
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        //对关键点的位置坐尺度恢复，恢复到原图的位置
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;                 //pt表示坐标，乘号表示按缩放因子进行坐标缩放
        }
        // And add the keypoints to the output
        
        //void insert(iterator loc,input_interator start,input_iterator end)在指定位置loc前插入区间[start,end]的所有元素
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}


//建立图像金字塔，将原始图像一级级缩小并以此存在mvImagePyramid里
//金字塔的底部是待处理图像的高分辨率表示，而顶部是低分辨率近似。
//层级越高图像越小，尺寸越小，分辨率越低
void ORBextractor::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];       //获取缩放尺度
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));    //当前图片的尺寸
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);      //截图前的图片尺寸
        
        
        //新建一个temp，大小为wholeSize
        //Mat image1(Size(5，3），cv_8uc3)创建大小为5*3，类型为8位3通道
        Mat temp(wholeSize, image.type()), masktemp;
        
        //从temp裁剪，存入mvImagePyramid[level]
        //Rect(int_x,inty_y,int_width,int_height):左上角x坐标、左上角y坐标，矩形的宽，矩阵的高
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 )
        {
          
            //从上一级图像mvImagePyramid[level-1]中图像缩小至sz大小，并存入mvImagePyramid[level]
            //resize(InputArray src,OutputArray dst,Size dsize,double fx=0,double fy=0,int interpolation=INTER_LINEAR)
            //dsize相当于直接规定了缩放后要得到的大小，最后一个表示插值方式，默认是线性插值
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
            
            
            //copyMakeBorder（InputArray src,OutputArray dst,int top,int bottom,int left,int right,int borderType)
            //输入图像、输出图像、表示对边界各个方向添加到像素个数，就是边框的粗细程度，
            //BORDER_REFLECT_101:反射101
            //扩充上下左右边界EDGE_THRESHOLD个像素，放入temp中，对mvImagePyramid无影响
            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101+BORDER_ISOLATED);            
        }
        else
        {    
            //滤波处理时要考虑图像边界，为图像增加一定的边缘，以适应卷积核在原图像边界的操作
            //第一张为原图分辨率，无需缩放
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101);            
        }
    }

}

} //namespace ORB_SLAM
