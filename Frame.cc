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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
//下边这些都是静态成员变量，因此只能在类外附初值，并且属于整个类，每个对象所共有的
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    //同时对左右目提特征
    //提取特征加入双线程同步提取，0、1代表左目和右目
    //两张提取的特征点会放在不同的vector中
    //对于单目和RGBD来说，右目不用，以左为准
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    
    
    threadLeft.join();   //该函数在线程执行完成时返回
    threadRight.join();  //在调用这个函数之后，线程对象变为不可连接的并且可以被安全地销毁

    N = mvKeys.size();   //N为特征点的数量，这里是一个装有特征点keys的vector容器

    if(mvKeys.empty())   //如果提取的特征点数目为0，则直接返回
        return;

    UndistortKeyPoints(); //Undistort特征点，这里没有对双目进行校正，因为要求输入的图像已经进行极线校正

    ComputeStereoMatches();//计算双目间的匹配, 匹配成功的特征点会计算其深度
                           //深度存放在mvuRight 和 mvDepth 中
    
    //初始化地图点及其外点
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)//只执行一次
    if(mbInitialComputations)
    {  
        ComputeImageBounds(imLeft);      //得到mnMaxX,mnMaxY,mnMinX,mnMinY

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();     //按照特征点的像素坐标,分配到各个网格内,每个网格记录了特征点的序列下标
}


//RGBD构建的Frame对象，基本上和双目类似，只需要恢复出右图深度为正的深度即可
 *     函数属性：类Frame的构造函数（RGB-D 相机）
 *     函数功能：
 *                 1. 初始化该帧的ID
 *                 2. 初始化高斯金字塔的尺度参数
 *                 3. 提取图像的特征点
 *                 4. 对特征点进行失真矫正，并将相机的深度图特征点的深度值存储到容器中，便于调用
 *                 5.初始化该帧数据的地图点和局外点
 *                 6.如果为初始帧则将相机的相关参数重新加载进来
 *                 7.将特征点加入到网格中
 *     函数参数介绍：
 *                 imGray：是指该帧的rgb图对应的灰度图
 *                 imDepth：是指该帧的深度图
 *                 timeStamp：是获取该帧数据的时间
 *                 extractor：是指该帧数据的ORB提取器
 *                 voc：是存储字典的首地址
 *                 distCoef：是指相机的参数矩阵
 *                 bf：是指数据集图片尺寸可能是通过缩放得到的，这个就是缩放的尺寸
 *                 thDepth： 远近点的阈值
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    // 定义该帧的ID
    mnId=nNextId++;

    // Scale Level Info
    // 提取器高斯金字塔的相关尺度参数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    // ORB提取器  提取该帧图像中的特征点以及计算描述子
    // 图像ORB特征提取未校正的图像，得到关键点位置后，直接对关键点坐标进行校正
    ExtractORB(0,imGray);

    // 特征点的数量
    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    // 对关键点坐标进行校正，只是校正了左图的关键点：  mvKeys->畸变校正->mvKeysUn
    // 调用opencv的校正函数校正orb提取的特征点
    UndistortKeyPoints();

    // 将RGB-D相机的图像数据映射到双目相机下
    ComputeStereoFromRGBD(imDepth);
    
    //初始化该帧数据中的地图点和局外点，默认无局外点，无地图点
    //其中有N个空指针，因此这里的Mappoint并没有指向实际的地图点（虽然有对应的idx，但是是要进行判断的）
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    // 当第一次初始化帧时，需要将相机的相关参数都加载进来，再有帧时就不需要加载了，提高运行速度
    if(mbInitialComputations)
    {
        //计算图像的边界
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }
    //计算基线
    mb = mbf/fx;
    //将特征点分配到各个网格，目的是加速特征匹配
    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    // -1表示没有深度信息
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        // 640*480图像分成64*48个网格
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);  //每个像素占用的网格数
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY); //坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;
    // 将每个特征点分配到图片网格中
    AssignFeaturesToGrid();
}

 *      函数属性：类Frame的成员函数AssignFeaturesToGrid()
 *      函数功能：
 *                 将整张图片分为64×48的网格
 *                 并将每个特征点的id加入到该网格中，即mGrid容器存储的是特征点的id
 // 将关键点分布到64*48分割而成的网格中，为了加速匹配和均匀化关键点分布
void Frame::AssignFeaturesToGrid()
{
    //计算每个格子应该保留的特征点数目，0.5是随便取的，一种算法
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);       //每个网格内实际保存的是关键点在mvKeysUn中的序号，进而保存了落在当前网格的去除畸变的关键点
                                                 //因此这里需要对每个网格预留空间以保留序号，这里假定每个网格按照平均分配所有关键点后除以2，就是需要保留的空间
                                                 //这里实际上也是mGrid内部动态申请的内存的大小，尽量做到刚好包含每个网格实际包含的关键点个数

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];    //mvKeysUn每个校正后的关键点


        int nGridPosX, nGridPosY;                //存储网格位置，证明第(nGridPosX,nGridPosY)个网格
        if(PosInGrid(kp,nGridPosX,nGridPosY))    //如果第i个特征点位置在第(nGridPosX,nGridPosY)个网格中，就将该特征点的id存入该网格中
                                                 //当前关键点是否在划分的网格内部，因为关键点是去处畸变后的，因此可能超出划分网格的范围，需要判断一下
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}


 *     函数功能：
 *                 调用ORB提取器的()运算符，将得到该帧图像的关键点和描述子
 *                 将该帧图像的关键点存储到mvKeys
 *                     该帧图像的描述子存储到mDescriptors
 *     函数参数介绍：
 *                 flag：提取图像的标志  0代表左提取   1代表右提取
 *                 im：待提取ORB特征的图像(灰度图)
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}


 *     函数功能：
 *                 给该帧设置相机位姿（变换矩阵T）
 *     函数参数介绍：
 *                 Tcw：该帧数据的相机位姿
void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}




void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);    //Pc=RPw+t   ：世界到相机旋转矩阵得到P的相机坐标
    mRwc = mRcw.t();                            //矩阵转置，注意R为3*3正交旋转矩阵  ：相机到世界旋转矩阵
    mtcw = mTcw.rowRange(0,3).col(3);           //rowRange（0，3）从第0行开始，往后数3行，即取0，1，2行   ：世界到相机平移向量（mtwc=-mtcw）
    mOw = -mRcw.t()*mtcw;                       //mOw = mRwc()*mtwc    :相机中心点在世界坐标系坐标（相机OO点->mRwc->mtwc)
}




 *     函数功能：
 *                 （1）根据地图点的世界坐标得到地图点的相机坐标，验证深度值
 *                 （2）根据地图点的相机坐标得到地图点的像素坐标，检测像素坐标是否在边界内
 *                 （3）计算地图点离相机中心的距离和角度是否符合设定的最大最小值
 *                 （4）如果都符合要求，就给地图点被用在追踪线程的数据赋值，标记该点将来要被投影
 *     函数参数介绍：
 *                pMP：待检测地图点
 *                viewingCosLimit：最大视角    也就是视锥中母线与中心线所呈的夹角
 *     备注：检查 一个地图点是否在该帧数据相机的视锥内
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;          //初始设置为不在视野内

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();      //得到地图点的绝对世界坐标

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;      //从世界坐标系转到相机坐标系
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    // 检测位置深度值
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    // 将地图点的相机坐标映射到像素坐标下
    // 将MapPoint投影到当前帧
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;
    
    // 检测像素坐标是否在边界以外
    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();       // 计算世界坐标系下地图点离相机中心的距离，并判断是否在尺度变化的距离内
    const float minDistance = pMP->GetMinDistanceInvariance();
    
    
    const cv::Mat PO = P-mOw;                                       // 世界坐标系下，相机到3D点P的向量，向量方向由相机指向3D点P 
    const float dist = cv::norm(PO);                                // norm表示计算范数，MapPoint距离相机的距离，进而推断，在图像金字塔中可能的尺度，越远尺度小，越近尺度大
    

    if(dist<minDistance || dist>maxDistance)                        // MapPoint的距离dist不在可测范围内
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();                                  // GetNormal表示获得法线


   // A.dot(B)相当于数学向量运算中的点乘 A·B
    const float viewCos = PO.dot(Pn)/dist;                         // 计算当前视角和平均视角夹角的余弦值，若小于cos（60），即夹角大于60度则退出

    if(viewCos<viewingCosLimit)                                    // 在tracking线程中是0.5限制60度，夹角小于60度才算是在当前帧的视野内
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);      // 根据深度预测MapPoint在帧图像上的尺度，深度大尺度小，深度小尺度大

    // Data used by the tracking
    pMP->mbTrackInView = true;                                     // 表示该地图点可以被观察到
    pMP->mTrackProjX = u;                                          // 该地图点在该帧相机的投影像素x坐标
    pMP->mTrackProjXR = u - mbf*invz;                              // 该3D点投影到双目右侧相机上的横坐标
    pMP->mTrackProjY = v;                                          // 该地图点在该帧相机的投影像素y坐标
    pMP->mnTrackScaleLevel= nPredictedLevel;                       // 该地图点在该帧被观察到时在高斯金字塔中的层数
    pMP->mTrackViewCos = viewCos;                                  // 该地图点在该帧中被观察到时的角度

    return true;
}




 *     函数功能：
 *                 （1）计算该区域所占据的最大最小网格点
 *                 （2）循环所有找到的网格点内的所有特征点
 *                              并剔除所有不符合要求的特征点（包括不在规定金字塔层数和不在范围内的）
 *                              返回满足条件特征点的序号
 *     函数参数介绍：
 *                x，y：区域的中心坐标（ x，y）
 *                r：边长的一半
 *                minLevel：所要提取特征点所在金字塔中的最小层数
 *                maxLevel：所要提取特征点所在金字塔中的最大层数
 *                返回满足条件特征点的序号
 *     备注：找到在 以x,y为中心,边长为2r的方形内且在[minLevel, maxLevel]的特征点（通过网格查找的方式）

//在金字塔图像上{minLevel,maxLevel},minLevel和maxLevel考察特征点是从图像金字塔的哪一层提取出来，寻找Frame中坐标（x，y），半径为r个像素区域内的所有关键点，这里的区域表示正方形
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;        //返回值：记录潜在关键点在去畸变关键点集mvKeysUn中的序号
                                    //size_t 的全称是size_type，一种用来记录大小的数据类型，通常我们用sizeof(XXX)操作得到的结果就是size_t类型
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));       // floor（）：返回一个小于传入参数的最大整数
                                                                                       // mfGridElementWidthInv表示每个像素占用的网格数
                                                                                       // 计算该区域位于的最小网格横坐标
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));      // 计算该区域位于的最大网格横坐标
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));                         // 计算该区域位于的最小网格纵坐标
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);         // ||表示或
    
    
    / 获取半径为r的所有网格内部的关键点
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];             // 第(ix,iy)个网格特征点序号的集合，vCell存储特征点的索引
            if(vCell.empty())
                continue;
           
            //遍历这个（第(ix,iy)）网格中所有特征点
            for(size_t j=0, jend=vCell.size(); j<jend; j++)         // vCell.size：vector的大小不等于capacity容量，用reserve()来增加容器的容量，这时元素的个数并没有改变
                                                                    // 减小容器的大小不会影响容器的容量
                                                                    // 在32位机器中，size_t占用4字节的内存空间，在64位机器中，size_t占用8字节内存空间
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];      // 每个网格内实际保存的是关键点在mvKeysUn中的序号
                if(bCheckLevels)
                {   
                    // kpUn.octave表示这个特征点所在的层
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;
                
                
                //（x,y）区域的中心坐标，剔除区域范围外的点
                if(fabs(distx)<r && fabs(disty)<r)         // &&表示与
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;        //这个才是真正要返回的满足条件特征点的序号
}


 *     函数功能：
 *                 计算特征点所在网格的位置
 *     函数参数介绍：
 *                 kp：特征点
 *                 posX、posY：第(posX,posY)个网格坐标
bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{   
  
    // 计算网格位置
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    //如果特征点的坐标超出边界则返回false
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}



 *     函数功能：
 *                 计算该帧数据描述子对应的BoW向量和Feature向量
 *     函数参数介绍：NULL
void Frame::ComputeBoW()
{
    if(mBowVec.empty())                //判断BoW向量是否为空，防止重复运算
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);     //Mat类型转换到vector类型描述子向量
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}


 *     函数功能：
 *                 首先检测是否需要失真矫正
 *                         如若需要利用opencv的函数cv::undistortPoints()对特征点进行矫正
 *                         并将矫正结果存入mvKeysUn容器中
 *                         此时就有：mvKeys容器存储矫正前的结果
 *                                   mvKeysUn容器存储矫正后的结果
 *     函数参数介绍：NULL
 // 调用opencv的校正函数校正orb提取的特征点，不是对整个图像进行校正
void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)     // 检测是否需要矫正，如果传入的矫正矩阵第一个参数为0那么就说明传入的图像就是已经被矫正过
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    //关键点坐标（未校正）转成opencv mat类型
    cv::Mat mat(N,2,CV_32F);                 // 单通道N*2矩阵内部元素是32float
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);             // 表示将mat修改为双通道，即变为N*1维矩阵，每一维内部是两个通道，一个通道一个值
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);      // 利用opencv的函数对关键点进行失真校正
    mat=mat.reshape(1);             // 重新变为单通道N*2矩阵

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N)             // 将失真校正后的关键点向量填充进mvKeysUn
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}


 *     函数功能：
 *                 函数分为两部分，一部分是当图片需要矫正时：图像的边界为矫正后的图像边界
 *                 第二部分是函数不需要矫正时 图像的边界就是原图像的边界
 *                 此函数的最终结果为：将图形的边界赋值即mnMinX、mnMaxX、mnMinY、mnMaxY
 *     函数参数介绍：
 *                         imLeft：图像彩色图对应的灰度图
 *     备注：计算图像边界（在初始化时调用）
void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)   // 如果图片需要失真矫正
    {
      
        // 矫正前四个边界点：(0,0) (cols,0) (0,rows) (cols,rows)
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;                   
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;           
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;            
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;    

        // Undistort corners
        // 对rgb图进行失真矫正
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));               // 左上和左下横坐标最小的
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));               // 右上和右下横坐标最大的
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));               // 左下和右下纵坐标最小的
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));               // 左上和右上纵坐标最大的

    }
    else                   //如果图片不需要失真矫正，所以边界直接是图像四个角
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
// 双目匹配，特征点匹配
void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;        // TH_HIGH=100、TH_LOW=50   匹配阈值

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;            // 最低层图像的行数

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());      // 初始化vRowIndices，长度和最低层图像的行数相同
                                                                      // 步骤1：建立特征点搜索范围对应表，一个特征点在一个带状区域内搜索匹配特征点
                                                                      // 匹配搜索的时候，不仅仅是在一条横线上搜索，而是在一条横向搜索带上搜索,简而言之，原本每个特征点的纵坐标为1，这里把特征点体积放大，纵坐标占好几行
                                                                      // 例如左目图像某个特征点的纵坐标为20，那么在右侧图像上搜索时是在纵坐标为18到22这条带上搜索，搜索带宽度为正负2，搜索带的宽度和特征点所在金字塔层数有关
                                                                     // 简单来说，如果纵坐标是20，特征点在图像第20行，那么认为18 19 20 21 22行都有这个特征点
                                                                     // vRowIndices[18]、vRowIndices[19]、vRowIndices[20]、vRowIndices[21]、vRowIndices[22]都有这个特征点编号

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);                                //每行最大装200个特征点

    const int Nr = mvKeysRight.size();                              //待匹配的右图特征点个数

    for(int iR=0; iR<Nr; iR++)
    {
        // !!在这个函数中没有对双目进行校正，双目校正是在外层程序中实现的
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        
        // 计算匹配搜索的纵向宽度，尺度越大（层数越高，距离越近），搜索范围越大
        // 如果特征点在金字塔第一层，则搜索范围为:正负2
        // 尺度越大其位置不确定性越高，所以其搜索半径越大
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);                // ceil 求不小于给定实数的最小整数
        const int minr = floor(kpY-r);               // floor 下取整

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);           // 最后vRowIndices每行存可能匹配的特征点的索引
    }

    // Set limits for search
    // 双目基线mb米单位；mbf像素单位
    const float minZ = mb;
    const float minD = 0;                 //最小视差设置为0
    const float maxD = mbf/minZ;          //最大视差

    // For each left keypoint search a match in the right image
    // 左图中每一个特征点寻找右图中的匹配点
    vector<pair<int, int> > vDistIdx;     // pair（最佳匹配的距离，leftIndex）
    vDistIdx.reserve(N);                  // 用reserve提高编程效率


    // 步骤2：对左目相机每个特征点，通过描述子在右目带状搜索区域找到匹配点, 再通过SAD做亚像素匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;        //左目层数
        const float &vL = kpL.pt.y;            //当前特征点纵坐标
        const float &uL = kpL.pt.x;            //当前特征点横坐标

        //可能的匹配点
        const vector<size_t> &vCandidates = vRowIndices[vL];
        
        //可能图像某一行上面没有特征点
        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;      //在右目上最小匹配范围
        const float maxU = uL-minD;      //在右目上最大匹配范围

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;
        
        // 每个特征点描述子占一行，建立一个指针指向iL特征点对应的描述子
        const cv::Mat &dL = mDescriptors.row(iL);
     
        // Compare descriptor to right keypoints
        // 步骤2.1：遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
        // vCandidates相当于基线上所有可能的特征点索引号
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];      //通过索引号找到右目相应特征点
            
            
            //所处金字塔层数差距大于1则否决，仅对相邻尺度的特征点进行匹配
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);      //二进制描述子、相似度距离、汉明距离

                if(dist<bestDist)
                {
                    bestDist = dist;       //保存最小的匹配距离
                    bestIdxR = iR;         //既然是最佳，那就用不着vector，只要记住索引号就行
                }
            }
        }
        
         // 最好的匹配的匹配误差存在bestDist，匹配点位置存在bestIdxR中

        // Subpixel match by correlation
        // 步骤2.2：通过SAD匹配提高像素匹配修正量bestincR
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0 )
            const float uR0 = mvKeysRight[bestIdxR].pt.x;                  //右目金字塔最低层横坐标
            const float scaleFactor = mvInvScaleFactors[kpL.octave];       //当前层octave所对应的尺度因子scaleFactor
            const float scaleduL = round(kpL.pt.x*scaleFactor);            //变换尺度后对应层的坐标
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);               //右目金字塔相应层横坐标

            // sliding window search
            // 滑动窗口的大小11*11 注意该窗口取自resize后的图像
            const int w = 5;
            
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            //opencv矩阵数据类型转换 cv::convetTo
            //void converTo（m，rtype）    m目标矩阵、rtype目标矩阵的类型）
            IL.convertTo(IL,CV_32F);
            
            //ones单位矩阵
            //at（）函数：像素值的读写
            //窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度的影响
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);     // 11、滑动窗口的大小为11*11
            
            // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0-L-w;        //右图像横坐标的下限
            const float endu = scaleduR0+L+w+1;      //右图像横坐标的上限
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)     //用左图像的层数去提取右图像金字塔
                continue;
            
            
            //左图窗口IL,右图滑动窗口IR
            for(int incR=-L; incR<=+L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);        //窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

                float dist = cv::norm(IL,IR,cv::NORM_L1);  //计算矩阵（IL-IR）的1范数
                                                           //计算两个像素块的差值
                if(dist<bestDist)           //找到最小的匹配像素块
                {
                    bestDist =  dist;       //SAD匹配  目前最小匹配偏差
                    bestincR = incR;        //SAD匹配  目前最佳的修正量（像素级）   最佳像素块的平移量
                }

                vDists[L+incR] = dist;      //储存每个像素块的相似度  正常情况下，这里面的数据应该以抛物线形式变化
            }

            if(bestincR==-L || bestincR==L) //整个滑动窗口过程中，SAD最小值不是以抛物线形式出现（窗口滑动到尽头获得最小距离值），SAD匹配失败，同时放弃求该特征点的深度
                continue;

            // Sub-pixel match (Parabola fitting)
            // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
            
            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 最佳右目匹配坐标
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
            
            // 这里是disparity，根据它算出depth
            float disparity = (uL-bestuR);
            
            // 最后判断视差是否在范围内
            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)              //≤0，认为的右特征点居然右移了
                {
                    disparity=0.01;           //认为的减一点，左移一点
                    bestuR = uL-0.01;
                }
                
                // depth 是在这里计算的
                // depth=baseline*fx/disparity
                mvDepth[iL]=mbf/disparity;    //计算深度值
                mvuRight[iL] = bestuR;        //储存在右图像中匹配点的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL));     //该特征点SAD匹配最小匹配偏差
            }
        }
    }


    // 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end());       //将点对按匹配距离排序（ SAD偏差，距离由小到大）
    const float median = vDistIdx[vDistIdx.size()/2].first;       //中值
    const float thDist = 1.5f*1.4f*median;       //计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            //距离过大的匹配点对，认为是误匹配，设置为-1用以标记
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

 *     函数功能：
 *                 将深度图中特征点的深度值存储到mvDepth容器中
 *                 通过深度值和已知的矫正好的特征点x坐标，来计算右眼坐标，并将其存储到mvuRight容器中
 *                 计算右眼坐标基本原理介绍：mbf=f*b   有公式深度z = f*b/d  ,d=Ul-Ur 代表的视差，因此计算右眼的坐标就有，Ur = Ul-mbf/z（这里用d表示了深度）
 *     函数参数介绍：
 *                 imDepth：深度图
 *     备注：将RGBD相机的数据转到双目相机下
// 由RGBD数据得到立体视觉深度信息和右坐标
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);     //初始匹配点
    mvDepth = vector<float>(N,-1);      //初始深度

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];          //关键点未校正，用于从深度图获取深度值
        const cv::KeyPoint &kpU = mvKeysUn[i];       //校正后的关键点

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);      //对应的深度值
 
        if(d>0)         //由可靠的深度信息推倒回右图坐标值
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;    //计算右眼的坐标  mbf=f*b   有公式深度z = f*b/d  ,d=Ul-Ur 代表的视差，因此计算右眼的坐标就有，Ur = Ul-mbf/z（这里用d表示了深度）
        }
    }
}


 *     函数功能：
 *                 首先根据深度值容器得到深度值，然后根据特征点容器得到该特征点的像素坐标，通过相机内参得到相机坐标
 *                 进而将相机坐标转换为世界坐标。
 *     函数参数介绍：
 *                 i：所要转换特征点的id
 *     备注：将特征点映射到3D世界坐标
cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];              //第i个特征点的深度
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;    //像素坐标横坐标
        const float v = mvKeysUn[i].pt.y;    //纵坐标
        const float x = (u-cx)*z*invfx;      //相机坐标系横坐标
        const float y = (v-cy)*z*invfy;      //纵坐标
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;                //投影到世界坐标系
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
