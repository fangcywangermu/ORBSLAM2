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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;




 * KeyFrame
 * 关键帧，和普通的Frame不一样，但是可以由Frame来构造
 * 许多数据会被三个线程同时访问，所以用锁的地方很普遍




class KeyFrame
{
public:




 *     函数功能：
 *                        构造关键帧
 *     函数参数介绍：
 *                         F：需要加入关键帧的帧
 *                         pMap：地图
 *                         pKFDB：关键帧数据集
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    // 设置位姿
    // 这里的set,get需要用到锁
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();                    //得到从世界坐标到相机坐标的变换矩阵Tcw
    cv::Mat GetPoseInverse();             //得到从相机坐标到世界坐标的变换矩阵Twc
    cv::Mat GetCameraCenter();            //得到相机中心
    cv::Mat GetStereoCenter();            //得到双目相机中心
    cv::Mat GetRotation();                //得到旋转矩阵Rcw
    cv::Mat GetTranslation();             //得到平移矩阵tcw

    // Bag of Words Representation
    void ComputeBoW();                    //计算BoW向量和Feature向量

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);     // 添加与该关键帧相关联的关键帧及其权重
    void EraseConnection(KeyFrame* pKF);                      // 擦除与该关键帧相关联的关键帧pKF
    
    
 * 函数功能：
 * 1. 首先获得该关键帧的所有MapPoint点，统计观测到这些3d点的每个关键帧与其它所有关键帧之间的共视程度
 *    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共3d点的个数。
 * 2. 并且该权重必须大于一个阈值，如果没有超过该阈值的权重，那么就只保留权重最大的边（与其它关键帧的共视程度比较高）
 * 3. 对这些连接按照权重从大到小进行排序，以方便将来的处理
 *    更新完covisibility图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树
    void UpdateConnections();
    
    
    // 将与该关键帧相关联的关键帧序列根据权重进行排序，将排序之后的关键帧和权重存储到mvpOrderedConnectedKeyFrames和mvOrderedWeights中
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();                        //得到相关联的关键帧(关联关键帧是指权重大于15的共视关键帧,也就是有15个以上的共同地图点)
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();               //返回根据权重排序好的关键帧序列
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);   //返回最好的（权重最大的）与该关键帧相关联的关键帧序列
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);          //返回权重大于w的关键帧
    
    
    //得到帧pKF的权重
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);            //给pKF添加孩子  孩子证明本关键点是pKF的父节点，即权重最大的关联关键帧
    void EraseChild(KeyFrame* pKF);          //给pKF删除孩子
    void ChangeParent(KeyFrame* pKF);        //将父节点改变为pKF并给pKF添加子节点为本关键帧   父节点是指与本节点最大关联关键帧
    std::set<KeyFrame*> GetChilds();         //返回本关键帧的所有的孩子，也就是本关键帧为哪些关键帧的最大关联关键帧
    KeyFrame* GetParent();                   //返回父关键帧，父关键帧为本关键帧的最大关联关键帧
    bool hasChild(KeyFrame* pKF);            //检查该关键帧是否有孩子，即该关键帧是否是其他关键帧的最大关联关键帧

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);         //添加回环边，pKF与本关键帧形成回环
    std::set<KeyFrame*> GetLoopEdges();      //返回该关键帧的回环关键帧

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);           //添加地图点pMP及其索引idx
    void EraseMapPointMatch(const size_t &idx);                   //擦除索引为idx的地图点
    void EraseMapPointMatch(MapPoint* pMP);                       //擦除地图点pMP及其在关键帧中的索引
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);  //替换该关键帧相关的地图点及其索引（有点儿bug）
    std::set<MapPoint*> GetMapPoints();                           //得到与该关键帧相关联的地图点的集合，得到在该关键帧中索引为idx的地图点
    std::vector<MapPoint*> GetMapPointMatches();                  //返回与该关键帧相关的地图点
    int TrackedMapPoints(const int &minObs);                      //该关键帧相关的地图点中被观察到的次数大于minObs的地图点个数
    MapPoint* GetMapPoint(const size_t &idx);                     //得到在该关键帧中索引为idx的地图点

    // KeyPoint functions
    // 在以(x,y)为中心,2r为边长的正方形区域内得到特征点的序列
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    
    //将该关键帧的第i个特征点投影到世界坐标系下
    cv::Mat UnprojectStereo(int i);

    // Image
    // 判断坐标为(x,y)的点是否在图片内
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    // 设置该关键帧不可被擦除
    void SetNotErase();
    // 设置该关键帧可擦除
    void SetErase();

    // Set/check bad flag
 *      函数功能：
 *              （1）验证该帧是否可以被擦除
 *              （2）擦除所有本关键帧与关联关键帧之间的关联
 *              （3）擦除所有地图点与本关键帧之间的关联，标志本关键帧已经不能看到这些地图点,这些地图点也不会存在这些关键帧
 *              （4）清空存储与本关键帧关联的其他关键帧变量，清空排序之后的关联关键帧序列
 *              （5）清空子关键帧   并找每个子关键帧的新的父关键帧
 *              （6）在地图中和关键帧数据集中剔除本关键帧
 *      函数参数：NULL
 *      备注：删除该关键帧
    void SetBadFlag();
    
    // 检测该关键帧是否是好的
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    // 计算当前关键帧的场景深度  q=2代表中值 (该关键中所有地图点的中值)
    float ComputeSceneMedianDepth(const int q);

    // 是否a>b
    static bool weightComp( int a, int b){
        return a>b;
    }
    
    // 是否pKF1帧的ID小于pKF2的ID
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    
    // nNextID名字改为nLastID更合适，表示上一个KeyFrame的ID号(改)
    static long unsigned int nNextId;
    
    // 在nNextID(nLastId)的基础上加1就得到了mnID，为当前KeyFrame的ID号
    long unsigned int mnId;
    
    // 每个KeyFrame基本属性是它是一个Frame，KeyFrame初始化的时候需要Frame，
    // mnFrameId记录了该KeyFrame是由哪个Frame初始化的
    const long unsigned int mnFrameId;
    
    // 此帧的时间戳
    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;                 //网格列数
    const int mnGridRows;                 //网格行数
    const float mfGridElementWidthInv;    //网格宽度的逆
    const float mfGridElementHeightInv;   //网格高度的逆

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;  // 回环帧的id(与当前帧产生回环的帧的id)
    int mnLoopWords;                // 当前帧和回环帧的同属同一节点的单词数目
    float mLoopScore;               // 当前帧和回环帧的BOW得分
    long unsigned int mnRelocQuery; // 待重定位帧的id
    int mnRelocWords;               // 当前关键帧与待重定位的帧相同节点数
    float mRelocScore;              // 重定位得分

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    // 相机的相关参数
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    // 此关键帧中关键点的数量
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;       // 矫正前的关键点
    const std::vector<cv::KeyPoint> mvKeysUn;     // 矫正后的关键点
    const std::vector<float> mvuRight; // negative value for monocular points   // 关键点的右眼坐标
    const std::vector<float> mvDepth; // negative value for monocular points    // 关键点的深度
    const cv::Mat mDescriptors;                   // 关键点的描述子

    //BoW
    DBoW2::BowVector mBowVec;           // 该关键点描述子对应的BoW向量
    DBoW2::FeatureVector mFeatVec;      // 图片所有描述子对应的特征向量  两个参数,第一个参数是该特征点对应词典中的节点id  第二个参数是该特征点的特征索引

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;    // 与父关键帧之间的变换矩阵

    // Scale
    const int mnScaleLevels;         // 高斯金字塔的层数
    const float mfScaleFactor;       // 高斯金字塔每层之间的缩放比例
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;  // 尺度因子，scale^n，scale=1.2，n为层数
    const std::vector<float> mvLevelSigma2;   // 尺度因子的平方
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    // 图片的边界
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    
    // 相机内参数矩阵
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;   // 从世界坐标到相机坐标的变换矩阵
    cv::Mat Twc;   // 从相机坐标到世界坐标的变换矩阵
    cv::Mat Ow;    // 相机中心

    cv::Mat Cw; // Stereo middel point. Only for visualization  双目相机的中心坐标,在双目相机中baseline中点坐标

    // MapPoints associated to keypoints
    // 和该关键帧相关的地图点
    std::vector<MapPoint*> mvpMapPoints;

    // BoW   关键帧数据集
    KeyFrameDatabase* mpKeyFrameDB;
    
    // ORB字典
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    // 加速特征提取的网格
    std::vector< std::vector <std::vector<size_t> > > mGrid;
    
    // 与该关键帧链接的关键帧及其权重(权重大于15 的关键帧)   权重为其它关键帧与当前关键帧共视3d点的个数
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights; 
    
    // 与当前关键帧相关的根据权重排序之后关键帧序列
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    
    // 排序之后的权重序列
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    // 第一次建立链接
    bool mbFirstConnection;
    
    // 存储与该关键帧最相关的其他关键帧（权重最大，共视点最多）  在UpdateConnections()中更新
    KeyFrame* mpParent;
    
    
    // std::set是集合，相比vector，进行插入数据这样的操作时会自动排序
    std::set<KeyFrame*> mspChildrens;     // 存储该节点是哪几个关键帧的最大权重关联关键帧
    std::set<KeyFrame*> mspLoopEdges;     // 回环边的关键帧容器，与本帧形成回环的边

    // Bad flags
    bool mbNotErase;       //该关键帧是否可被删除
    bool mbToBeErased;     //该关键帧是否将要被删除
    bool mbBad;            //该关键帧是否已经被删除
 
    float mHalfBaseline; // Only for visualization  //基线的一半

    Map* mpMap;    // 地图
   
    //由于KeyFrame中一部分数据会被多个线程访问修改，因此需要在这些成员中加线程锁，
    //保证同一时间只有一个线程有访问权。涉及线程安全的如下：
    std::mutex mMutexPose;          //关键帧位姿的设置
    std::mutex mMutexConnections;   //关键帧间连接关系的设置
    std::mutex mMutexFeatures;      //关键帧对应地图点的操作，包括地图点计算相连关键帧之间的权重
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
