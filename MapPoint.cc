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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;


//构造函数主要是突出地图点和关键帧之间的观测关系，参考关键帧是哪一帧，该地图点被哪些关键帧观测到
//给定点的世界坐标和关键帧构造地图点
//参考帧是关键帧，该地图点将许多关键帧对应，建立关键帧之间的共视关系
MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):  

    //第一关键帧Id（创建该地图点的关键帧的id），第一帧的id（创建该地图点的帧的id），observations中关键帧的数量 ，即可观察到该地图点的关键帧的数量，
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    
    //上一被观察到该地图点的帧的id，用来局部地图构建的变量，用来回环检测的变量
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),mnCorrectedReference(0), mnBAGlobalForKF(0), 
    
    //参考关键帧,跟踪计数器 可以看到该地图点的次数,从内存中擦除该地图点失败
    mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    
    //可代替该地图点的其他地图点,尺度不变距离
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);      // openCV中image.copyTo()有两种形式：
                                // 1、image.copyTo(imageROI)，作用是把image的内容粘贴到imageROI；
                                // 2、image.copyTo(imageROI，mask),作用是把mask和image重叠以后把mask中像素值为0（black）的点对应的image中的点变为透明，而保留其他点
 
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);             // 观察该关键点时的平均观测方向

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    // 地图点创建锁，防止冲突
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}
  
  
//给定点的世界坐标和帧构造地图点
//参考帧是普通帧，该地图点只与普通帧有关
MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):


    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0),
    
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0), mnCorrectedReference(0), mnBAGlobalForKF(0),
    
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1), mbBad(false),
    
    
    mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;                             //世界坐标系下相机到3D点的向量
    mNormalVector = mNormalVector/cv::norm(mNormalVector);      //对方向归一化

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);                           //地图点距离相机中心的距离
    const int level = pFrame->mvKeysUn[idxF].octave;           //得到该关键点（地图点）所在的金字塔的层数
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];       //得到该关键点（地图点）所在的金字塔层的缩放比例
    const int nLevels = pFrame->mnScaleLevels;                //得到该金字塔的总层数
    
    
    //最大距离和最小距离
    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];
    
    
    
    //得到该地图点在该帧下的描述子
    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}


//设置该地图点的世界坐标
void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);               //全局锁，使用与该类的所有对象
    unique_lock<mutex> lock(mMutexPos);                   //位置锁
    Pos.copyTo(mWorldPos);
}



//得到该地图点的世界坐标
cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}


//得到该地图点在该所有关键帧下的平均观测方向
cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}


//得到参考关键帧
KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}


//增加地图点观测：能够观测到同一个地图点的关键帧之间存在的共视关系
//为该地图点的Observation添加关键帧,Observation存储的是可以看到该地图点的所有关键帧的集合   idx是指该地图点在关键帧的索引 
//在添加地图点到全局地图map时调用
void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    
      //如果在观察器中已经存在该关键帧，则不需要继续添加
     //count函数的功能是：统计容器中等于value元素的个数。
    //count(first,last,value); first是容器的首迭代器，last是容器的末迭代器，value是询问的元素。
    if(mObservations.count(pKF))
        return;
        
    //否则将该关键帧和该地图点在该关键帧的索引加入该map容器mObservations
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;           //双目或RGBD(nObs是观测到地图点的相机数，所以肯定是两个相机同时观测到)
    else
        nObs++;            //单目
}



//为该地图点的Observation删除关键帧  Observation存储的是可以看到该地图点的所有关键帧的集合
//删除地图点观测：从当前地图点的mObservation和nObs成员中删除对应的关键帧的
//观测关系，若该关键帧恰好是参考帧，则需要重新指定
//当观测相机数小于等于2时，该地图点需要剔除
//删掉观测关系，和删掉地图点（以及替换地图点），需要分开
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;             //擦除失败标志量
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))            //如果可以找到需要擦除的关键帧
        { 
            int idx = mObservations[pKF];       //得到该关键帧下地图点索引
            if(pKF->mvuRight[idx]>=0)           
                nObs-=2;                        //双目或RGBD
            else
                nObs--;                         //单目 
            
            
            //删除地图点中维护关键帧观测关系
            mObservations.erase(pKF);           //从mObservations中删除该关键帧
            
            
            //如果该keyFrame是参考帧，该Frame被删除后重新指定RefFrame
            if(mpRefKF==pKF)                    //如果参考关键帧是该关键帧，那么将参考关键帧设为mObservations的第一帧
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            // 如果仅有两个或者更少的关键帧那么取消该节点，并标志为失败
            if(nObs<=2)
                bBad=true;
        }
    }
    
    
    //如果擦除失败
    if(bBad)
        SetBadFlag();
}


//获得观测
map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}


//观察到该地图点的关键帧数量
int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}


//告知可以观测到该MapPoint的Frame，该MapPoint已被删除
void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;                 
        obs = mObservations;          //将此时可观察到该地图点的关键帧拷贝给obs,把mObservations转存到obs，obs和mObservations里存的是指针，赋值过程为浅拷贝
        mObservations.clear();        //把mObservations指向的内存释放，obs作为局部变量之后自动删除
    }
    
    //遍历obs  从关键帧中擦除地图点
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);    //告诉可以观测到该MapPoint的KeyFrame，该MapPoint被删了
    }

    mpMap->EraseMapPoint(this);                 //从全局地图中擦除该地图点
}


//得到该地图点的替代地图点
MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}


//将当前地图点（this）,替换成pMp。
//在形成闭环的时候，会更新KeyFrame与MapPoint之间的关系
//关键帧将联系的this替换成pMap
//如果被替换,当前地图点的mbBad会被记为true
void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    
    
    // 和SetBadFlag函数相同
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }
    
    
    // 遍历所有可以看见该地图点的关键帧
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        // 所有能观测到该MapPoint的keyframe都要替换
        KeyFrame* pKF = mit->first;    
        
        
        //如果地图点pMP没在该帧中
        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);//KeyFrame中mit->second索引对应的地图点，用pMP替换掉原来的this，用pMP替换掉本地图点在该帧的位置
                                                        //让KeyFrame用pMP替换掉原来的MapPoint
            pMP->AddObservation(pKF,mit->second);       //并在地图点pMP中增加可观察到该地图点的帧数
                                                        //让MapPoint替换掉对应的KeyFrame
        } 
        else
        {   
          
            // 产生冲突，即pKF中有两个特征点a,b（这两个特征点的描述子是近似相同的），这两个特征点对应两个MapPoint为this,pMP
            // 然而在fuse的过程中pMP的观测更多，需要替换this，因此保留b与pMP的联系，去掉a与this的联系
            pKF->EraseMapPointMatch(mit->second);       //如果地图点pMP在该帧中，则从关键帧中擦除现在的地图点
        }
    }
    
    
    
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();
    
    
    //删掉Map中该地图点
    mpMap->EraseMapPoint(this);
}


//查看该地图点是否是有问题的
bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

//增加该地图点被看到次数n
void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

//增加该地图点被查找次数n
void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}


 * Visible表示：
 * 1. 该MapPoint在某些帧的视野范围内，通过Frame::isInFrustum()函数判断
 * 2. 该MapPoint被这些帧观测到，但并不一定能和这些帧的特征点匹配上
 *    例如：有一个MapPoint（记为M），在某一帧F的视野范围内，
 *    但并不表明该点M可以和F这一帧的某个特征点能匹配上
//返回该地图点的查找率
//visible和found并不是等价的，found的地图点一定是visible的，
//但是visible的地图点可能not found
float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    
    
    //GetFoundRatio低表示该地图点在很多关键帧的视野内，
    //但是没有匹配上很多的特征点
    return static_cast<float>(mnFound)/mnVisible;
}


 *     函数功能：
 *                 1、检查该地图点在各个关键帧中的描述子
 *                 2、如果该关键帧没有问题，那么将该关键中该地图点的描述子存入vDescriptors容器中
 *                 3、计算所有被找到描述子之间的距离，并将其距离存入到Distances数组中
 *                 4、取第i个描述子与其他描述子距离的中值作为其均值参考，然后选出这N个中值中最小的，认为该描述子与其他描述子的距离和最近，认为该描述子可以代表本地图点
 *     函数参数介绍：NULL
 *     备注： 计算最优描述子
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    // 检查该地图点在各个关键帧中的描述子
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        
        //如果地图点标记为不好，直接返回
        if(mbBad)
            return;
            
            
        observations=mObservations;
    }
    
    
    //如果观测为空
    if(observations.empty())
        return;
    
    //保留的描述子数最多和观测数一致
    vDescriptors.reserve(observations.size());
    
    
    

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));      //如果该关键帧没有问题，那么将该关键中该地图点的描述子存入vDescriptors容器中
    }





    if(vDescriptors.empty())
        return;

    // Compute distances between them
    // 计算所有被找到描述子之间的距离，并将其距离存入到Distances数组中
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    // 取出距离中值所对应的描述子
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {   
       
        // 第i个描述子到其它所有所有描述子之间的距离
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];         //取第i个描述子与其他描述子距离的中值作为其均值参考

        if(median<BestMedian)                   //取这些中值中最小的，认为该描述子与其他描述子的距离和最近。
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures); 
        
        // 最好的描述子就是和其它描述子的平均距离最小
        mDescriptor = vDescriptors[BestIdx].clone();        //给最优描述子赋值
    }
}


//得到该地图点的最优描述子，所谓最优就是在观察到该地图点的所有关键帧中描述子距离的中值描述子 详情见函数ComputeDistinctiveDescriptors()
cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

//在关键帧pKF中该地图点的索引
int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

//判断是否为关键帧，如果是，返回值大于0
bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}


//更新地图点平均观测方向（法线）和深度
//由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要更新相应变量
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;           //获得观测到该3d点的所有关键帧
        pRefKF=mpRefKF;                       //观测到该点的参考关键帧
        Pos = mWorldPos.clone();              //3d点在世界坐标系中的位置
    }

    if(observations.empty())
        return;



   
    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        
        //观测点坐标减去关键帧中相机光心的坐标就是观测方向
        //也就是说相机光心指向地图点
        cv::Mat normali = mWorldPos - Owi;
        
        //对所有关键帧对该点的观测方向归一化为单位向量进行求和
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();                          // 参考关键帧相机指向3D点的向量（在世界坐标系下的表示
    const float dist = cv::norm(PC);                                       //地图点到参考关键帧相机中心的距离
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;                             //金字塔层数




    //深度范围：地图点到参考帧（只有一帧）相机中心距离，乘上参考帧中描述子获取金字塔放大尺度
    //得到最大距离mfMaxDistance;最大距离除以整个金字塔最高层的放大尺度得到最小距离mfMinDistance.
    //通常说来，距离较近的地图点，将在金字塔较高的地方提出，
    //距离较远的地图点，在金字塔层数较低的地方提取出（金字塔层数越低，分辨率越高，才能识别出远点）
    //因此，通过地图点的信息（主要对应描述子），我们可以获得该地图点对应的金字塔层级
    //从而预测该地图点在什么范围内能够被观测到
    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;                            // 观测到该点的距离下限
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];  // 观测到该点的距离上限
        mNormalVector = normal/n;                                         // 计算平均观测方向
    }
}


//获得最小距离不变量
float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}


//获得最大距离不变量
float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}


//              ____
// Nearer      /____\     level:n-1 --> dmin
//            /______\                       d/dmin = 1.2^(n-1-m)
//           /________\   level:m   --> d
//          /__________\                     dmax/d = 1.2^m
// Farther /____________\ level:0   --> dmax
//
//           log(dmax/d)
// m = ceil(------------)
//            log(1.2)
//注意金字塔ScaleFactor和距离的关系：当特征点对应ScaleFactor为1.2的意思是：
//图片分辨率下降1.2倍后，可以提取出该特征点(分辨率更高的时候，肯定也可以提出，这里取金字塔中能够提取出该特征点最高层级作为该特征点的层级)
//同时，由当前特征点的距离，推测所在的层级
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;                      //计算当前的缩放比例
    }
    
    // 同时取log线性化
    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);       //得到预测的当前金字塔层数
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
