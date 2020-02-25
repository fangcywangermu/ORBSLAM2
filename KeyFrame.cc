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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;


 *     函数属性：类KeyFrame的构造函数
 *     函数功能：
 *                        构造关键帧
 *     函数参数介绍：
 *                         F：需要加入关键帧的帧
 *                         pMap：地图
 *                         pKFDB：关键帧数据集
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}


//计算BoW向量和Feature向量 在添加关键帧到地图点时调用
void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}


//设置位姿
//Tcw直接求逆计算量比较大？从博客上看到的解释。。。实时系统为了加速也是很拼啊
//对于现在的计算机，4*4的矩阵求逆的计算量应该不是很大的吧
//Ow是对应Twc的twc部分，Ow就对应Tcw^-1中的平移向量-R^T*t
void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    
    // center为相机坐标系（左目）下，立体相机中心的坐标
    // 立体相机中心点坐标与左目相机坐标之间只是在x轴上相差mHalfBaseline
    // 因此可以看出，立体相机中两个摄像头的连线为x轴，正方向为左目相机指向右目相机
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    
    // 世界坐标系下，左目相机中心到立体相机中心的向量，方向由左目相机指向立体相机中心
    Cw = Twc*center;
}

//得到从世界坐标到相机坐标的变换矩阵Tcw
cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

//得到从相机坐标到世界坐标的变换矩阵Twc
cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}



//这里的Ow其实是世界坐标系(第一帧)原点(相机光心)在当前帧参考系(相机坐标系)中的坐标
//等价于twc,运行ORB界面上有个Follow Camera选项，选上后，相机在界面中的位置固定，
//这是就需要这个Ow来计算第一帧的坐标，而不能错误地理解为当前相机在世界参考系下的坐标
//得到相机中心
cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

//得到双目相机中心
cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

// 得到旋转矩阵Rcw
cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

// 得到平移矩阵tcw
cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}



// pKF    关键帧
// weight 权重，该关键帧与pKF共同观测到的3d点数量
// 添加与该关键帧相关联的关键帧及其权重  ，并存储到mConnectedKeyFrameWeights容器中
void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        
        // std::map::count函数只可能返回0或1两种情况
        // count函数返回0，mConnectedKeyFrameWeights中没有pKF，之前没有连接
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
            
                // 之前连接的权重不一样
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}



// 将与该关键帧相关联的关键帧序列根据权重进行排序 ，将排序之后的关键帧和权重存储到mvpOrderedConnectedKeyFrames和mvOrderedWeights中
//每一个关键帧都会维护自己的map，其中记录了与其他关键帧之间的weight
//每次为当前关键帧添加新的连接关键帧后，都需要根据weight对map结构重新排序
void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    
    
    // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的3D点数放在前面，利于排序
    // 由于map结构中没有sort函数，需要将元素取出放入一个pair组成的vector中，排序后放入vPairs
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));
    
    // 进行排序，默认按升序
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;   // keyframe
    list<int> lWs;          // weight
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {   
      
        //所以定义的链表中权重由大到小排列要用push_front
        lKFs.push_front(vPairs[i].second);   //构造关键帧链表
        lWs.push_front(vPairs[i].first);     //构造权重链表
    }
    
    // 更新排序好的连接关键帧及其对应的权重
    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}




//得到与该关键帧连接的关键帧(关联关键帧是指权重大于15的共视关键帧,也就是有15个以上的共同地图点)
set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;   // 定义成set类型，无顺序
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

// 得到与该关键帧连接的关键帧(已按权值排序)
vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}



//获取前N个与当前关键帧共视关系最好的关键帧，如果小于N个，则返回全部当前相邻关键帧
vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}


// 得到与该关键帧连接的权重大于等于w的关键帧
vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();
    
    // 存储权重大于w的权重序列
    // 从mvOrderedWeights找出第一个大于w的那个迭代器
    // 这里应该使用lower_bound，因为lower_bound是返回小于等于，而upper_bound只能返回第一个大于的
    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}


// 返回指定帧和当前帧之间的权重
int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

//当前帧对应的地图点的指针均存放在mvpMapPoints(mvp代表：member、vector、pointer)向量中
//通过对mvpMapPoints操作封装，可以得到以下的操作：

// 添加地图点pMP及其索引idx  在加入关键帧时调用
  pMP MapPoint
  idx MapPoint在KeyFrame中的索引
void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}


// 擦除索引为idx的地图点
// 删除对应关键帧中的地图点以及在地图中该地图点的内存
void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

//删除在关键帧中对应的地图点的index
void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

//替换对应idx的地图点
void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

//得到当前关键帧中的地图点
set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {   
        //判断vector中对应idx的地图点是否存在
        if(!mvpMapPoints[i])
            continue;
            
        //存在，取出，判断是否为坏点，不是，则存入set中
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}



//返回高质量MapPoints(感觉也就是设定了一个阈值)的数量
//这个阈值设定是该地图点最少被多少相机观测到
关键帧中，大于等于minObs的MapPoints的数量
 * minObs就是一个阈值，大于minObs就表示该MapPoint是一个高质量的MapPoint
 * 一个高质量的MapPoint会被多个KeyFrame观测到，
 * minObs 最小观测
int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {   
                    //地图点被观察次数大于minObs
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}


//返回与该关键帧相关的地图点
//获取该关键帧的MapPoints
vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}


//得到在该关键帧中索引为idx的地图点
MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}


 * 函数功能：
 * 1. 首先获得该关键帧的所有MapPoint点，统计观测到这些3d点的每个关键帧与其它所有关键帧之间的共视程度
 *    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共3d点的个数。
 * 2. 并且该权重必须大于一个阈值，如果没有超过该阈值的权重，那么就只保留权重最大的边（与其它关键帧的共视程度比较高）
 * 3. 对这些连接按照权重从大到小进行排序，以方便将来的处理
 *    更新完covisibility图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树
 * 4. 更新关联关键帧及权重
 * 5. 更新父关键帧为关联关键帧权重最大帧
 // 在没有执行这个函数前，关键帧只和MapPoints之间有连接关系，这个函数可以更新关键帧之间的连接关系
void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;   // 关键帧-权重，权重为其它关键帧与当前关键帧共视3d点的个数

    vector<MapPoint*> vpMP;         // 存储该关键帧的地图点

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    //通过3D点间接统计可以观测到这些3D点的所有关键帧之间的共视程度
    //即统计每一个关键帧都有多少关键帧与它存在共视关系，统计结果放在KFcounter
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;
        
        //得到可以看到pMP地图点的所有关键帧以及地图点在这些关键帧中的索引
        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {   
            // 除去自身，自己与自己不算共视
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;              //存储权重的最大值
    KeyFrame* pKFmax=NULL;   //权重最大值所对应的关键帧
    int th = 15;
    
    //vPairs记录与其它关键帧共视帧数大于th的关键帧 权重-关键帧
    //pair<int,KeyFrame*>将关键帧的权重写在前面，关键帧写在后面方便后面排序
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;   // 找到对应权重最大的关键帧（共视程度最高的关键帧）
        } 
        
        //如果权重大于阈值
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));  //将权重大于th的关键帧存入vPairs中
            
            // 更新KFcounter中该关键帧的mConnectedKeyFrameWeights
            // 更新其它KeyFrame的mConnectedKeyFrameWeights，更新其它关键帧与当前帧的连接权重
            (mit->first)->AddConnection(this,mit->second);        
        }
    }
    
    // 如果没有超过阈值的权重，则对权重最大的关键帧建立连接
    if(vPairs.empty())
    {
        // 如果每个关键帧与它共视的关键帧的个数都少于th，
        // 那就只更新与其它关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
        // 这是对之前th这个阈值可能过高的一个补丁
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    // vPairs里存的都是相互共视程度比较高的关键帧和共视权重，由大到小
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;  //排序后的关联关键帧
    list<int> lWs;         //排序后的权重
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;                                    //将关联关键帧及其权重存储
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());//存储排序后的关联关键帧
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());                   //存储排序后的权重
        
        // 更新生成树的连接
        if(mbFirstConnection && mnId!=0)
        {   
            // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
            mpParent = mvpOrderedConnectedKeyFrames.front();
            // 建立双向连接关系
            mpParent->AddChild(this);
            // 证明不是第一次建立链接
            mbFirstConnection = false;
        }

    }
}


//给pKF添加孩子  孩子证明本关键点是pKF的父节点，即权重最大的关联关键帧  在为当前关键帧添加父关键帧的同时为父关键帧添加子关键帧
void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}


//给pKF删除孩子  孩子证明本关键点是pKF的父节点，即权重最大的关联关键帧
void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

// 将父节点改变为pKF并给pKF添加子节点为本关键帧   父节点是指与本节点最大关联关键帧
void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}


// 返回本关键帧的所有的孩子，也就是本关键帧为哪些关键帧的最大关联关键帧
set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

// 返回父关键帧 父关键帧为本关键帧的最大关联关键帧
KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}


// 检查该关键帧是否有孩子，即该关键帧是否是其他关键帧的最大关联关键帧
bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}


//添加回环边  pKF与本关键帧形成回环
void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;       //与其他关键帧形成回环后就不会被擦除
    mspLoopEdges.insert(pKF);
}


//返回该关键帧的回环关键帧
set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}


// 设置该关键帧不可被擦除
void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}


// 设置该关键帧可擦除
void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        //删除关键帧是，首先要检测mspLoopEdges是否是空的，因为如果当前帧维护了一个回环，删了这个关键帧回环没有了
        //所以通常情况系下是空的，就可以吧mbNotErase设置成false，也就是说当前帧是可删除的
        //如果没有其他关键帧与该关键帧形成回环，则将该关键帧设置为可擦除
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}


 *      函数功能：
 *              （1）验证该帧是否可以被擦除
 *              （2）擦除所有本关键帧与关联关键帧之间的关联
 *              （3）擦除所有地图点与本关键帧之间的关联，标志本关键帧已经不能看到这些地图点,这些地图点也不会存在这些关键帧
 *              （4）清空存储与本关键帧关联的其他关键帧变量，清空排序之后的关联关键帧序列
 *              （5）清空子关键帧   并找每个子关键帧的新的父关键帧
 *              （6）在地图中和关键帧数据集中剔除本关键帧
 *      函数参数：NULL
 *      备注：删除该关键帧
//设置坏的标志
//KeyFrame中比较难理解的是setFlag()函数，真实删除当前关键帧之前，需要处理好父亲和儿子关键帧关系，不然会造成整个关键帧维护图的断裂
//或者混乱，不能够为后端提供较好的初值
//理解起来就是当前帧(父亲)挂了，儿子节点需要找新的父亲节点，在候选的父亲节点中，当前帧的父亲节点（父亲的父亲）肯定在候选节点中
//说先设置成坏帧，且不是回环帧，则可以删除当前帧，如果是回环帧，那么则无法删除该帧
void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
            
            // mbNotErase表示不应该擦除该KeyFrame，于是把mbToBeErased置为true，表示已经擦除了，其实没有擦除
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }


    //擦除所有本关键帧与关联关键帧之间的关联
    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);
    
    //擦除所有地图点与本关键帧之间的关联，标志本关键帧已经不能看到这些地图点
    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();    // 与该关键帧链接的关键帧及其权重(权重大于15 的关键帧)清空
        mvpOrderedConnectedKeyFrames.clear(); // 与当前关键帧相关的根据权重排序之后关键帧序列清空

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        
        //将本关键帧的父关键帧插入sParentCandidates变量
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        // 如果当前这个关键帧有自己的孩子关键帧，告诉这些子关键帧，它们的父关键帧不行了，赶紧找新的父关键帧
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;
            
            // 主要解决的问题是：如果将本关键帧消除的话，以本关键帧为父关键帧（共视化程度最高）的子关键帧中没有了父关键帧，需要重新给这些子关键帧找寻父关键帧
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)  // 遍历所有的子关键帧
            {
                KeyFrame* pKF = *sit;   //pKF存储的是子关键帧
                if(pKF->isBad())        //如果此子关键帧是坏的，则继续下一个子关键帧的检测
                    continue;

                // Check if a parent candidate is connected to the keyframe
                // 子关键帧遍历每一个与它相连的关键帧（共视关键帧）
                // 如果该帧的子节点和父节点（祖孙节点）之间存在连接关系（共视）
                    // 举例：B-->A（B的父节点是A） C-->B（C的父节点是B） D--C（D与C相连） E--C（E与C相连） F--C（F与C相连） D-->A（D的父节点是A） E-->A（E的父节点是A）
                    //      现在B挂了，于是C在与自己相连的D、E、F节点中找到父节点指向A的D
                    //      此过程就是为了找到可以替换B的那个节点。
                    // 上面例子中，B为当前要设置为SetBadFlag的关键帧
                    //           A为spcit，也即sParentCandidates
                    //           C为pKF,pC，也即mspChildrens中的一个
                    //           D、E、F为vpConnected中的变量，由于C与D间的权重 比 C与E间的权重大，因此D为pP
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                
                //遍历每一个子关键帧相关联的其他关键帧
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {   
                  
                    //如果共视帧有候选父亲，则将该共视帧的候选父亲 更新为该候选父亲
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)    //如果该帧的子节点与父节点之间存在联系
                        { 
                            int w = pKF->GetWeight(vpConnected[i]);   //找出与子关键帧关联关键帧中最大权重的关键帧，认为该关键帧为子关键帧的父关键帧
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }
            
            //如果找到，则修改该儿子节点的父节点为所有共视帧父亲节点
            if(bContinue)
            {
                pC->ChangeParent(pP);         //认别的帧做父亲节点了
                sParentCandidates.insert(pC); // 因为子节点找到了新的父节点并更新了父节点，那么该子节点升级，作为其它子节点的备选父节点
                mspChildrens.erase(pC);       //该子节点也就和当前帧没有关系了
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        //如果当前子节点没有任何候选父节点，分配初始的父节点
        //不存在共视关系可能是因为(速度太快，旋转太急，匹配跟丢)
        
        //直接分配当前帧的父节点(也就是爷爷辈的节点来管)
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);   //那么将本关键帧的父节点作为自己的父节点（父节点的父节点作为自己的父节点）
            }
 
        mpParent->EraseChild(this);               //在父节点中擦除该子节点（本关键帧）
        mTcp = Tcw*mpParent->GetPoseInverse();    //与父关键帧之间的变换矩阵
        mbBad = true;                             //说明该关键帧为坏的，已经被剔除
    }  


    mpMap->EraseKeyFrame(this);                   //从地图中擦除该关键帧
    mpKeyFrameDB->erase(this);                    //从关键帧数据集中剔除本关键帧
}


// 检测该关键帧是否是好的
bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

// 擦除与该关键帧相关联的关键帧pKF
void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        
        // 与该关键帧链接的关键帧及其权重(权重大于15 的关键帧)序列关键帧中是否有pKF关键帧
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }
    
    // 如果擦除成功则更新关联关键帧序列
    if(bUpdate)
        UpdateBestCovisibles();
}

// 在以(x,y)为中心,2r为边长的正方形区域内得到特征点的序列
vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);
    
    // floor向下取整，mfGridElementWidthInv为每个像素占多少个格子
    // 最小网格x坐标
    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;
    
    // ceil向上取整
    // 最大网格x坐标
    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;
    
    // 最小网格y坐标
    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;
    
    // 最大网格y坐标
    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;
    
    // 遍历区域内的所有网格
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];     //得到该网格内的特征点序列
            for(size_t j=0, jend=vCell.size(); j<jend; j++) //遍历每个网格内特征点序列
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)          //除去在网格内却不在区域内的特征点，并将剩下的特征点存入到vIndices中
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}


//判断坐标为(x,y)的点是否在图片内
bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

//将该关键帧的第i个特征点投影到世界坐标系下
cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {   
        
        // 由2维图像反投影到相机坐标系
        // mvDepth是在ComputeStereoMatches函数中求取的
        // mvDepth对应的校正前的特征点，因此这里对校正前特征点反投影
        // 可在Frame::UnprojectStereo中却是对校正后的特征点mvKeysUn反投影
        // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);  //得到相机坐标系下的该特征点的坐标

        unique_lock<mutex> lock(mMutexPose);
        
        // 由相机坐标系转换到世界坐标系
        // Twc为相机坐标系到世界坐标系的变换矩阵
        // Twc.rosRange(0,3).colRange(0,3)取Twc矩阵的前3行与前3列
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);  //得到世界坐标系下该特征点的坐标  R*x3Dc+t
    }
    else
        return cv::Mat();
}


//计算当前关键帧的场景深度  q=2代表中值(该关键中所有地图点的中值)
float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);  //R的最后一行（第三行）
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);           //t的最后一个元素（第三个元素）
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();  //得到地图点pMP的世界坐标
            float z = Rcw2.dot(x3Dw)+zcw;       //(R*x3Dw+t)的第三行，即z。      Rcw2.dot(x3Dw)代表Rcw2和x3Dw的数量积
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());        //对该关键帧的所有地图点进行排序，并若q=2,返回中值

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
