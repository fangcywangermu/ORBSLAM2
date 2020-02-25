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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;    //直方图宽度



ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}


    // 在帧的关键点和映射地图点之间寻找匹配，返回匹配点的数量
    // 通过投影的方式进行匹配,将地图点投影到对应帧的相应区域中,在对应区域中搜索匹配
    // 在帧F中搜索vpMapPoints地图点的匹配点
   筛选方法:
   在投影区域内遍历所有的特征点,计算描述子距离第一小和描述子距离第二小  当第一小明显小于第二小时证明该特征点具有最优性
   从而确定匹配
    // th:  决定投影后搜索半径大大小，其中的阈值th一般根据单目还是双目，或者最近有没有进行重定位来确定，代表在投影点的这个平面阈值范围内寻找匹配特征点。
    //跟踪局部地图(在局部地图中寻找与当前帧特征点匹配点)，因为在TrackReferenceKeyFrame和TrackWithMotionModel中
    //仅仅是两帧之间跟踪，会跟丢地图点，这里通过跟踪局部地图，在当前帧中恢复出一些当前帧的一些地图点
int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;   //匹配点数量

    const bool bFactor = th!=1.0;  //是否需要尺度
    
    //给每一个地图点在帧F里寻找匹配点
    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)  // 遍历所有地图点
    {
        MapPoint* pMP = vpMapPoints[iMP];
        
        // 该地图点是否在追踪线程中被追踪到，如果没有则舍弃该地图点
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;
        
        // 该特征点被追踪到时在金字塔中的层数
        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        // 搜索窗口的大小取决于视角, 若当前视角和平均视角夹角接近0度时, r取一个较小的值
        float r = RadiusByViewingCos(pMP->mTrackViewCos);
        
        
        // 如果需要进行更粗糙的搜索，则增大范围
        if(bFactor)
            r*=th;
        
        // 得到地图点所在投影区域的所有特征点索引
        const vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;
        
        // 地图点的描述子
        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;




        // Get best and second matches with near keypoints
        // 在关键点附近查找最好和第二匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)    // 遍历区域内所有的特征点索引
        {
            const size_t idx = *vit;
            
            // 如果Frame中的该特征点已经有对应的MapPoint了,则退出该次循环
            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            if(F.mvuRight[idx]>0)
            {
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                if(er>r*F.mvScaleFactors[nPredictedLevel])
                    continue;
            }
            
            
            // 当前特征点的描述子
            const cv::Mat &d = F.mDescriptors.row(idx);
            // 两描述子之间的距离
            const int dist = DescriptorDistance(MPdescriptor,d);
            
            
            // 根据描述子寻找描述子距离最小和次小的特征点
            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }
        
        
        
        
        

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {   
            //此条件证明该特征点只与第一距离的描述子距离近,而与其他描述子距离远  
            //第一最优描述子距离/第二描述子距离 > mfNNratio  
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;  // 为Frame中的特征点增加对应的MapPoint
            nmatches++;
        }
    }

    return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}



// 该点是否满足对极几何 (映射点到极线的距离)
bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const KeyFrame* pKF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    // 求出kp1在pKF2上对应的极线
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);
    
    
    // 计算kp2特征点到极线的距离：
    // 极线l：ax + by + c = 0
    // (u,v)到l的距离为： |au+bv+c| / sqrt(a^2+b^2)
    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;
    
    // 计算特征点到极线的距离
    const float dsqr = num*num/den;
    
    
    // 尺度越大，范围应该越大。
    // 金字塔最底层一个像素就占一个像素，在倒数第二层，一个像素等于最底层1.2个像素（假设金字塔尺度为1.2）
    // 卡方分布中自由度为1 概率为0.05  确定误差的取值为小于3.84
    return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave];
}


// 通过词包，对关键帧的特征点进行跟踪
// 通过bow对pKF和F中的特征点进行快速匹配（不属于同一node的特征点直接跳过匹配）
// 对属于同一node的特征点通过描述子距离进行匹配
// 根据匹配，用pKF中特征点对应的MapPoint更新F中特征点对应的MapPoints
// 每个特征点都对应一个MapPoint，因此pKF中每个特征点的MapPoint也就是F中对应点的MapPoint
               pKF               KeyFrame
               F                 Current Frame
               vpMapPointMatches F中MapPoints对应的匹配，NULL表示未匹配
               return            成功匹配的数量
int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{   
  
    //关键帧特征点对应的地图点
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
    
    
    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));
    
     //关键帧的特征向量
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;




    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    // 将属于同一节点(特定层)的ORB特征进行匹配
    // 关键帧特征向量的迭代器    first代表该特征向量的节点id  second代表该特征点在图像所有特征点集合中的索引
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    while(KFit != KFend && Fit != Fend)
    {   
        //步骤1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
        if(KFit->first == Fit->first)    //如果当前帧和关键帧的当前特征向量属于同一节点
        {
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;
            
            // 步骤2：遍历KF中属于该node的特征点
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)    //遍历关键帧所有特征点在图像所有特征点集合中的索引
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                MapPoint* pMP = vpMapPointsKF[realIdxKF];           // 取出KF中该特征对应的MapPoint

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;                

                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);// 取出KF中该特征对应的描述子

                int bestDist1=256;  //第一最优距离
                int bestIdxF =-1 ;
                int bestDist2=256;  //第二最优距离
                
                
                // 步骤3：遍历F中属于该node的特征点，找到了最佳匹配点
                for(size_t iF=0; iF<vIndicesF.size(); iF++)          // 遍历当前帧所有特征点在图像所有特征点集合中的索引
                {
                    const unsigned int realIdxF = vIndicesF[iF]; 
                    
                    // 表明这个点已经被匹配过了，不再匹配，加快速度
                    if(vpMapPointMatches[realIdxF])
                        continue;

                    const cv::Mat &dF = F.mDescriptors.row(realIdxF);// 取出F中该特征对应的描述子

                    const int dist =  DescriptorDistance(dKF,dF);    // 当前帧特征点描述子和关键帧特征点描述子的距离
                    
                    // 如果两描述子距离小于最优距离  则更新第一最优距离和第二最优距离
                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    // bestDist1 < dist < bestDist2，更新bestDist2
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }
                
                // 步骤4：根据阈值 和 角度投票剔除误匹配
                if(bestDist1<=TH_LOW)   //第一最优距离小于阈值
                {   
                    // 最佳匹配比次佳匹配明显要好，那么最佳匹配才真正靠谱
                    // 第一最优距离/第二最优距离 < 距离比阈值     此条件证明该特征点只与第一距离的描述子距离近,而与其他描述子距离远
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {   
                        // 步骤5：更新特征点的MapPoint
                        //将匹配到的地图点添加到vpMapPointMatches容器中
                        vpMapPointMatches[bestIdxF]=pMP;
                        
                        
                        // 提取此时匹配点的矫正关键点
                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];
                        
                        // 是否需要判别旋转
                        if(mbCheckOrientation)
                        {   
                            // angle：每个特征点在提取描述子时的旋转主方向角度，如果图像旋转了，这个角度将发生改变
                            // 所有的特征点的角度变化应该是一致的，通过直方图统计得到最准确的角度变化值
                            float rot = kp.angle-F.mvKeys[bestIdxF].angle;      // 该特征点的角度变化值
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);     // 将rot分配到bin组
                            if(bin==HISTO_LENGTH)            // 构建旋转角度差直方图
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);    // 将最优匹配点在当前帧的索引添加进rotHist[]容器
                        }
                        nmatches++;                         //匹配点数量增加1
                    }
                }

            }

            KFit++;    //关键帧特征点迭代指针后移
            Fit++;     //当前帧特征点指针后移
        }
        
        
        //如果关键帧的当前特征向量节点id小于当前帧的特征点节点id
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }
  
    //根据方向剔除误匹配的点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
        
        // 计算rotHist中最大的三个的index
        // 选出图像特征点的三个主方向
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);



        
        for(int i=0; i<HISTO_LENGTH; i++)
        {   
            // 如果特征点的旋转角度变化量属于这三个组，则保留
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
                
            // 将除了ind1 ind2 ind3以外的匹配点去掉
            // 如果匹配点的图像方向不具有区分性 则将该匹配点删除
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}





// 根据Sim3变换，将每个vpPoints投影到pKF上，并根据尺度确定一个搜索区域
// 根据该MapPoint的描述子与该区域内的特征点进行匹配，如果匹配误差小于TH_LOW即匹配成功，更新vpMatched
 * 		通过相似矩阵Scw映射计算匹配点   将地图点向关键帧进行投影
 * 		pKF(in): 待检测关键帧
 * 		Scw(in): 当前关键帧的相似矩阵位姿
 * 		vpPoints(in):待匹配的地图点
 * 		vpMatched(out):得到的匹配点
 * 		th:阈值
 *   	主要思路:将地图点根据相似矩阵映射到当前关键帧中,在映射区域中查找地图点的匹配
 // 在当前关键帧中匹配所有关键帧中的地图点
int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    // 将相似矩阵Scw分解为旋转平移和尺度的形式
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));  // 计算得到尺度s
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;            // pKF坐标系下，世界坐标系到pKF的位移，方向由世界坐标系指向pKF
    cv::Mat Ow = -Rcw.t()*tcw;                             // 世界坐标系下，pKF到世界坐标系的位移（世界坐标系原点相对pKF的位置），方向由pKF指向世界坐标系



    // Set of MapPoints already found in the KeyFrame
    // 使用set类型，并去除没有匹配的点，用于快速检索某个MapPoint是否有匹配
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches=0;





    // For each Candidate MapPoint Project and Match
    // 遍历所有的待匹配MapPoints
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];


        // Discard Bad MapPoints and already found
        // 丢弃坏的MapPoints和已经匹配上的MapPoints
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;


        // Get 3D Coords.
        // 得到当前地图点的世界坐标系坐标
        cv::Mat p3Dw = pMP->GetWorldPos();


        // Transform into Camera Coords.
        // 当前地图点在当前关键帧的相机坐标系下的坐标
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0)
            continue;




        // Project into Image
        // 当前地图点在当前关键帧下的映射像素坐标
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;



        // Depth must be inside the scale invariance region of the point
        // 判断距离是否在尺度协方差范围内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist,pKF);


        // Search in a radius
        // 根据尺度确定搜索半径
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        
        // 遍历搜索区域内所有特征点，与该MapPoint的描述子进行匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const int &kpLevel= pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }
        
        
        // 该MapPoint与bestIdx对应的特征点匹配成功
        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}









 * 		为地图初始化寻找匹配点
 * 		F1:待匹配帧1
 * 		F2:待匹配帧2
 * 		vbPrevMatched:F1的特征点位置坐标
 * 		vnMatches12:  匹配帧2在匹配帧1的匹配索引,下标为匹配帧1的关键点索引,值为匹配帧2的关键点索引
 * 		windowSize:   在F2帧中搜索特征点的窗口大小
 * 		步骤:
 * 			1. 构建搜索区域  假定匹配帧1和匹配帧2之间没有太大的位移,所以在匹配帧2的帧1特征点位置坐标处寻找对应的匹配点
 * 			2. 通过计算描述子距离来寻找最优匹配距离以及最优匹配距离对应帧2的特征点索引
 * 			3. 如果需要检查旋转,则构建旋转角度直方图来筛选匹配点
int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{   
    //匹配点数量
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);  // 匹配点的最优距离
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);            // 匹配帧1在匹配帧2的匹配索引,下标为匹配帧2的关键点索引,值为匹配帧1的关键点索引



    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        //F1帧i1索引特征点
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        //得到F2帧对应区域的特征点索引
        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);  // F1帧i1索引特征点的描述子

        int bestDist = INT_MAX;                // 最优特征距离
        int bestDist2 = INT_MAX;               // 第二优特征描述子距离
        int bestIdx2 = -1;                     // 最优匹配点在帧2中的特征索引


        // 遍历所有F2中对应区域的特征点 寻找最优特征描述子距离和第二优特征描述子距离
        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }
        
        
        // 如果最优描述子距离小于TH_LOW阈值  并且bestDist/bestDist2<mfNNratio阈值,证明  最优距离远小于第二优距离
        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                
                // 将最优距离对应的匹配点在帧1和帧2中的索引存储
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;
                
                // 如果需要旋转则构建旋转角度的直方图
                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}







// 通过词包，对关键帧的特征点进行跟踪，该函数用于闭环检测时两个关键帧间的特征点匹配
// 对属于同一node的特征点通过描述子距离进行匹配，根据匹配，更新vpMatches12
pKF1               KeyFrame1
pKF2               KeyFrame2
vpMatches12        pKF2中与pKF1匹配的MapPoint，null表示没有匹配
return             成功匹配的数量
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;             //关键帧1的关键点
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;            //关键帧1的特征向量   迭代器的first是bow节点,对应的second是bow节点下该图像中的特征点
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches(); //关键帧1的地图点
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;                  //关键帧1的描述子

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;             //关键帧2的关键点
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;            //关键帧2的特征向量   迭代器的first是bow节点,对应的second是bow节点下该图像中的特征点
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches(); //关键帧2的地图点
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;                  //关键帧2的描述子

    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL)); //匹配到的地图点
    vector<bool> vbMatched2(vpMapPoints2.size(),false);                                //关键帧2中的地图点是否是匹配点的关键帧标志

    vector<int> rotHist[HISTO_LENGTH];        //旋转角度差的直方图
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();   //关键帧1的特征向量迭代器
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();   //关键帧2的特征向量迭代器
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();
    
    
    //遍历所有的特征点向量
    while(f1it != f1end && f2it != f2end)
    {   
      
        //步骤1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
        if(f1it->first == f2it->first)
        {    
            // 步骤2：遍历关键帧1的所有特征点
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];
                
                //特征点1对应的地图点坐标
                MapPoint* pMP1 = vpMapPoints1[idx1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;
                
                //特征点1对应的描述子
                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1=256;   // 第一最优描述子距离
                int bestIdx2 =-1 ;
                int bestDist2=256;   // 第二最优描述子距离
               
                
                // 步骤3：遍历关键帧2属于该node的特征点，找到了最佳匹配点
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    const cv::Mat &d2 = Descriptors2.row(idx2);

                    int dist = DescriptorDistance(d1,d2);  // 计算两描述子的距离
                    
                    
                    // 查找第一最优描述子距离和第二最优描述子距离
                    if(dist<bestDist1)   
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2=idx2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }
                
                
                // 步骤4：根据阈值 和 角度投票剔除误匹配
                if(bestDist1<TH_LOW)
                {   
                  
                    // 第一最优描述子距离/第二最优描述子距离<mfNNratio
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1]=vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2]=true;
                        
                        //如果需要检测旋转
                        if(mbCheckOrientation)
                        {
                            float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        
        //如果特征点1的节点id小于节点2的节点id
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}





// 利用三角化，在两个关键帧之间恢复出一些地图点
// 利用基本矩阵F12，在两个关键帧之间未匹配的特征点中产生新的3d点
pKF1          关键帧1
pKF2          关键帧2
F12           基础矩阵
vMatchedPairs 存储匹配特征点对，特征点用其在关键帧中的索引表示
bOnlyStereo   在双目和rgbd情况下，要求特征点在右图存在匹配
return        成功匹配的数量
int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                       vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
{    
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;  //取KF1的特征向量
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;  //取KF2的特征向量

    //Compute epipole in second image
    //计算KF1的相机中心在KF2图像平面的坐标，即极点坐标
    cv::Mat Cw = pKF1->GetCameraCenter();     // twc1
    cv::Mat R2w = pKF2->GetRotation();        // Rc2w
    cv::Mat t2w = pKF2->GetTranslation();     // tc2w
    cv::Mat C2 = R2w*Cw+t2w;                  // tc2c1 KF1的相机中心在KF2坐标系的表示
    const float invz = 1.0f/C2.at<float>(2);
    
    // 步骤0：得到KF1的相机光心在KF2中的坐标（极点坐标）
    const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;
    const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;



    // Find matches between not tracked keypoints   在没有追踪到的关键点之间寻找匹配
    // Matching speed-up by ORB Vocabulary          通过orb词典进行快速匹配
    // Compare only ORB that share the same node    仅仅比较在同一节点上的orb特征点

    int nmatches=0;
    vector<bool> vbMatched2(pKF2->N,false);
    vector<int> vMatches12(pKF1->N,-1);

    vector<int> rotHist[HISTO_LENGTH];  //直方图
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;
    
    
    // 将属于同一节点(特定层)的ORB特征进行匹配
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();   //关键帧1的特征向量迭代器
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();   //关键帧2的特征向量迭代器
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();
   
   
    // 将左图像的每个特征点与右图像同一node节点的所有特征点
    // 依次检测，判断是否满足对极几何约束，满足约束就是匹配的特征点
    while(f1it!=f1end && f2it!=f2end)   //遍历所有的特征点向量
    {
      
        //如果两个特征点在同一个BOW词典node节点上 则寻找匹配
        if(f1it->first == f2it->first)  
        {   
            // 遍历该node节点下(f1it->first)的所有特征点
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {   
                // 获取pKF1中属于该node节点的所有特征点索引
                const size_t idx1 = f1it->second[i1];
                
                
                // 通过特征点索引idx1在pKF1中取出对应的MapPoint
                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
                
                
                
                // If there is already a MapPoint skip
                // 由于寻找的是未匹配的特征点，所以pMP1应该为NULL
                if(pMP1)
                    continue;
                
                
                // 如果mvuRight中的值大于0，表示是双目，且该特征点有深度值
                const bool bStereo1 = pKF1->mvuRight[idx1]>=0;

                if(bOnlyStereo)
                    if(!bStereo1)
                        continue;
                
                
                //该关键帧1中该特征向量对应的关键点
                const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];
                
                
                //该关键帧1中对应于该特征向量的描述子向量
                const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);
                
                int bestDist = TH_LOW;
                int bestIdx2 = -1;
                
                
                //循环图片2中所有特征点的特征索引 找到最小描述子距离的情况下满足对极几何约束条件的匹配点
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {   
                    // 获取pKF2中属于该node节点的所有特征点索引
                    size_t idx2 = f2it->second[i2];
                    
                    
                    // 通过特征点索引idx2在pKF2中取出对应的MapPoint
                    MapPoint* pMP2 = pKF2->GetMapPoint(idx2);
                    
                    
                    
                    // If we have already matched or there is a MapPoint skip
                    // 如果pKF2当前特征点索引idx2已经被匹配过或者对应的3d点非空，那么这个索引idx2就不能被考虑
                    // vbMatched2：关键帧2中的地图点是否是匹配点的关键帧标志
                    if(vbMatched2[idx2] || pMP2)
                        continue;
                    
                    
                    //该特征点对应的右眼坐标
                    const bool bStereo2 = pKF2->mvuRight[idx2]>=0;
                    
                    
                    //如果是双目，右眼坐标不能为空
                    if(bOnlyStereo)
                        if(!bStereo2)
                            continue;
                    
                    
                    //该关键帧2中对应于该特征向量的描述子向量
                    const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);
                    
                    
                    //计算idx1与idx2在两个关键帧中对应特征点的描述子距离
                    const int dist = DescriptorDistance(d1,d2);
                    
                    
                    //如果两特征点之间的描述子距离大于阈值或者大于当前最优距离，则跳过该特征点
                    if(dist>TH_LOW || dist>bestDist)
                        continue;
                    
                    //该关键帧2中该特征向量对应的关键点
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                    
                    
                    //如果两个右眼坐标都不为空
                    if(!bStereo1 && !bStereo2)
                    {
                        const float distex = ex-kp2.pt.x;
                        const float distey = ey-kp2.pt.y;
                        
                        // 该特征点距离极点太近，表明kp2对应的MapPoint距离pKF1相机太近
                        if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                            continue;
                    }
                    
                    
                    // 计算特征点kp2到kp1极线（kp1对应pKF2的一条极线）的距离是否小于阈值
                    // 检测匹配点是否满足对极几何的约束
                    if(CheckDistEpipolarLine(kp1,kp2,F12,pKF2))
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }
                
                if(bestIdx2>=0)
                {
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                    vMatches12[idx1]=bestIdx2;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = kp1.angle-kp2.angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }

            f1it++;
            f2it++;
        }
        
        //如果图像1中的特征点节点id小于图像2中特征点节点id  则将图像1特征点的节点id跳转到图像2特征点节点id
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        
        //如果图像2中的特征点节点id小于图像1中特征点节点id  则将图像2特征点的节点id跳转到图像1特征点节点id
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }




    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;
        vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
    }

    return nmatches;
}








// 将MapPoints投影到关键帧pKF中，并判断是否有重复的MapPoints
// 两个重载的Fuse函数，用于地图点的融合
// 地图点能匹配上当前关键帧的地图点，也就是地图点重合了
 * 1.如果MapPoint能匹配关键帧的特征点，并且该点有对应的MapPoint，那么将两个MapPoint合并（选择观测数多的）
 * 2.如果MapPoint能匹配关键帧的特征点，并且该点没有对应的MapPoint，那么为该点添加MapPoint
     pKF         相邻关键帧
     vpMapPoints 当前关键帧的MapPoints
     th          搜索半径的因子
     return      重复MapPoints的数量
int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
{
    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();

    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    const float &bf = pKF->mbf;

    cv::Mat Ow = pKF->GetCameraCenter();

    int nFused=0;

    const int nMPs = vpMapPoints.size();


    // 遍历所有的MapPoints
    for(int i=0; i<nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc = Rcw*p3Dw + tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;
        
        
        // 步骤1：得到MapPoint在图像上的投影坐标
        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        const float ur = u-bf*invz;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist3D,pKF);



        // Search in a radius
        // 步骤2：根据MapPoint的深度确定尺度，从而确定搜索范围
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        
        
        // 步骤3：遍历搜索范围内的features
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF->mvKeysUn[idx];

            const int &kpLevel= kp.octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;
            
            
            // 计算MapPoint投影的坐标与这个区域特征点的距离，如果偏差很大，直接跳过特征点匹配
            if(pKF->mvuRight[idx]>=0)
            {
                // Check reprojection error in stereo
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->mvuRight[idx];
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float er = ur-kpr;
                const float e2 = ex*ex+ey*ey+er*er;

                if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                    continue;
            }
            else
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;
                
                
                // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
                if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                    continue;
            }

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);
            
            
            // 找MapPoint在该区域最佳匹配的特征点
            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }



        // If there is already a MapPoint replace otherwise add new measurement
        // 找到了MapPoint在该区域最佳匹配的特征点
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            
            
            // 如果这个点有对应的MapPoint
            if(pMPinKF)
            {   
              
                // 如果这个MapPoint不是bad，用被观察次数多的地图点代替被观察次数少的
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->Observations()>pMP->Observations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            }
            
            
            // 如果这个点没有对应的MapPoint
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}



// 主要思路: 将回环地图点根据当前关键帧的位姿(相似矩阵)映射到当前关键帧,在当前关键帧中寻找回环地图点的代替点,存储进vpReplacePoint
 *    根据相似矩阵Scw映射地图点到关键帧  匹配地图点为:vpReplacePoint   用于回环检测
 * 		pKF(in):映射关键帧
 * 		Scw(in):关键帧的相似矩阵位姿
 * 		vpPoints(in):待映射地图点
 * 		th(in):阈值
 * 		vpReplacePoint(out):匹配到的地图点
// 重载的函数是为了减少尺度漂移的影响，需要知道当前关键帧的sim3位姿
// Scw为世界坐标系到pKF机体坐标系的Sim3变换，用于将世界坐标系下的vpPoints变换到机体坐标系
int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;



    // Decompose Scw
    // 将Sim3转化为SE3并分解
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0))); // 计算得到尺度s
    cv::Mat Rcw = sRcw/scw;                               // 除掉s
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;           // 除掉s
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();





    // For each candidate MapPoint project and match
    // 将地图点映射为当前关键帧的像素坐标,根据三维点深度确定搜索半径从而确定搜索区域,在搜索区域内寻找匹配点
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        // Project into Image
        const float invz = 1.0/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;
        
        
        // 得到MapPoint在图像上的投影坐标
        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;



        // Depth must be inside the scale pyramid of the image
        // 根据距离是否在图像合理金字塔尺度范围内和观测角度是否小于60度判断该MapPoint是否正常
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        // Compute predicted scale level
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];


        // 收集pKF在该区域内的特征点   
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;
            const int &kpLevel = pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            
            
            // 如果这个MapPoint已经存在，则替换，
            // 这里不能轻易替换（因为MapPoint一般会被很多帧都可以观测到），这里先记录下来，之后调用Replace函数来替换
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;    //记录下来该pMPinKF需要被替换掉
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}







 * 通过相似矩阵计算匹配地图点
 * 		pKF1(in):关键帧1
 * 		pKF2(in):关键帧2
 * 		vpMatches12(out):匹配地图点
 * 		s12(in):关键帧1->关键帧2的相似矩阵中的尺度参数
 * 		R12(in):关键帧1->关键帧2的相似矩阵中的旋转矩阵参数
 * 		t12(in):关键帧1->关键帧2的相似矩阵中的位移矩阵参数
 * 		th(in):阈值
 * 步骤:
 * 		计算关键帧1 2的相机位姿以及相机1->相机2的位姿变换相似矩阵
 * 		双向匹配:
 * 		根据相似矩阵将关键帧1中的地图点向关键帧2中投影,确定投影区域,并在投影区域内寻找关键帧1中地图点的匹配
 * 		根据相似矩阵将关键帧2中的地图点向关键帧1中投影,确定投影区域,并在投影区域内寻找关键帧2中地图点的匹配
 * 		根据双向匹配结果,如果两次匹配都能成功,则确定该对匹配是有效的.将其存入vpMatches12容器   
 * 		最终返回匹配点对个数
// 通过Sim3变换，确定pKF1的特征点在pKF2中的大致区域，同理，确定pKF2的特征点在pKF1中的大致区域
// 在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新vpMatches12（之前使用SearchByBoW进行特征点匹配时会有漏匹配）
int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                             const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
{
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 from world
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    //Camera 2 from world
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    //Transformation between cameras
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();
    cv::Mat t21 = -sR21*t12;

    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);    // 用于记录该特征点是否被处理过
    vector<bool> vbAlreadyMatched2(N2,false);    // 用于记录该特征点是否在pKF1中有匹配
    
    
    // 用vpMatches12更新vbAlreadyMatched1和vbAlreadyMatched2
    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;                // 该特征点已经判断过
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;         // 该特征点在pKF1中有匹配
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);



    // Transform from KF1 to KF2 and search
    // 在KF2中寻找KF1各地图点的匹配
    // 通过Sim变换，确定pKF1的特征点在pKF2中的大致区域,在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点,更新vpMatches12
    //（之前使用SearchByBoW进行特征点匹配时会有漏匹配）
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];


        // 该特征点已经有匹配点了，直接跳过
        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos(); // 关键帧1中的地图点坐标
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;    // 把pKF1系下的MapPoint从world坐标系变换到camera1坐标系
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;  // 再通过Sim3将该MapPoint从camera1变换到camera2坐标系



        // Depth must be positive
        // 检测相机坐标系下三维点坐标的深度
        if(p3Dc2.at<float>(2)<0.0)
            continue;
        
        
        // 将关键帧1中的地图点映射到相机2的像素坐标
        const float invz = 1.0/p3Dc2.at<float>(2);
        const float x = p3Dc2.at<float>(0)*invz;
        const float y = p3Dc2.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;



        // Point must be inside the image
        // 验证像素点坐标必须在图片内
        if(!pKF2->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc2);




        // Depth must be inside the scale invariance region
        // 地图点在关键帧2中的深度符合该地图点的最大观测深度和最小观测深度
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;



        // Compute predicted octave
        // 通过地图点在关键帧2的深度预测该地图点在关键帧2中图像金字塔的层数
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);



        // Search in a radius
        // 通过高斯金字塔的层数预测投影半径
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];


        // 在关键帧1的地图点投影到关键帧2区域内寻找关键帧1的地图点匹配
        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        // 得到当前待匹配的关键帧1中的地图点描述子
        const cv::Mat dMP = pMP->GetDescriptor();




        int bestDist = INT_MAX;
        int bestIdx = -1;
        
        // 在投影区域内寻找地图点的匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx;
        }
    }




    // Transform from KF2 to KF2 and search
    // 双向匹配  在KF1中寻找KF2各地图点的匹配
    for(int i2=0; i2<N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc2 = R2w*p3Dw + t2w;
        cv::Mat p3Dc1 = sR12*p3Dc2 + t12;

        // Depth must be positive
        if(p3Dc1.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc1.at<float>(2);
        const float x = p3Dc1.at<float>(0)*invz;
        const float y = p3Dc1.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }





















    // Check agreement
    // 检测双向匹配的结果都成功,证明是匹配点
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];
            if(idx1==i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}






    // 从上一帧映射地图点与当前帧进行匹配，使用追踪线程追踪上一帧的位姿
    // 根据上一帧到当前帧的位姿,将上一帧的地图点投影到当前帧,然后将地图点反投影到像素坐标,在像素坐标一定范围内寻找最佳匹配点
    // 注意这里用到的当前帧的位姿是根据上一帧的位姿和上一帧的位姿变化速度来推算的相机位姿
    // 当前帧匹配上一帧的地图点，即前后两帧匹配，用于TrackWithMotionModel
    CurrentFrame 当前帧
    LastFrame    上一帧
    th           阈值
    bMono        是否为单目
    return       成功匹配的数量
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];     // 旋转直方图，为了检测旋转方向的一致性
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
    
    
    //当前帧的相机位姿
    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    //世界坐标系到当前相机坐标系的位移向量
    const cv::Mat twc = -Rcw.t()*tcw;



    //上一帧的相机位姿
    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);
    //上一帧相机坐标系到当前帧相机坐标系的位移向量
    const cv::Mat tlc = Rlw*twc+tlw;
    
    
    // 判断前进还是后退
    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;  // 非单目情况，如果Z大于基线，则表示前进
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;// 非单目情况，如果Z小于基线，则表示前进
    
    
    // 遍历上一帧所有的特征点
    for(int i=0; i<LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                // Project    
                // 将上一帧的局内点(地图点)映射到当前帧
                // 对上一帧有效的MapPoints进行跟踪
                cv::Mat x3Dw = pMP->GetWorldPos();
                // 上一帧的局内地图点映射到当前帧的地图点坐标
                cv::Mat x3Dc = Rcw*x3Dw+tcw;



                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);
                
                
                //判断地图点深度是否是正值
                if(invzc<0)
                    continue;
                
                
                // 计算映射到当前帧的像素坐标
                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;
                
                
                //判断映射后的像素坐标是否在图像范围内
                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;
                
                
                
                //上一帧关键点所在金字塔的层数
                int nLastOctave = LastFrame.mvKeys[i].octave;
                // Search in a window. Size depends on scale
                // 特征点所在高斯金字塔的层数决定了半径大小
                // 尺度越大，搜索范围越大
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];
                
                
                
                
                // 投影区域内的特征点索引
                vector<size_t> vIndices2;



                尺度越大,图像越小
                // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
                // 当前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
                // 因此m>=n，对应前进的情况，nCurOctave>=nLastOctave。后退的情况可以类推
                if(bForward)                  // 前进,则上一帧兴趣点在所在的尺度nLastOctave<=nCurOctave
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);
                else if(bBackward)            // 后退,则上一帧兴趣点在所在的尺度0<=nCurOctave<=nLastOctave
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
                else                          // 在[nLastOctave-1, nLastOctave+1]中搜索
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;
                
                
                // 遍历投影区域内所有的特征点
                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                            continue;

                    if(CurrentFrame.mvuRight[i2]>0)
                    {   
                      
                        // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                        const float ur = u - CurrentFrame.mbf*invzc;
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                        if(er>radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    //Apply rotation consistency
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {   
              
                //只保留三个主方向上的特征点
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)  
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}






  // 在当前帧中匹配所有关键帧中的地图点，用于Relocalization
  // 映射在关键帧中观察到的地图点与当前帧进行匹配，使用在重定位中   
  // 在通过BOW匹配当前帧和重定位候选关键帧内点数量不足时,通过此方式增加匹配点
  // 选取其中一个用于Relocalization的投影匹配着重理解。疑问是，何时用投影匹配，何时用DBoW2进行匹配？
  // 在Relocalization和LoopClosing中进行匹配的是很多帧关键帧集合中匹配，属于Place Recognition，因此需要DBoW
  // 而投影匹配适用于两帧之间，或者投影范围内(局部地图，前一个关键帧对应地图点)的MapPoints与当前帧之间
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    const cv::Mat Ow = -Rcw.t()*tcw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
   
   
    //获取pKF对应的地图点vpMPs,遍历
    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];
        
        
        
        //若该点为NULL，isBad或者在SearchByBow中已经匹配
        //Rlocalization中首先会通过SearchByBow匹配一次，抛弃
        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);   // 深度是取倒数

                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;



                // Compute predicted scale level
                // 根据地图点到光心的距离来推算该特征点对应的高斯金字塔层数
                cv::Mat PO = x3Dw-Ow;
                float dist3D = cv::norm(PO);

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if(dist3D<minDistance || dist3D>maxDistance)
                    continue;

                int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                // Search in a window
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];
                
                
                // 搜索得到候选匹配点向量vIndices2
                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;
                
                
                // 计算地图点的描述子和候选匹配点的描述子距离
                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;
                
                
                //获得最近距离的最佳匹配
                //但是也要满足距离小于设定的阈值ORBdist
                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=ORBdist)
                {   
                  
                    // 当前帧生成和关键帧匹配上的地图点
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    
                    // 统计通过投影匹配上的点
                    nmatches++;
                    
                    
                    //最后，还需要通过直方图验证描述子的方向是否匹配
                    //其中角度直方图是用来剔除不满足两帧之间角度旋转的外点的
                    //也就是所谓的旋转一致性检测
                    if(mbCheckOrientation)
                    {   
                      
                        //将关键帧与当前帧匹配点的angle相减，得到rot(0<=rot<360)
                        float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        
                        
                        
                        //放入一个直方图中，对于每一对匹配点的角度差均可以放入一个bin的范围内(360/HISTO_LENGTH)
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }
    
    
    
    // 统计直方图最高的三个bin保留，其他范围内的匹配点剔除
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}


// 另外，若最高的比第二高的高10倍以上，则只保留最高的bin中的匹配点
// 功能:筛选出在直方图区间内特征点数量最多的三个特征点方向的索引
     histo  两描述子旋转距离的直方图
     L      直方图宽度
     ind1   第一最小旋转距离的索引
     ind2   第二最小旋转距离的索引
     ind3   第三最小旋转距离的索引
     如果max2/max1<0.1  那么证明第二第三方向不具有区分性,则将其索引置位初值
     如果max3/max1<0.1  那么证明第三方向不具有区分性,则将其索引置位初值
void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;   // 在直方图区间内特征点数量最大值
    int max2=0;   // 在直方图区间内特征点数量第二最大值
    int max3=0;   // 在直方图区间内特征点数量第三最大值

    for(int i=0; i<L; i++)
    {   
      
        // 在该直方图区间内特征点数量
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)     // 如果max2/max1<0.1  那么证明第二第三方向不具有区分性,则将其索引置位初值
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)// 如果max3/max1<0.1  那么证明第三方向不具有区分性,则将其索引置位初值
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
// 计算描述子的汉明距离  位运算 提高速度
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
