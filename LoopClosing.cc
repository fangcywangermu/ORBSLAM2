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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;   // 连续帧的阈值是3,也就是在检测是否为闭环时,需要连续3帧都符合
}


// 设置与之对应的追踪线程
void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}




// 设置与之对应的局部地图
void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}





// 我们需要在KeyFrameDataBase中寻找与mlpLoopKeyFrameQueue相似的闭环候选帧
// 主要过程包括：寻找回环候选帧，检查候选帧连续性，计算sim3；闭环(地图点融合，位姿图优化)
void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {   
      
        
        // Check if there are keyframes in the queue
        // 检测是否存在新的关键帧
        // Loopclosing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
        // 在LocalMapping中通过InsertKeyFrame将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
        // 闭环检测队列mlpLSearchBySim3oopKeyFrameQueue中的关键帧不为空
        if(CheckNewKeyFrames())
        {
          
          
          
            // Detect loop candidates and check covisibility consistency
            *  功能:检测是否产生了回环
            *  检测回环的步骤:
            *       1  检测上次回环发生是否离当前关键帧足够长时间    并且满足当前关键帧总数量大于10
            *       2  找出当前关键帧的共视关键帧,并找出其中的最小得分
            *       3  根据最小得分寻找回环候选帧   具体见含函数DetectLoopCandidates
            * 			4  在候选回环关键帧中寻找具有连续性的关键帧
            // 这里将候选回环关键帧和他的共视关键帧组成一个候选组
            // 一个组和另一个组是连续的,是指他们至少存在一个共视关键帧
            // 如果两个组之间存在足够多的帧是共视关键帧,则证明两个组之间是完全连续组,则说明发生了回环
            if(DetectLoop())
            {
              
              
              
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
                * 	计算每一个回环候选关键帧与当前关键帧之间的相似矩阵
                * 		1. 首先通过BOW向量将回环候选关键帧和当前关键帧进行匹配,得到匹配地图点,通过匹配地图点初始化相似矩阵求解器
                * 		2. 遍历所有的回环候选关键帧和当前关键帧计算sim矩阵,并优化sim矩阵,根据优化sim矩阵确定匹配内点数量,从而确定此sim矩阵的准确性,以及是否可以判定为一回环.
                * 		3. 在找到的回环关键帧周围查找共视关键帧   并匹配共视关键帧的地图点和当前关键帧  相当与是匹配covisual graph和当前关键帧  根据匹配点数量确定当前帧是否发生了回环
               if(ComputeSim3())
               {
                 
                 
                 
                  
                   // Perform loop fusion and pose graph optimization
                        * 	根据回环进行位姿矫正
                        * 		1. 请求局部地图线程停止,并且中止现有的全局优化进程
                        *		  2. 根据当前帧求得的相机位姿(相似变换矩阵)来求解矫正前和矫正后的相邻帧位姿变换矩阵(相似变换矩阵)
                        * 		3. 将相邻关键帧的所有地图点都根据更新后的相机位姿(相似变换矩阵)重新计算地图点世界坐标  
                        * 		4. 进行地图点融合   将之前匹配的(在ComputeSim3()函数中计算局部地图点和当前帧的匹配)两地图点融合为同一地图点
                        * 		5. 根据第3步中计算的地图点重新进行匹配,并融合匹配点和当前关键帧中的地图点
                        * 		6. 在地图点融合之后,更新当前关键帧的共视图中各个关键帧的相连关键帧,更新连接之后,将这些相邻关键帧全部加入LoopConnections容器
                        * 		7. 根据四种边(1 新检测到的回环边  2 父关键帧与子关键帧的边 3 历史回环关键帧  4 共视图边)对全局地图中的所有关键帧的位姿进行矫正
                         * 		8. 根据地图点和关键帧位姿计算重投影误差对全局地图进行优化
                   CorrectLoop();
               }
            }
        }       
        
        
        // 检测是否有复位请求
        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}



// LoopClosing中的关键帧是LocalMapping中送过来的：送来一帧，就检查一帧
void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}



// 检测回环检测关键帧队列是否为空
bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}






 *  功能:检测是否产生了回环
 *  检测回环的步骤:
 *       1  检测上次回环发生是否离当前关键帧足够长时间    并且满足当前关键帧总数量大于10
 *       2  找出当前关键帧的共视关键帧,并找出其中的最小得分
 *       3  根据最小得分寻找回环候选帧   具体见含函数DetectLoopCandidates
 * 			4  在候选回环关键帧中寻找具有连续性的关键帧
// 这里将候选回环关键帧和他的共视关键帧组成一个候选组
// 一个组和另一个组是连续的,是指他们至少存在一个共视关键帧
// 如果两个组之间存在足够多的帧是共视关键帧,则证明两个组之间是完全连续组,则说明发生了回环
bool LoopClosing::DetectLoop()
{
    {   
        // 从队列中取出一个关键帧
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        
        
        // Avoid that a keyframe can be erased while it is being process by this thread
        // 设置当前关键帧不可被擦除(防止进程运行过程中,当前关键帧被擦除),当检测完回环之后重新设为可被擦除
        mpCurrentKF->SetNotErase();
    }





    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    // 步骤1：如果距离上次闭环没多久(通过的关键帧帧数不超过10，则将该关键帧添加到关键帧集中，将该关键帧设为可擦除关键帧），或者map中关键帧总共还没有10帧，则不进行闭环检测
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }





    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    
    // 当前关键帧的共视关键帧
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    
    
    // 得到当前关键帧的BOW向量
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    
    
    // 步骤2：循环每个共视关键帧  计算每个共视关键帧与当前待检测回环关键帧之间的BOW得分  并得到其中最小的得分
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
            
            
        // 共视关键帧的BOW向量
        const DBoW2::BowVector &BowVec = pKF->mBowVec;
        // 得到共视关键帧和当前关键帧的BOW向量得分
        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);
        // 得到最小的得分
        if(score<minScore)
            minScore = score;
    }
    
    
    

    // Query the database imposing the minimum score
    // 步骤3：在关键帧数据集中查找当前关键帧的回环候选关键帧  最小得分大于minScore
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }
    
    
    
    
    
    

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    // 一个组和另一个组是连续的,是指他们至少存在一个共视关键帧
    // 步骤4：在候选帧中检测具有连续性的候选帧
    // 1、每个候选帧将与自己相连的关键帧构成一个“子候选组spCandidateGroup”，vpCandidateKFs-->spCandidateGroup
    // 2、检测“子候选组”中每一个关键帧是否存在于“连续组”，如果存在nCurrentConsistency++，则将该“子候选组”放入“当前连续组vCurrentConsistentGroups”
    // 3、如果nCurrentConsistency大于等于3，那么该”子候选组“代表的候选帧过关，进入mvpEnoughConsistentCandidates
    // 筛选后得到的具有连续性的候选帧
    // 候选关键帧需要进行连续性检验的原因: 
    // 我们通过聚类相连候选帧,可以将一些得分很高但却相对独立的帧给去掉，这些帧与其他帧相对没有关联，
    // 而我们知道事实上回环处会有一个时间和空间上的连续性，因此对于正确的回环来讲，这些相似性评分较高的帧是错误关键帧。
    mvpEnoughConsistentCandidates.clear();    // 最终筛选后得到的闭环帧
    
    
    
    
    // ConsistentGroup数据类型为pair<set<KeyFrame*>,int>
    // ConsistentGroup.first对应每个“连续组”中的关键帧，ConsistentGroup.second为当前该连续组与其他连续组之间连续的连续组数量
    // 当前的连续组
    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    
    
    
    // 遍历那些分数够的关键帧(关键帧数据库里的)
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];
        
        
        // 将自己以及与自己相连的关键帧构成一个“子候选组”
        // 得到与该关键帧连接的关键帧，注意和共视帧区分，这个是没有按照权重排序的
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        
        
        // 遍历之前的“子连续组”
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {   
            
            
            // 取出一个之前的子连续组
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;
            
            
            
            bool bConsistent = false;
            // 遍历每个“子候选组”，检测候选组中每一个关键帧在“子连续组”中是否存在
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {   
              
                // 如果有一帧共同存在于“子候选组”与之前的“子连续组”，那么“子候选组”与该“子连续组”连续
                if(sPreviousGroup.count(*sit))
                {   
                    bConsistent=true;             // 该“子候选组”与该“子连续组”相连
                    bConsistentForSomeGroup=true; // 该“子候选组”至少与一个”子连续组“相连
                    break;
                }
            }
            
            
            // 如果与之前的连续组是连续的  则将它加入到当前连续组中
            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;-
                int nCurrentConsistency = nPreviousConsistency + 1;
                
                
                
                // vbConsistentGroup(false)表示 mvConsistentGroups不为零, i表示有多少个vpCandidateKFs, IG表示mvConsistentGroups大小, 所以这里应该是i而不是IG
                if(!vbConsistentGroup[i])
                {   
                  
                    // 将该“子候选组”的该关键帧打上编号加入到“当前连续组”
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    
                    // 当前连续组
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                
                
                // 如果与当前连续组连续的其他连续组之间连续数量大于某一阈值,则说明他有足够多的连续组  将其加入足够连续组集   mnCovisibilityConsistencyTh=3
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
                break;
            }
        }



        // If the group is not consistent with any previous group insert with consistency counter set to zero
        // 如果该“子候选组”的所有关键帧都不存在于“子连续组”，那么vCurrentConsistentGroups将为空，
        // 于是就把“子候选组”全部拷贝到vCurrentConsistentGroups，并最终用于更新mvConsistentGroups，计数器设为0，重新开始
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }



    // Update Covisibility Consistent Groups
    // 更新连续组
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    // 添加关键帧到关键帧集中
    mpKeyFrameDB->add(mpCurrentKF);
    
    
    
    // 如果足够连续的候选组为空则将返回false  ,如果存在足够连续候选组  则证明发生回环
    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }
    
    
    // 设置当前关键帧可以被擦除,与刚进行检测时形成呼应
    mpCurrentKF->SetErase();
    return false;
}





 * 	计算每一个回环候选关键帧与当前关键帧之间的相似矩阵
 * 		1. 首先通过BOW向量将回环候选关键帧和当前关键帧进行匹配,得到匹配地图点,通过匹配地图点初始化相似矩阵求解器
 * 		2. 遍历所有的回环候选关键帧和当前关键帧计算sim矩阵,并优化sim矩阵,根据优化sim矩阵确定匹配内点数量,从而确定此sim矩阵的准确性,以及是否可以判定为一回环.
 * 		3. 在找到的回环关键帧周围查找共视关键帧   并匹配共视关键帧的地图点和当前关键帧  相当与是匹配covisual graph和当前关键帧  根据匹配点数量确定当前帧是否发生了回环
 
 * 1. 通过Bow加速描述子的匹配，利用RANSAC粗略地计算出当前帧与闭环帧的Sim3（当前帧---闭环帧）
 * 2. 根据估计的Sim3，对3D点进行投影找到更多匹配，通过优化的方法计算更精确的Sim3（当前帧---闭环帧）
 * 3. 将闭环帧以及闭环帧相连的关键帧的MapPoints与当前帧的点进行匹配（当前帧---闭环帧+相连关键帧）
 * 注意以上匹配的结果均都存在成员变量mvpCurrentMatchedPoints中
bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;               // 相似性矩阵求解器
    vpSim3Solvers.resize(nInitialCandidates);        // 每个候选帧都有一个Sim3Solver

    vector<vector<MapPoint*> > vvpMapPointMatches;  // 匹配地图点容器
    vvpMapPointMatches.resize(nInitialCandidates);
  
    vector<bool> vbDiscarded;                       // 取消候选回环关键帧的标志变量
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches
    
    
    
    // 初始化当前帧和候选关键帧的相似矩阵求解器,计算匹配地图点,取消回环关键帧标志变量的赋初值
    for(int i=0; i<nInitialCandidates; i++)
    {   
      
        // 步骤1：从筛选的闭环候选帧中取出一帧关键帧pKF
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];


        // avoid that local mapping erase it while it is being processed in this thread
        // 防止在LocalMapping中KeyFrameCulling函数将此关键帧作为冗余帧剔除
        pKF->SetNotErase();
        
        
        // 直接将该候选帧舍弃
        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }
        
        
        // 步骤2：将当前帧mpCurrentKF与闭环候选关键帧pKF匹配
        // 通过bow加速得到mpCurrentKF与pKF之间的匹配特征点，vpMapPointMatches是匹配特征点对应的MapPoints
        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);
        
        
        // 匹配的特征点数太少，该候选帧剔除
        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {   
            // 相似性矩阵的求解器初始化
            // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目）
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            
            // 至少20个inliers 300次迭代
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }
        
        // 参与Sim3计算的候选关键帧数加1
        nCandidates++;
    }
    
    
    // 用于标记是否有一个候选帧通过Sim3的求解与优化
    bool bMatch = false;




    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    // RANSAC迭代每一个回环候选关键帧和当前待回环关键帧
    // 一直循环所有的候选帧，每个候选帧迭代5次，如果5次迭代后得不到结果，就换下一个候选帧
    // 直到有一个候选帧首次迭代成功bMatch为true，或者某个候选帧总的迭代次数超过限制，直接将它剔除
    // 通过SearchByBoW匹配得到初步匹配点,根据此匹配计算两关键帧之间的sim矩阵
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;
            
            
            
            // 步骤3：对步骤2中有较好的匹配的关键帧求取Sim3变换
            Sim3Solver* pSolver = vpSim3Solvers[i];
            
            
            // 最多迭代5次，返回的Scm是候选帧pKF到当前帧mpCurrentKF的Sim3变换（T12）
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);



            // If Ransac reachs max. iterations discard keyframe
            // 经过n次循环，每次迭代5次，总共迭代 n*5 次
            // 总迭代次数达到最大限制还没有求出合格的Sim3变换，则证明当前候选帧不是回环帧,该候选帧剔除
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }





            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            // 如果RANSAC求取了一个合适的相似变换矩阵sim   则通过sim矩阵重新进行地图点匹配,并优化sim矩阵并确定内点  根据内点数量判定sim矩阵是否符合要求
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {   
                  
                  
                    // 保存inlier的MapPoint
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }
                
                
                // 步骤4：通过步骤3求取的Sim3变换引导关键帧匹配弥补步骤2中的漏匹配
                // [sR t;0 1]
                cv::Mat R = pSolver->GetEstimatedRotation();                       // 候选帧pKF到当前帧mpCurrentKF的R（R12）
                cv::Mat t = pSolver->GetEstimatedTranslation();                    // 候选帧pKF到当前帧mpCurrentKF的t（t12），当前帧坐标系下，方向由pKF指向当前帧
                const float s = pSolver->GetEstimatedScale();                      // 候选帧pKF到当前帧mpCurrentKF的变换尺度s（s12）
                
                
                // 查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数，之前使用SearchByBoW进行特征点匹配时会有漏匹配）
                // 通过Sim3变换，确定pKF1的特征点在pKF2中的大致区域，同理，确定pKF2的特征点在pKF1中的大致区域
                // 在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新匹配vpMapPointMatches
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);
                
                
                
                
                // 步骤5：Sim3优化，只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                // OpenCV的Mat矩阵转成Eigen的Matrix类型
                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                
                
                // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目）
                // 优化mpCurrentKF与pKF对应的MapPoints间的Sim3，得到优化后的量gScm
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);



                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    
                    
                    // mpMatchedKF就是最终闭环检测出来与当前帧形成闭环的关键帧
                    mpMatchedKF = pKF;
                    
                   
                    // 得到从世界坐标系到该候选帧的Sim3变换，Scale=1
                    // 其实就是闭环关键帧的姿态(世界到相机)
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    
                    // 得到g2o优化后从世界坐标系到当前帧的Sim3变换
                    // gScm: 是刚刚求得的两个相机坐标系的相似变换矩阵(闭环帧到当前帧的变换)
                    // 两者相乘,得到世界到当前帧的变换  mg2oScw或mScw
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;     // 只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                }
            }
        }
    }
    
    
    
    // 没有一个闭环匹配候选帧通过Sim3的求解与优化
    if(!bMatch)
    {   
      
        // 清空mvpEnoughConsistentCandidates
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }






    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    // 步骤6：取出闭环匹配上关键帧的相连关键帧，得到它们的MapPoints放入mvpLoopMapPoints
    // 注意是匹配上的那个关键帧：mpMatchedKF
    // 将mpMatchedKF相连的关键帧全部取出来放入vpLoopConnectedKFs
    // 将vpLoopConnectedKFs的MapPoints取出来放入mvpLoopMapPoints
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    
    
    // 包含闭环匹配关键帧本身
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    
                    
                    // 标记该MapPoint被mpCurrentKF闭环时观测到并添加，避免重复添加
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }





    // Find more matches projecting with the computed Sim3
    // 步骤7：将闭环匹配上关键帧mpMatchedKF以及相连关键帧的MapPoints,将这些地图点通过上面优化得到的Sim3(gScm>mScw)变换投影到当前关键帧进行匹配
    // 根据投影查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数）若匹配点>=40个，则返回true,进行闭环调整，否则，返回false,
    // 根据Sim3变换，将每个mvpLoopMapPoints投影到mpCurrentKF上，并根据尺度确定一个搜索区域，根据该MapPoint的描述子与该区域内的特征点进行匹配，如果匹配误差小于TH_LOW即匹配成功，更新mvpCurrentMatchedPoints
    // mvpCurrentMatchedPoints将用于SearchAndFuse中检测当前帧MapPoints与匹配的MapPoints是否存在冲突
    // 注意这里得到的当前关键帧中匹配上闭环关键帧共视地图点(mvpCurrentMatchedPoints)将用于后面CorrectLoop时当时关键帧地图点的冲突融合
    // 到这里，不仅确保了当前关键帧与闭环帧之间匹配度高，而且保证了闭环帧的共视图中的地图点和当前帧的特征点匹配度更高,说明该闭环帧是正确的
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);   // 搜索范围系数为10
                                                                                                   // 第一个参数:当前帧//说明这些(世界坐标系下的)点要投影到当前帧;所以之前要去计算当前帧相机坐标系和世界坐标系之间的变换关系(mScw)
                                                                                                   // 线程暂停5ms后继续接收Tracking发送来的关键帧队列


    // If enough matches accept Loop
    // 步骤8：判断当前帧与检测出的所有闭环关键帧是否有足够多的MapPoints匹配
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }
    
    
    // 步骤9：清空mvpEnoughConsistentCandidates
    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}






     * 	根据回环进行位姿矫正
     * 		1. 请求局部地图线程停止,并且中止现有的全局优化进程(闭环纠正时，LocalMapper和Global BA必须停止。注意Global BA必须停止。)
     *		2. 根据当前帧求得的相机位姿(相似变换矩阵)来求解矫正前和矫正后的相邻帧位姿变换矩阵(相似变换矩阵)，将相邻关键帧的所有地图点都根据更新后的相机位姿(相似变换矩阵)重新计算地图点世界坐标  
     * 		4. 进行地图点融合，将之前匹配的(在ComputeSim3()函数中计算局部地图点和当前帧的匹配)两地图点融合为同一地图点
     * 		5. 根据第3步中计算的地图点重新进行匹配,并融合匹配点和当前关键帧中的地图点
     * 		6. 在地图点融合之后,更新当前关键帧的共视图中各个关键帧的相连关键帧,更新连接之后,将这些相邻关键帧全部加入LoopConnections容器
     * 		7. 根据四种边(1 新检测到的回环边  2 父关键帧与子关键帧的边 3 历史回环关键帧  4 共视图边)对全局地图中的所有关键帧的位姿进行矫正
     * 		8. 根据地图点和关键帧位姿计算重投影误差对全局地图进行优化
     
     
 * 1. 通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的MapPoints的位置（当前帧---相连关键帧）
 * 2. 将闭环帧以及闭环帧相连的关键帧的MapPoints和与当前帧相连的关键帧的点进行匹配（当前帧+相连关键帧---闭环帧+相连关键帧）
 * 3. 通过MapPoints的匹配关系更新这些帧之间的连接关系，即更新covisibility graph
 * 4. 对Essential Graph（Pose Graph）进行优化，MapPoints的位置则根据优化后的位姿做相对应的调整
 * 5. 创建线程进行全局Bundle Adjustment
void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;



    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    // 步骤0：请求局部地图停止，防止局部地图线程中InsertKeyFrame函数插入新的关键帧
    mpLocalMapper->RequestStop();




    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {   
        
        // 这个标志位仅用于控制输出提示，可忽略
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }



    // Wait until Local Mapping has effectively stopped
    // 等待局部地图线程已经完全停止
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    // 步骤1：根据共视关系更新当前帧与其它关键帧之间的连接
    mpCurrentKF->UpdateConnections();





    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    // 步骤2：通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的MapPoints
    // 当前帧与世界坐标系之间的Sim变换在ComputeSim3函数中已经确定并优化
    // 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的Sim3变换
    
    // 取出与当前帧相连的关键帧，包括当前关键帧
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);
    
    // 矫正后的相机位姿          ，未矫正的相机位姿
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    
    
    // 先将mpCurrentKF的Sim3变换存入，固定不动
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        
        
        // 步骤2.1：通过位姿传播，得到Sim3调整后其它与当前帧相连关键帧的位姿（只是得到，还没有修正）
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();
            
            
            // currentKF在前面已经添加
            // 将当前帧的相邻关键帧的相机位姿都根据当前帧的相似变换矩阵进行矫正
            if(pKFi!=mpCurrentKF)
            {   
                // 得到当前帧到pKFi帧的相对变换
                // Tic = Tiw*Twc  =====>  g2oSic(其实就是当前帧到第i帧变换)
                // mg2oScw:世界到当前帧的相似变换
                // g2oSic*mg2oScw :世界到第i帧的变换(经过相似变换一转手....)
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                
                
                // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                
                
                //Pose corrected with the Sim3 of the loop closure
                // 得到闭环g2o优化后各个关键帧的位姿
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }


            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            
            
            //Pose without correction
            // 当前帧相连关键帧，没有进行闭环g2o优化的位姿
            NonCorrectedSim3[pKFi]=g2oSiw;
        }



        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        // 步骤2.2：步骤2.1得到调整相连帧位姿后，修正这些关键帧的MapPoints
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)     // 防止重复修正
                    continue;





                // Project with non-corrected pose and project back with corrected pose
                // eigP3Dw:代表的是世界坐标系下的地图点
                // g2oSiw: 代表的是世界坐标系到相机坐标系的变换(原始)
                // g2oSiw.map(eigP3Dw)=T*p ===> 地图点转换到相机坐标系下P'
                // CorrectedSim3<pkf,g2oSic*mg2oScw>
                // g2oCorrectedSiw = mit->second;
                // g2oSic*mg2oScw :世界到第i帧的变换(经过相似变换一转手....)
                // g2oCorrectedSwi:第i帧到世界的变换
                // g2oCorrectedSwi.map(P'): 从矫正的相机坐标系转化到世界坐标系里---->矫正过的地图点
                
                // 将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }




            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            // 步骤2.3：将Sim3转换为SE3，根据更新的Sim3，更新关键帧的位姿
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();  // 尺度

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            // 步骤2.4：根据共视关系更新当前帧与其它关键帧之间的连接
            pKFi->UpdateConnections();
        }

       
       
       
        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        // 进行地图点融合   将匹配的两地图点融合为同一地图点
        // 步骤3：检查当前帧的MapPoints与闭环匹配帧的MapPoints是否存在冲突，对冲突的MapPoints进行替换或填补（投影匹配上的和Sim3计算过的地图点进行融合(就是替换成高质量的)）
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                
                
                // 如果有重复的MapPoint（当前帧和匹配帧各有一个），则用匹配帧的代替现有的(更相信匹配帧)
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                    
                // 如果当前帧没有该MapPoint，则直接添加
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }




    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    // 步骤4：将闭环帧以及连续帧的地图点投影(用矫正过的位姿)到当前帧中,检查融合
    SearchAndFuse(CorrectedSim3);




    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    // 步骤5：更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;


    
    // 步骤5.1：遍历当前帧相连关键帧（一级相连）
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        
        
        // 步骤5.2：得到与当前帧相连关键帧的相连关键帧（二级相连）
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();




        // Update connections. Detect new links.
        // 步骤5.3：更新一级相连关键帧的连接关系
        pKFi->UpdateConnections();
        
        // 步骤5.4：取出该帧更新后的连接关系
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        
        // 步骤5.5：从连接关系中去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        
        
        // 步骤5.6：从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }



    // Optimize graph
    // 步骤6：进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();




    // Add loop edge
    // 步骤7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);




    // Launch a new thread to perform Global Bundle Adjustment
    // 步骤8：新建一个线程用于全局BA优化
    // OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);



    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}







// 通过将闭环时相连关键帧的MapPoints投影到关键帧中，进行MapPoints检查与替换
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);
    
    
    
    
    // 遍历当前帧以及相连的关键帧
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);
        
        
        // 将闭环相连帧的MapPoints坐标变换到pKF帧坐标系，然后投影，检查冲突并融合
        // 将地图点投影到这些刚被纠正位姿的相连关键帧中(更相信这些点)
        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);     // 搜索区域系数为4

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);     // 用mvpLoopMapPoints[i]替换掉之前的那个点
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}



// 检测是否需要复位操作,如果需要则将回环关键帧序列清空  上次回环关键帧id清零  复位请求清零
void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);






    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    // 有某些地图点和关键帧是没加入到全局优化中进行优化的,这个时候我们需要利用关键帧的子关键帧和地图点的参考关键帧来计算其优化后的值
    {
        unique_lock<mutex> lock(mMutexGBA);
        
        
        // 如果在全局优化过程中又检测到其他回环,则本次回环取消
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped  停止局部地图线程


            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);




            // Correct keyframes starting at map first keyframe    从地图的第一帧关键帧开始矫正关键帧位姿
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                
                
                
                
                
                // 地图中第一关键帧的相机位姿的逆
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    
                    
                    // 如果该关键帧没有参加全局优化
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {   
                      
                      
                        //  从第一帧到当前子关键帧的相机变换矩阵
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;   子关键帧的相机位姿
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;





                // 如果该地图点加入了全局优化
                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                
                
                // 如果该地图点没有加入全局优化,则根据他的参考关键帧计算地图点的三维坐标
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }            

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
