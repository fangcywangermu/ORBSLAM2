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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{


//构造函数，传入的是ORBVocabulary
//关键帧数据库通过预先训练好的词典，维护一个向量 mvInvertedFile
KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());                 // mvInvertedFile[i]表示包含了第i个word id的所有关键帧
}



//根据词袋模型增加关键帧
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);
    
    //mvInvertedFile[i]表示第i个WordId的所有关键帧, vit->first取出的就是WordID
    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);           //将该关键帧加入该关键帧的所有BOW向量节点下
                                                             //BoW向量中存储着叶子节点的编号和叶子节点的权重
                                                             //为每一个word添加该KeyFrame
}




//删除对应的关键帧
void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    // 每一个KeyFrame包含多个words，遍历mvInvertedFile中的这些words，然后在word中删除该KeyFrame
    // 对整个vector进行遍历
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        // 对每个voc进行再次遍历
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
        
        
        //如果list中存在指定KF,删除后重新检索其他list
        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}



//清除对应的mvInvertedFile,重新初始化mvInvertedFile的尺寸
void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();                        //mvInvertedFile[i]表示包含了第i个word id的所有关键帧
    mvInvertedFile.resize(mpVoc->size());          //mpVoc：预先训练好的词典
} 

 *          功能: 得到回环候选帧
 *          将所有与当前帧具有公共单词id的所有关键帧(不包括与当前关键帧链接共视的关键帧)都设为候选关键帧,然后进行筛选
 *          筛选条件:
 *                    1  根据共有单词数来筛选   筛选最大共有单词数0.8倍以上的所有关键帧为候选关键帧
 *                    2  根据候选关键帧和当前待回环关键帧之间的BOW得分来筛选候选关键帧(大于阈值minScore得分的关键帧)
 *                    3  根据候选关键帧的前10个共视关键帧的累积回环得分来筛选回环候选关键帧(大于0.75最大累积得分的所有回环候选帧,并将得分大于当
 *                               前候选关键帧的共视关键帧代替当前候选关键帧)
 
 * 1. 找出和当前帧具有公共单词的所有关键帧（不包括与当前帧相连的关键帧）
 * 2. 只和具有共同单词较多的关键帧进行相似度计算
 * 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数最高的关键帧
 // pKF      需要闭环的关键帧
 // minScore 相似性分数最低要求
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{   
  
    // 提出所有与该pKF相连的KeyFrame，这些相连Keyframe都是局部相连，在闭环检测的时候将被剔除
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    
    // 用于保存可能与pKF形成回环的候选帧（只要有相同的word，且不属于局部相连帧）
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    // 步骤1：找出和当前帧pKF具有公共单词的所有关键帧pKFi（不包括与当前帧链接的关键帧）
    {
        unique_lock<mutex> lock(mMutex);
        
        // words是检测图像是否匹配的枢纽，遍历该pKF的每一个word
        // 关键帧的所有BOW向量
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {   
            //提取所有包含该word的KeyFrame
            //寻找每一BOW向量所在词典节点中所有的关键帧序列
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
            
            //遍历这些关键帧序列，查找这些关键帧的回环关键帧是否是本帧
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                
                // pKFi还没有标记为pKF的候选帧，该关键帧还没有加入lKFsSharingWords容器
                if(pKFi->mnLoopQuery!=pKF->mnId)
                {
                    pKFi->mnLoopWords=0;
                    
                    // 与pKF局部链接的关键帧不进入闭环候选帧
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery=pKF->mnId;            // pKFi标记为pKF的候选帧
                        lKFsSharingWords.push_back(pKFi);       // 将本关键帧加入回环列表中
                    }
                }
                pKFi->mnLoopWords++;        // 记录pKFi与pKF具有相同word的个数
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环候选帧中与pKF具有共同单词最多的单词数
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }


    // 设定一个阈值，只有当共有单词数大于0.8*maxCommonWords以及匹配得分大于给定的minScore的关键帧，存入IScoreAndMatch
    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // 步骤3：遍历所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    // 对于上文筛选出来的pKFi,每一个都要抽取出自身的共视（共享地图点最多的前10帧）关键帧分为一组，计算该组整体得分（与pKF比较的），记为bestAccScore;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {   
        // 循环回环候选帧中所有的帧
        KeyFrame* pKFi = *lit;
        
        // pKF只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;
            
            // 检测待回环关键帧与当前候选回环关键帧的BOW得分
            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;       //回环BOW得分
            if(si>=minScore)             //将得分小于最小BOW阈值的候选回环关键帧删除
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧pKFi相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 步骤4：具体而言：lScoreAndMatch中每一个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);       // 检测候选回环关键帧的前10帧共视关键帧，这里Neigh应该是想说Neighbor
        
        // 当前回环候选帧的最高分(与回环候选帧共视帧的前10帧中与当前待回环关键帧回环得分中的最高分)
        float bestScore = it->first;
        
        // 当前回环关键帧的累计得分(与回环候选帧共视帧的前10帧如果也与当前帧构成回环,则将它的得分累计进来)
        float accScore = it->first;
        
        // 最高回环得分的关键帧
        KeyFrame* pBestKF = pKFi;
        
        // 检测候选回环关键帧的前10帧共视关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            
            
            // 共视的帧也和当前帧有很强的BoW的反应
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)  
            {
                accScore+=pKF2->mLoopScore;
                
                // 统计得到组里分数最高的KeyFrame
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;                  //更新最强回环帧
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));    //每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
        if(accScore>bestAccScore)
            bestAccScore=accScore;                //更新累计最高分
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 所有组得分大于0.75*bestAccScore的，均当做闭环候选帧
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());


    // 步骤5：得到组得分大于minScoreToRetain的组，根据累计得分对其进行筛选，只取前75%的关键帧
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))       // 判断该pKFi是否已经在队列中了
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}




//检测重定位候选帧
//闭环候选帧和重定位候选帧之间的区别是闭环候选帧是从关键帧中选取，重定位候选帧是从普通帧中选取
//其他的流程和闭环候选帧选取方式相同

在所有关键帧中检测重定位候选关键帧
          候选关键帧的选择标准:
          1  首先查找与该帧存在相同词典节点的所有关键帧作为候选关键帧
          2  然后根据关键帧与待重定位帧的相同节点数来删除相同节点数小的关键帧
          3  之后计算每个关键帧的累计共视相似度得分  并且如果该关键帧的共视关键帧比该关键帧有更多的得分就用该共视关键帧代替该关键帧
			  累计共视相似度得分的计算方法:  
			    根据两帧之间相同节点在词典中相应节点的得分来计算两帧的共视相似度 如果共视相似度大的证明有更多的可能进行重定位
          重定位候选关键帧的存储容器(经过层层筛选的)
          lKFsSharingWords--->lScoreAndMatch--->lAccScoreAndMatch--->vpRelocCandidates
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{   
  
    // 相对于关键帧的闭环检测DetectLoopCandidates，重定位检测中没法获得相连的关键帧
    // 用于保存可能与F形成回环的候选帧（只要有相同的word，且不属于局部相连帧）
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    // 步骤1：找出和当前帧具有公共单词的所有关键帧
    {
        unique_lock<mutex> lock(mMutex);
        
        //遍历当前帧的所有BOW向量(每个特征点对应一个)
        //words是检测图像是否匹配的枢纽，遍历该pKF的每一个word
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {   
            //寻找与该特征点存在于同一词典节点上的其他关键帧
            //提取所有包含该word的KeyFrame
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
            
            
            //遍历这所有的关键帧
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                
                // pKFi还没有标记为pKF的候选帧
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环候选帧中与当前帧F具有共同单词最多的单词数，并以此决定阈值
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;      // 寻找最大共视单词数
    }

    int minCommonWords = maxCommonWords*0.8f;
    
    // 存储的是第一次筛选后的候选关键帧以及其重定位得分(候选关键帧与待重定位关键帧的BOW得分)
    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    // 步骤3：遍历所有闭环候选帧，挑选出共有单词数大于阈值minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    // 计算与当前待重定位帧有相同词典节点的所有关键帧的相似度得分(根据词典的相同节点数来筛选)
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        
        // 如果该关键帧与待重定位帧的相同节点数大于最小相同节点数
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            
            // si存储的是在词典中待重定位帧与查询到的关键帧之间的得分
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();
    
    //累计重定位得分(候选关键帧和当前待重定位关键帧的BOW得分) 及 最优的重定位关键帧
    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    // 步骤4：计算候选帧组得分，得到最高组得分bestAccScore，并以此决定阈值minScoreToRetain
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 具体而言：lScoreAndMatch中每一个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;     //存储关键帧的最优得分
        float accScore = bestScore;      //所有共视关键帧的累计重定位得分
        KeyFrame* pBestKF = pKFi;        //存储最优重定位关键帧
        
         //遍历这些共视关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            
            //如果它的待重定位帧与该帧不同,则跳过该帧
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;
            //如果相同  则将共视关键帧的得分也加到本次重定位得分中
            accScore+=pKF2->mRelocScore;
            
            // 如果共视关键帧的重定位得分大于当前关键帧的重定位得分
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;                    //最优重定位关键帧
                bestScore = pKF2->mRelocScore;   //则将当前关键帧的最优重定位得分赋值为共视关键帧的重定位得分
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 返回所有重定位累计得分大于0.75倍最优重定位累计得分的所有关键帧
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        
        // 如果该关键帧的累计重定位得分大于最优累计关键帧重定位得分的0.75倍
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            
            // 如果没有插入过该关键帧则插入该关键帧
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
