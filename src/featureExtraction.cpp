#include "utility.h"
#include "lio_sam/cloud_info.h"

//Zeng Shengyao: zengshengyao20@mails.ucas.edu.cn
//本代码为LIO-SAM适配Mid360版本，支持六轴和九轴IMU

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::PointCloud<PointType>::Ptr testCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction()
    {
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
        
        initializationValue();
    }

    void initializationValue()
    {
        cloudSmoothness.resize(pointNumberMax);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());
        testCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[pointNumberMax];
        cloudNeighborPicked = new int[pointNumberMax];
        cloudLabel = new int[pointNumberMax];
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfo = *msgIn; // new cloud info
        cloudHeader = msgIn->header; // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction
        ROS_DEBUG("F1");
        calculateSmoothness();
        ROS_DEBUG("F2");
        markOccludedPoints();
        ROS_DEBUG("F3");
        extractFeatures();
        ROS_DEBUG("F4");
        publishFeatureCloud();
    }

    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        //迭代四根扫描线
        for(uint i=0;i<4;i++)
        {
            for (int j = 5; j < cloudInfo.Line2point[i].index.size() - 5; j++)
            {
                float diffRange =cloudInfo.pointRange[cloudInfo.Line2point[i].index[j-4]]+ cloudInfo.pointRange[cloudInfo.Line2point[i].index[j-3]] + cloudInfo.pointRange[cloudInfo.Line2point[i].index[j-2]] + cloudInfo.pointRange[cloudInfo.Line2point[i].index[j-1]]- cloudInfo.pointRange[cloudInfo.Line2point[i].index[j]] *8
                                                    + cloudInfo.pointRange[cloudInfo.Line2point[i].index[j+1]] + cloudInfo.pointRange[cloudInfo.Line2point[i].index[j+2]] +cloudInfo.pointRange[cloudInfo.Line2point[i].index[j+3]]+cloudInfo.pointRange[cloudInfo.Line2point[i].index[j+4]] ;            

                cloudCurvature[cloudInfo.Line2point[i].index[j]] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

                cloudNeighborPicked[cloudInfo.Line2point[i].index[j]] = 0;
                cloudLabel[cloudInfo.Line2point[i].index[j]] = 0;
                // cloudSmoothness for sorting
                cloudSmoothness[cloudInfo.Line2point[i].index[j]].value = cloudCurvature[cloudInfo.Line2point[i].index[j]];
                cloudSmoothness[cloudInfo.Line2point[i].index[j]].ind = cloudInfo.Line2point[i].index[j];
            }
        }



        // pcl::copyPointCloud(*extractedCloud ,*testCloud);
        // for(int i=0; i<testCloud->points.size(); i++)
        // {
        //     // ROS_INFO("%.2f",cloudCurvature[i]);
        //     float curv=sqrt(cloudCurvature[i]);
        //     testCloud->points[i].intensity=curv>5?500:curv*100;
        // }
    }

    void markOccludedPoints()
    {
        //迭代四根扫描线
        for(uint k=0;k<4;k++)
        {
            int cloudSize = cloudInfo.Line2point[k].index.size();
            // mark occluded points and parallel beam points
            for (int i = 5; i < cloudSize - 6; ++i)
            {
                // occluded points
                float depth1 = cloudInfo.pointRange[cloudInfo.Line2point[k].index[i]];
                float depth2 = cloudInfo.pointRange[cloudInfo.Line2point[k].index[i+1]];

                // 10 pixel diff in range image
                if (depth1 - depth2 > 3){
                    cloudNeighborPicked[cloudInfo.Line2point[k].index[i-1]] = 1;
                    cloudNeighborPicked[cloudInfo.Line2point[k].index[i]] = 1;
                }else if (depth2 - depth1 > 3){
                    cloudNeighborPicked[cloudInfo.Line2point[k].index[i+1]] = 1;
                }

                // parallel beam
                float diff1 = float(cloudInfo.pointRange[cloudInfo.Line2point[k].index[i-1]] - cloudInfo.pointRange[cloudInfo.Line2point[k].index[i]]);
                float diff2 = float(cloudInfo.pointRange[cloudInfo.Line2point[k].index[i]] - cloudInfo.pointRange[cloudInfo.Line2point[k].index[i+1]]);

                if (diff1 > 0.2 * cloudInfo.pointRange[cloudInfo.Line2point[k].index[i]] && diff2 >0.2 * cloudInfo.pointRange[cloudInfo.Line2point[k].index[i]])
                    cloudNeighborPicked[cloudInfo.Line2point[k].index[i]] = 1;
                if ((-diff1) > 0.2 * cloudInfo.pointRange[cloudInfo.Line2point[k].index[i]] && (-diff2) >0.2 * cloudInfo.pointRange[cloudInfo.Line2point[k].index[i]])
                    cloudNeighborPicked[cloudInfo.Line2point[k].index[i]] = 1;
            }
        }

            // pcl::copyPointCloud(*extractedCloud ,*testCloud);
            // for(int i=0; i<extractedCloud->size()-1; i++)
            // {
            //     testCloud->points[i].intensity= cloudNeighborPicked[i ]*30;
            // }
    }

    void extractFeatures()
    {
        ROS_DEBUG("F3.0");
        uint cloudSize = extractedCloud->points.size();
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        //一块一块的选角点
        //暂定每200个点选15个角点
        const uint div = 200;
        const uint pickNum = 15;
        for (uint sp = 5; sp+div<cloudSize-6; sp+=div)
        {
            uint ep=std::min(sp+div,cloudSize-6);
            std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() +ep, by_value());

            int largestPickedNum = 0;
            for (int k =ep; k >= sp; k--)
            {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= pickNum)
                    {
                        cloudLabel[ind] = 1;
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    if (ind < 6 || ind > cloudSize - 7)
                        continue;
                    for (int l = 1; l <= 4; l++)
                    {
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -4; l--)
                    {
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            for (int k = sp; k <ep; k++)
            {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                {

                    cloudLabel[ind] = -1;
                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 2; l++)
                    {
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -2; l--)
                    {

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            for (int k = sp; k < ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }
        }

        surfaceCloudScanDS->clear();
        downSizeFilter.setInputCloud(surfaceCloudScan);
        downSizeFilter.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
    }

    void freeCloudInfoMemory()
    {

    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);

        // cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  testCloud,  cloudHeader.stamp, lidarFrame);

    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}