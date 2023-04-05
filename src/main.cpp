#include<iostream>
// ros
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>



#include "lidar.h"
#include "filter_my.h"


using namespace std;

ros::Publisher ladar_pub;
ros::Publisher whereToshoot;
ros::Publisher whether;

Lidar ladar;
Filter filter;

// std_msgs::Bool yes_or_no;


void LidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //获取雷达数据
    ladar.getData(scan);
    
    //对雷达数据进行滤波
    //1.去除离群点
    filter.delete_outlier(ladar.nowData,ladar.THETA, 0.15,6);
    
    //2.分离是圆弧的连续段数据
    filter.get_circle(ladar.nowData, ladar.THETA, 1);
    
    //3.滤掉点数过少的连续段
    filter.num_less_filter(ladar.nowData,5);
    
    //4.滤掉点数过多的连续段
    filter.num_more_filter(ladar.nowData,50);

    //5.获得连续段
    std::vector<float> start_index; //连续段的起始坐标
    std::vector<float> end_index;   //连续段的终止坐标
    find_continue(ladar.nowData, start_index, end_index);
    //测试用12月4日16.41
    // ROS_INFO("start:%ld",start_index.size());
    // ROS_INFO("end:%ld",end_index.size());
    
    if(start_index.size()!=0)
    {
        // for(int i=0;i<=start_index.size()-1;i++)
        // {
        //     ROS_INFO("start:%lf,end:%lf",start_index[i],end_index[i]);
        // }
    //6.拟合圆心
        vector< vector<float> > disR;//(start_index.size(),vector<float>(3))
        output_circle(start_index, end_index, ladar.nowData, disR);
        int num = end_index.size();
        for(int i=0;i<num;i++)
        {
            ROS_INFO("第%d个圆柱的,x值为:%f,y值为:%f,圆心与机器人坐标原点距离为为:%f",i+1,disR[i][0],disR[i][1],disR[i][2]); 
        }
        
    //7.寻找最好的射环点
        std_msgs::Float64MultiArray the_best_way;
        float best_num=0;
        float best_num_x=0;
        float best_num_y=0;
        
        find_best_data(disR, best_num, best_num_x, best_num_y);
        ROS_INFO("最好的圆在：  x:%f,y:%f",best_num_x, best_num_y);
        best_num_x = best_num_x -0.8*best_num_x/sqrt(best_num_x*best_num_x+best_num_y*best_num_y);
        best_num_y = best_num_y -0.8*best_num_y/sqrt(best_num_x*best_num_x+best_num_y*best_num_y);
        best_num = abs(best_num - sqrt(0.8*0.8+0.8*0.8));
        
        the_best_way.data.push_back(best_num_x);
        the_best_way.data.push_back(best_num_y);
        the_best_way.data.push_back(0);
        // ROS_INFO("我发出最好的射击位置是  x:%f,y:%f,z:%f",best_num_x, best_num_y, best_num);
        whereToshoot.publish(the_best_way);
        ROS_INFO("我发出最好的雷达射击位置是x:%f,y:%f",the_best_way.data[0],the_best_way.data[1]);
        the_best_way.data.clear();
        
    }
    else{
        printf("no circle!!!!\n");
    }
    
    //发布点云，使其可视化
    ladar.prePublish(scan);
    ladar_pub.publish(ladar.result);

}

int main(int argc, char *argv[])
{
    
    setlocale(LC_ALL, "");
    ros::init(argc,argv,"ladar");

    ros::NodeHandle nh;

    ros::Rate loop_rate(5);
    // serialInit();
    ros::Subscriber subscriber = nh.subscribe("/scan",1 ,LidarCallback);
   
    while(ros::ok())
    {
        ros::spinOnce();
        //发布雷达当前数据  
        ladar_pub = nh.advertise<sensor_msgs::LaserScan>("/now", 10);
        //发布最好射击位置的数据，第一个数据为x,第二个数据为y，第三个数据是距离
        whereToshoot = nh.advertise<std_msgs::Float64MultiArray>("/radar/shootPosition",10);
        // whether =nh.advertise<std_msgs::Bool>("whether",10);
        // ladar_pub = nh.advertise<sensor_msgs::PointCloud2>("/now", 10);
        loop_rate.sleep();
    }    
    ros::spin();
    return 0;
}


