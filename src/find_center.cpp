#include"find_center.h"
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

using namespace std;

//输出圆心坐标以及半径
void output_circle(vector<float> &start_index, vector<float> &end_index,vector<float> &data,vector<vector<float> > &disR)
{
    vector<float> xyR(3,0);
    for(int i = 0;i<start_index.size();i++)
    {
        if(start_index[i]==0 && end_index[i]==0)
        {
            for(int k=0;k<3;k++){
                xyR[k]=0;
            }
        }
        else
        {
            vector<float> circle(DATA_NUM);
            for(int j=0;j<DATA_NUM;j++)
            {
                if(j>=start_index[i] && j<end_index[i])
                {
                    circle[j]=data[j];
                    // ROS_INFO("num = %d data=%lf",j, circle[j]);
                }
                else
                {
                    circle[j]=0;
                }
            }
            find_circle_center(circle,xyR);
            // printf("x:%f,y:%f,z:%f\n" ,xyR[0],xyR[1],xyR[2]);
            // for(int p=0;p<start_index.size();p++)
            //     for(int q=0;q<3;q++)
            //         disR[p][q]=xyR[q];
            disR.push_back(xyR);
            // for(int p=0;p<start_index.size();p++)
            //     for(int q=0;q<3;q++)
            //         ROS_INFO("%lf",disR[p][q]);
        }
    }
    //disR为当前扫描到所有圆的xyR值表
}



//最小二乘法
void find_circle_center(vector<float> &vec,vector<float> &xyR){
    int num = 0; //用于计算被过滤掉的数据个数

    float center_x =0, center_y =0, radius = 0; //定义圆心的坐标及半径 

    //定义最小二乘法所需要用到的计算值
    float sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0;
	float sum_x3 = 0, sum_y3 =0, sum_xy = 0, sum_x1y2 =0, sum_x2y1 = 0; 
    
    for (int i = 0; i <  vec.size(); ++i)
    {
        float range_first = vec[i];

        //当该点不在所需范围时，过滤掉点		
		if(range_first>10 || range_first<1e-4) 
		{
			num++;
			continue;
		}

        float angle1 =-PI/2+i * ANGLE_INCREMENT; //角度的计算，用于后面计算x和y
//    -3*PI/4 +
		float x = range_first * cos(angle1);  //计算x和y
		float y = range_first * sin(angle1);
		
		float x2 = x*x;
		float y2 = y*y;
		
		sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }
    float C, D, E, G, H;
	float a, b, c;

	int N = vec.size() - num; //得到数据个数

 	C = N * sum_x2 - sum_x * sum_x;
 	D = N * sum_xy - sum_x * sum_y;
 	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
 	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

 	xyR[0] = a / (-2);
 	xyR[1] = b / (-2);
 	xyR[2] = sqrt(a * a + b * b ) / 2;//- 4 * c
    // ROS_INFO("x:%lf,y:%lf,dis:%lf",xyR[0],xyR[1],xyR[2]);
}


//获取连续段
void find_continue(vector<float> &data,vector<float> &start_index, vector<float> &end_index){
    bool start_sign = true;//开始的标志
    data.push_back(0);
    for(int i = 0;i < DATA_NUM;i++)
    {
        // printf("data %d is %lf\n",i,data[i]);//测试用 12月4日 16.31
        if(data[i] == 0)
        {
            if (!start_sign)
            {
                //没有continue 唯一会导致start_sign = true
                //即end后又一轮重新开始
                if(data[i-1]!=0 && data[i+1]==0)
                {
                    end_index.push_back(i);
                }
                else{
                    continue;
                }
                // printf("end_i is %d\n",i);
            }
            else
            {
                continue;
            }            
        }
        else
        {
            if(start_sign)
            {
                start_index.push_back(i);
                // printf("start_i is %d\n",i);
                start_sign = false;
                continue;
            }
            else
            {
                continue;
            }            
        }
        start_sign = true;       
    }
   
    // 避免最后连续段刚好在数据段末尾
    if(start_index.size()!=end_index.size())
    {
        end_index.push_back(DATA_NUM - 1);
    }
    data.pop_back();
    
}


//寻找最近的圆柱段
void find_best_data(vector< vector<float> > &disR, float &best_num, float &best_num_x, float &best_num_y){
    
    vector<float> distance(disR.size(),0);
    
    for(int i = 0;i<disR.size();i++)
    {      
        distance.push_back(disR[i][2]);
       
    }
    for(int i = 0;i<disR.size()-1;i++)
    {   
        for(int j = 0; j<disR.size()-1-i;j++)
        {
            if(disR[j][2]>disR[j+1][2])//比较距离的远近，将最近的一根柱子放在数组的第一位
            {
                for(int k=0;k<3;k++)
                {
                    float te;
                    te = disR[j][k];
                    disR[j][k] = disR[j+1][k];
                    disR[j+1][k] = te;
                }
            }
        }
    }
    best_num_x = disR[0][0];
    best_num_y = disR[0][1];
    best_num = disR[0][2];   
}
