#include "filter_my.h"


//除去离群点
void Filter::delete_outlier(std::vector<float>&data,std::vector<float>&theta,float radius,int k)
{
    for(int i =0;i<DATA_NUM;i++)
    {
        if (data[i] ==0)
        {
            continue;
        }

        int cnt =0;
        for(int j =0;j<DATA_NUM;j++)
        {
            if(i==j)
            {
                continue;
            }
            if (sqrt(data[i] * data[i] + data[j] * data[j]- 2 * fabs(cos(theta[i] - theta[j]))* data[i] * data[j]) < radius)
            {
                cnt ++;
            }              
        }
        if (cnt <= k)
        {
            data[i] = 0;
        }
    }   
}

//分离是圆弧的连续段数据
void Filter::get_circle(std::vector<float>&data,std::vector<float>&theta,float deviation)
{
    std::vector<float>start_index;
    std::vector<float>end_index;

    find_continue(data,start_index,end_index);
    for(int i=0;i<start_index.size();i++)
    {
        int middle = static_cast<int>((start_index[i] + end_index[i]-1)/ 2);
        float x0 = (data[middle] + R) * cos(theta[middle]);
        float y0 = (data[middle] + R) * sin(theta[middle]);
        for(int j =start_index[i];j<end_index[i];j++)
        {
            float x1 = data[j] * cos(theta[j]);
            float y1 = data[j] * sin(theta[j]);
            if ((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) - R * R > deviation)
            {
                data[j] = 0;
            }                
        }
    }
}

//滤掉点数过少的连续段
void Filter::num_less_filter(std::vector<float>&data,const int MinNumber)
{
    std::vector<float>start_index;
    std::vector<float>end_index;

    find_continue(data,start_index,end_index);

    for(int i =0;i<start_index.size();i++)
    {
        if(end_index[i] - start_index[i] - 1 < MinNumber)
        {
            for(int j =start_index[i];j<end_index[i];j++)
            {
                data[j] =0;
            }
        }
    }
}

//滤掉点数过多的连续段
void Filter::num_more_filter(std::vector<float>&data,const int MaxNumber)
{
    std::vector<float>start_index;
    std::vector<float>end_index;

    find_continue(data,start_index,end_index);

    for(int i =0;i<start_index.size();i++)
    {
        if(end_index[i] - start_index[i] - 1 > MaxNumber)
        {
            for(int j =start_index[i];j<end_index[i];j++)
            {
                data[j] =0;
            }
        }
    }
}


Filter::Filter()
{

}
Filter::~Filter()
{

}