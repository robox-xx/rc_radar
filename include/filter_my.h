#include <iostream>
#include <math.h>
#include <vector>
#include "baseData.h"
#include "find_center.h"

class Filter
{
    public:
        Filter();
        ~Filter();

        void delete_outlier(std::vector<float>&data,std::vector<float>&theta,float radius,int k);
        void get_circle(std::vector<float>&data,std::vector<float>&theta,float deviation);
        void num_less_filter(std::vector<float>&data,const int MinNumber);
        void num_more_filter(std::vector<float>&data,const int MaxNumber);

    private:
};
