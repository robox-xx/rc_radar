#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "baseData.h"

using namespace std;

void find_circle_center(vector<float> &vec,vector<float> &xyR);
void find_continue(vector<float> &data,vector<float> &start_index, vector<float> &end_index);
void output_circle(vector<float> &start_index, vector<float> &end_index,vector<float> &data,vector< vector<float> > &disR);
void find_best_data(vector< vector<float> > &disR, float &best_num, float &best_num_x, float &best_num_y);