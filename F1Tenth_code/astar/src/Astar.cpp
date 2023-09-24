//
// Created by lihao on 19-7-9.
//

#include "Astar.h"
#include <ros/ros.h>
#include <cmath>

namespace pathplanning{

void Astar::InitAstar(Mat& _Map, AstarConfig _config)
{
    Mat Mask;
    InitAstar(_Map, Mask, _config);
}

void Astar::InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config)
{
    signed char neighbor8[8][2] = {
            {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1},
            {-1, 0}, {-1, -1}, {0, -1} 
    };

    Map = _Map;
    config = _config;
    neighbor = Mat(8, 2, CV_8S, neighbor8).clone();

    MapProcess(Mask);
}

void Astar::PathPlanning(Point _startPoint, vector<double>& _startPointDirection, Point _targetPoint, vector<Point>& path)
{
    // Get variables
    startPoint = _startPoint;
    startPointDirection = _startPointDirection;
    goalPoint = _targetPoint;

    // Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode, path);
}

void Astar::DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask, Scalar color,
        int thickness, Scalar maskcolor)
{
    if(path.empty())
    {
        cout << "Path is empty!" << endl;
        return;
    }
    _Map.setTo(maskcolor, Mask);
    for(auto it:path)
    {
        rectangle(_Map, it, it, color, thickness);
    }
}

void Astar::MapProcess(Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    if(config.OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, config.OccupyThresh, 255, cv::THRESH_BINARY);
    }

    // Inflate
    Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    LabelMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                LabelMap.at<uchar>(y, x) = free;
            }
        }
    }
}

Node* Astar::FindPath()
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(pair<int, Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        CloseDict[index] = CurNode;
        OpenDict.erase(index);

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        // Determine whether arrive the target point
        if(curX == goalPoint.x && curY == goalPoint.y)
        {
            return CurNode; // Find a valid path
        }

        int isBackward[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        double arr_yaw[8] = {0.0,};
        // Traversal the neighborhood
        for(int k = 0; k < neighbor.rows;k++)
        {
            int y = curY + neighbor.at<signed char>(k, 0);
            int x = curX + neighbor.at<signed char>(k, 1);
            
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }

            if(_LabelMap.at<uchar>(y, x) == inCloseList)
            {
                Node* pastnode = CloseDict[point2index(Point(x, y))];
                pastnode->G = 987654;
                pastnode->F = pastnode->G + pastnode->H;

                if((CurNode->prev_k + 3) % 8 == k || (CurNode->prev_k + 5) % 8 == k){
                        cosineSimilarity = -1/sqrt(2);
                    }

                if((CurNode->prev_k + 2) % 8 == k || (CurNode->prev_k + 6) % 8 == k){
                    cosineSimilarity = 0;
                }

                if((CurNode->prev_k + 1) % 8 == k || (CurNode->prev_k + 7) % 8 == k){
                    cosineSimilarity = 1/sqrt(2);
                }


                if(CurNode->prev_k == k){
                    cosineSimilarity = 1;
                }

            }

            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor.at<signed char>(k, 0)) + abs(neighbor.at<signed char>(k, 1));
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
                    continue;

                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }

                int directG = 0;
                double cosineSimilarity = 0.0;
                Direct curDirectVector;
                if(!CurNode->parent){
                    double q_x = startPointDirection[0];
                    double q_y = startPointDirection[1];
                    double q_z = startPointDirection[2];
                    double q_w = startPointDirection[3];

                    double siny_cosp = 2 * (q_w * q_z + q_x * q_y);
                    double cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z);
                    double yaw = atan2(siny_cosp, cosy_cosp);

                    curDirectVector.x = cos(yaw);
                    curDirectVector.y = sin(yaw);

                    Direct curToNextVector;
                    curToNextVector.x = x - curX;
                    curToNextVector.y = y - curY;
                    
                    cosineSimilarity = (curDirectVector.x * curToNextVector.x + curDirectVector.y * curToNextVector.y);

                }

                else{

                    if((CurNode->prev_k + 3) % 8 == k || (CurNode->prev_k + 5) % 8 == k){
                        cosineSimilarity = -1/sqrt(2);
                    }

                    if((CurNode->prev_k + 2) % 8 == k || (CurNode->prev_k + 6) % 8 == k){
                        cosineSimilarity = 0;
                    }

                    if((CurNode->prev_k + 1) % 8 == k || (CurNode->prev_k + 7) % 8 == k){
                        cosineSimilarity = 1/sqrt(2);
                    }

                    if(CurNode->prev_k == k){
                        cosineSimilarity = 1;
                    }

                    ROS_INFO("prev_k : %d \t k : %d", CurNode->prev_k, k);

                }
                
                arr_yaw[k] = cosineSimilarity;
                if(cosineSimilarity < 0){
                    directG = 987654321;
                }

                G = CurNode->G + addG + directG;
                if(config.Euclidean)
                {
                    int dist2 = (x - goalPoint.x) * (x - goalPoint.x) + (y - goalPoint.y) * (y - goalPoint.y);
                    H = round(10 * sqrt(dist2)) ;
                }
                else
                {
                    H = 10 * (abs(x - goalPoint.x) + abs(y - goalPoint.y));
                }
                F = G + H;


                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    node->prev_k = k;
                    OpenList.push(pair<int, Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    int index = point2index(Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }

        }

        if(!CurNode->parent){
            ROS_INFO("%10f %10f %10f", arr_yaw[0], arr_yaw[1], arr_yaw[2]);
            ROS_INFO("%10f            %10f", arr_yaw[7], arr_yaw[3]);
            ROS_INFO("%10f %10f %10f\n",arr_yaw[6], arr_yaw[5], arr_yaw[4]);
        
        }
        else{
            ROS_INFO("%10f %10f %10f", arr_yaw[0], arr_yaw[1], arr_yaw[2]);
            ROS_INFO("%10f            %10f", arr_yaw[7], arr_yaw[3]);
            ROS_INFO("%10f %10f %10f\n",arr_yaw[6], arr_yaw[5], arr_yaw[4]);
        }
    }

    return NULL; // Can not find a valid path
}

void Astar::GetPath(Node* TailNode, vector<Point>& path)
{
    PathList.clear();
    path.clear();

    int final_cost = 0;
    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        final_cost += CurNode->F;
        CurNode = CurNode->parent;
    }

    ROS_INFO("PathList size : %d", PathList.size());
    ROS_INFO("final cost : %d", final_cost);

    // Save path to vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(PathList.back()->point);
        PathList.pop_back();
    }

    // Release memory
    while(OpenList.size()) {
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}

}