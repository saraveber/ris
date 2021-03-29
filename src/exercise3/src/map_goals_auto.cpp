#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <queue>
#include <bit>
#include <bitset>
#include <cstdint>
#include <initializer_list>
#include <iostream>

#define N 20



using namespace std;
using namespace cv;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::TransformStamped map_transform;
ros::Subscriber map_sub;

Mat cv_map,dst_norm,dst_norm_scaled,poskus,eroded;

vector<Point> b;
vector<int>vrstni_red;
int matrika_razdalij [N][N];
pair<int,int> previous [(1 << N)][N] ;
int dp [(1 << N)][N] ;

int tresh = 100;
float map_resolution = 0;
int erosion_elem = 2;
int erosion_size = 2;
int dilation_elem = 2;
int dilation_size = 2;
int const max_elem = 2;
int const max_kernel_size = 21;


class QItem {
public:
    int row;
    int col;
    int dist;
    QItem(int x, int y, int w)
        : row(x), col(y), dist(w)
    {
    }
};


//Preverim ali je točka dostopna, ker točka ne more biti čisto ob robu mapo erodam (razširim rob v vse smeri)
//tako ne bom izbirala točk ki so preblizu
bool check_if_point_is_on_map(int x,int y){
    int erosion_type = MORPH_RECT;
    Mat element = getStructuringElement( erosion_type,
                       Size( 2*erosion_size + 5, 2*erosion_size+5 ),
                       Point( erosion_size, erosion_size ));
    erode( cv_map,eroded, element );
    if ((int)eroded.at<unsigned char>(x, y) == 255){return true;}
    return false;
}
//v naboru dosedanjih točk preverim ali je kakšna točka ki je zelo blizu, tako se izognem prevelikemu številu točk
bool is_new (int x, int y){
    for (int i = 0; i < b.size();i++){
        int x1 = b[i].x;
        int y1 = b[i].y;
        float dist = sqrt(pow(x-x1,2)+pow(y-y1,2));
        if (dist < 17){ 
                    
            return false;
        }
    }
    return true;
}
//funkcija pripravi mapo iz katerih extractiram točke
Mat prepere_img_for_point_extraction(Mat img){
    int erosion_type = MORPH_RECT;
    Mat element = getStructuringElement( erosion_type,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );
    
    Mat a,b, im1,im2,im3;
    b = img;

    Mat m = 255-Mat::zeros( img.size(), CV_32FC1 );

    erode( b,a, element );
    erode( a,b, element );
    dilate( b, b, element );
    absdiff(b,a,im1);
    im1 = 255-im1;


    for(int i = 0; i < 20; i++){
        erode( b,a, element );
        erode( a,b, element );
        dilate( b, b, element );
        absdiff(b,a,im2);
        multiply(im1,255-im2,im1); 
    }

    //erode( im1,im1, element );

    return im1;
}
// funkcija vzame naprej pripravljen Mat img ter najde vogale 
// točke pusha v vektor nabora točk
// preveri ali so točke dodtopne in niso preblizu roba ter preblizu drugih točk
// po končani tej funkciji je vektor točk poln
Mat corner_harris_my_function( Mat img ){
    int blockSize = 2;
    int apertureSize = 0.5;
    double k = 0.04;

    Mat dst = Mat::zeros( img.size(), CV_32FC1 );
    cornerHarris( img, dst, blockSize, apertureSize, k );

    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    for( int i = 0; i < dst_norm.rows ; i++ ){
        for( int j = 0; j < dst_norm.cols; j++ ){
            if( (int) dst_norm.at<float>(i,j) > tresh && check_if_point_is_on_map(i,j) && is_new(i,j)){        
                circle(img, Point(j,i), 5,  Scalar(0), 1, 8, 0 );
                Point tocka = Point(i,j);
                b.push_back(tocka);
            }
        }
    }
    return img;
}
// Funkcija napolni matrikorazdalij, ki nam pove razdalije med izbranimi točkami
// uporabila sem enostaven algoritem bfs
// po klicu te funkcije poznamo razdalije med točkami
void make_distance_matrix(){
    
    memset(matrika_razdalij,0,sizeof(matrika_razdalij));
    Mat grid;
    Mat visited;
    eroded.copyTo(grid);


    for (int zacetni_index = 0; zacetni_index < b.size(); zacetni_index++){

        visited = Mat::zeros( grid.size(), CV_8U);
        visited = grid > 1;

        int x = b[zacetni_index].x;
        int y = b[zacetni_index].y;
        QItem source(x, y, 0);
        std::queue<QItem> q;
        q.push(source);
        visited.at<bool>(source.row,source.col) = true;

        int num_of_found = 0;
        while (!q.empty()){
            QItem p = q.front();
            q.pop();

            //cout << visited.at<bool>(p.row+1,p.col);

            for (int i = 0; i < b.size(); i++){
                if (p.row == b[i].x && p.col == b[i].y){
                    matrika_razdalij[zacetni_index][i] = p.dist;
                    //matrika_razdalij[i][zacetni_index] = p.dist;
                    num_of_found ++;
                    break;
                } 
            }
            //cout << num_of_found;
            if (num_of_found == b.size()){

                break;
            }
            if (visited.at<bool>(p.row-1,p.col) == 255) {
                q.push(QItem(p.row - 1, p.col, p.dist + 1));
                visited.at<bool>(p.row-1,p.col) = 0;
            }
            if (visited.at<bool>(p.row+1,p.col) == 255) {
                q.push(QItem(p.row + 1, p.col, p.dist + 1));
                visited.at<bool>(p.row+1,p.col) = 0;
            }
            if (visited.at<bool>(p.row,p.col-1) == 255) {
                q.push(QItem(p.row, p.col-1, p.dist + 1));
                visited.at<bool>(p.row,p.col-1) = 0;
            }
            if (visited.at<bool>(p.row,p.col+1) == 255) {
                q.push(QItem(p.row , p.col+1, p.dist + 1));
                visited.at<bool>(p.row,p.col+1) = 0;
            }
            if (visited.at<bool>(p.row-1,p.col-1) == 255) {
                q.push(QItem(p.row - 1, p.col-1, p.dist + 1));
                visited.at<bool>(p.row-1,p.col-1) = 0;
            }
            if (visited.at<bool>(p.row+1,p.col+1) == 255) {
                q.push(QItem(p.row + 1, p.col+1, p.dist + 1));
                visited.at<bool>(p.row+1,p.col+1) = 0;
            }
            if (visited.at<bool>(p.row+1,p.col-1) == 255) {
                q.push(QItem(p.row+1, p.col-1, p.dist + 1));
                visited.at<bool>(p.row+1,p.col-1) = 0;
            }
            if (visited.at<bool>(p.row-1,p.col+1) == 255) {
                q.push(QItem(p.row -1, p.col+1, p.dist + 1));
                visited.at<bool>(p.row-1,p.col+1) = 0;
            }
        }
    }
}
// dp ki najde najkrajšo pot z uporabo bitmaska
// iz previous dobimo pot
int f(int bitmask, int tren){
    //cout << bitmask << "  " << tren  << "  " <<__builtin_popcount(bitmask)<< endl;
    if(dp[bitmask][tren] != -1){
        return dp[bitmask][tren]; 
    }
    if (__builtin_popcount(bitmask) == 1){
        return 0;
    }
    for(int i = 0; i < b.size();i++){
        if (bitmask >> i & 1 && i != tren){
            //cout << bitmask << "  " << tren  << "  " << i << endl;
            int dol = f(bitmask ^ (1 << tren),i)+matrika_razdalij[i][tren];
            if (dp[bitmask][tren] > dol || dp[bitmask][tren] == -1){
                dp[bitmask][tren] = dol;
               
                previous[bitmask][tren].first = bitmask ^ (1<< tren);
                previous[bitmask][tren].second = i;
            }

        }
    }
    return dp[bitmask][tren];
}
//funkcija pošlje robota po točkah v pravem vrstnem redu ne ozira se na karkolidrugega
void go_to_locations(){

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    

    for (int i = 0; i < b.size(); i++){
        int index = vrstni_red[i];
        int x = b[index].y;
        int y = b[index].x;

        geometry_msgs::Point pt;
        geometry_msgs::Point transformed_pt;

        pt.x = (float)x * map_resolution;
        pt.y = (float)(cv_map.rows - y) * map_resolution;
        pt.z = 0.0;
        
        tf2::doTransform(pt, transformed_pt, map_transform);

        move_base_msgs::MoveBaseGoal goal;
        
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.orientation.w = 20;
        goal.target_pose.pose.position.x = transformed_pt.x;
        goal.target_pose.pose.position.y = transformed_pt.y;

        ROS_INFO("Sending goal %d",i);
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You arived at destination: %d.",(i+1));
        else
            ROS_INFO("UPSI, you are stuck.");

        move_base_msgs::MoveBaseGoal goal1;

        goal1.target_pose.header.frame_id = "base_link";
        goal1.target_pose.header.stamp = ros::Time::now();
        goal1.target_pose.pose.position.y = 0;
        goal1.target_pose.pose.orientation.z = 20;
        goal1.target_pose.pose.orientation.w = 1;

        
        ROS_INFO("TURNING ...");
        ac.sendGoal(goal1);

        ac.waitForResult();

        move_base_msgs::MoveBaseGoal goal2;
        

        goal2 = goal1;
        goal2.target_pose.header.stamp = ros::Time::now();
        

        
        ROS_INFO("TURNING ...");
        
        ac.sendGoal(goal2);

        ac.waitForResult();
                
        

    }
}

//we get points> get path
void define_path(){

    cv_map.copyTo(poskus);
    //prepere
    poskus = prepere_img_for_point_extraction(poskus);
    //extract points
    poskus = corner_harris_my_function(poskus);

    //distance matrix
    make_distance_matrix();
    //get vrstni red
    memset(dp,-1,sizeof(dp));
    int k = f(pow(2,b.size())-1,0);

    int bitmask = pow(2,b.size())-1;
    int tren = 0;


    while (__builtin_popcount(bitmask) > 1){
        
        vrstni_red.push_back(tren);
        int newbitmask = previous[bitmask][tren].first;
        int newtren = previous[bitmask][tren].second;
        bitmask = newbitmask;
        tren = newtren;
    }
    vrstni_red.push_back(tren);
    multiply(cv_map,poskus,poskus); 

}
//prebere in napolni mapo
void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
    map_transform.transform.translation.x = msg_map->info.origin.position.x;
    map_transform.transform.translation.y = msg_map->info.origin.position.y;
    map_transform.transform.translation.z = msg_map->info.origin.position.z;

    map_transform.transform.rotation = msg_map->info.origin.orientation;

    //tf2::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }
    define_path();
    go_to_locations();
}
//main
int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goals_auto");
    ros::NodeHandle n;  
    map_sub = n.subscribe("map", 10, &mapCallback);

    namedWindow("Map");
    while(ros::ok()) {

        //if (!cv_map.empty()) imshow("Map",cv_map );
        //if (!eroded.empty()) imshow("Map",eroded );
        if (!poskus.empty()) imshow("Map",poskus);
        //if (!visited.empty()) imshow("Map",visited);
        waitKey(30);
        ros::spinOnce();
    }
    return 0;

}


