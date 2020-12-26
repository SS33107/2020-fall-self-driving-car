#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
using namespace ros;
using namespace std;

class localization{
  public:
    localization(ros::NodeHandle nh);
    void lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void gps_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void update_localize();
    Eigen::Matrix4f transfer_cloud();

  private:
    pcl::PointCloud<pcl::PointXYZI> map_cloud;
    pcl::PointCloud<pcl::PointXYZI> car_cloud;

    ros::Subscriber sub_lidar;
    ros::Subscriber sub_gps;
    ros::Publisher pub_lidar_match;
    ros::Publisher pub_map;
    ros::Publisher pub_predict;
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    sensor_msgs::PointCloud2 car_cloud_ROS, map_ROS;
    nav_msgs::Odometry localize_result;
    double gps_x, gps_y, gps_z;
    int count=1;
    string map_path ="/home/ss/catkin_ws/src/localization_309605004/map/map.pcd";
    string csv_path ="/home/ss/catkin_ws/src/localization_309605004/submit_1.csv";
    string target_frame = "base_link";
    string source_frame = "velodyne";
    ofstream outfile;
    Eigen::Matrix4f init_guess,trans;
};

localization::localization(ros::NodeHandle nh)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ori(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile (map_path, *map_cloud_ori);
  std::cout << "finish load map"<< std::endl;

  pcl::PassThrough<pcl::PointXYZI> passMx;
  passMx.setInputCloud (map_cloud_ori);
  passMx.setFilterFieldName ("x");
  passMx.setFilterLimits (-400, 0);
  passMx.filter (*map_cloud_ori);

  pcl::PassThrough<pcl::PointXYZI> passMy;
  passMy.setInputCloud (map_cloud_ori);
  passMy.setFilterFieldName ("y");
  passMy.setFilterLimits (150,400);
  passMy.filter (*map_cloud_ori);


  pcl::PassThrough<pcl::PointXYZI> passMz;
  passMz.setInputCloud (map_cloud_ori);
  passMz.setFilterFieldName ("z");
  passMz.setFilterLimits (-12, 5);
  passMz.filter (map_cloud);

  outfile.open(csv_path,ios::app);
  outfile<<"id,x,y,z,yaw,pitch,roll,\n";
  outfile.close();

  pcl::toROSMsg(map_cloud, map_ROS);


  pub_lidar_match = nh.advertise<sensor_msgs::PointCloud2>("/fin_cloud", 1);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
  pub_predict = nh.advertise<nav_msgs::Odometry>("/lidar_match", 1);
  sub_lidar = nh.subscribe("lidar_points", 10, &localization::lidar_Callback, this);
  sub_gps = nh.subscribe("gps", 10, &localization::gps_Callback, this);
}

Eigen::Matrix4f localization::transfer_cloud(){
  tf::StampedTransform transform;
  Eigen::Matrix4f trans;
  try{
    listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return trans;
  }
    Eigen::Quaternionf q(transform.getRotation().getW(), \
    transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
  Eigen::Matrix3f mat = q.toRotationMatrix();
  trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
      mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
      mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
      0, 0, 0, 1;
  return trans;
}

void localization::lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{ 
  pcl::PointCloud<pcl::PointXYZI>::Ptr car_cloud_ori(new pcl::PointCloud<pcl::PointXYZI>);
  std::cout << "frame: "<<count<<std::endl;
  pcl::fromROSMsg(*msg,*car_cloud_ori);
  
  trans = transfer_cloud();
  
  transformPointCloud(*car_cloud_ori, *car_cloud_ori, trans);
  
   //PassThrough filter
   pcl::PassThrough<pcl::PointXYZI> passCx;
   passCx.setFilterFieldName ("x");
   pcl::PassThrough<pcl::PointXYZI> passCy;
   passCy.setFilterFieldName ("y");
   pcl::PassThrough<pcl::PointXYZI> passCz;
   passCz.setFilterFieldName ("z");
  if(count<130) {
   passCx.setFilterLimits (-70,70);
   passCy.setFilterLimits (-10, 12);
   passCz.setFilterLimits (1.5, 15);

  }
  
  else{
   passCx.setFilterLimits (-35,70);
   passCy.setFilterLimits (-15, 12);
   passCz.setFilterLimits (1.5, 15);
   }

   passCx.setInputCloud (car_cloud_ori);
   passCx.filter (*car_cloud_ori);

   passCy.setInputCloud (car_cloud_ori);
   passCy.filter (*car_cloud_ori);

   passCz.setInputCloud (car_cloud_ori);
   passCz.filter (car_cloud);

   update_localize();
}

void localization::gps_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  gps_x = msg->point.x;
  gps_y = msg->point.y;
  gps_z = msg->point.z;
  if(count < 2){
    double theta = 140*3.14159/180;
    init_guess <<     cos(theta), -sin(theta),  0.0, gps_x,
                      sin(theta),  cos(theta),  0.0, gps_y,
                             0.0,         0.0,  1.0, gps_z,
                             0.0,         0.0,  0.0,   1.0; 
  }
}

void localization::update_localize()
{ 
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  *input_cloud = car_cloud;
  *target_cloud = map_cloud;

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(input_cloud);
  icp.setInputTarget(target_cloud);
  icp.setMaxCorrespondenceDistance (0.8);
  icp.setMaximumIterations (4000);
  icp.setTransformationEpsilon (1e-9);
  icp.setEuclideanFitnessEpsilon (1e-4);
  icp.setRANSACOutlierRejectionThreshold(0.05);

      
  pcl::PointCloud<pcl::PointXYZI>::Ptr Final_cloud (new pcl::PointCloud<pcl::PointXYZI>);   

  icp.align(*Final_cloud, init_guess);

  std::cerr << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  //std::cerr << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation ();
  init_guess = icp.getFinalTransformation ();

  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(init_guess(0,0)), static_cast<double>(init_guess(0,1)), static_cast<double>(init_guess(0,2)), 
                    static_cast<double>(init_guess(1,0)), static_cast<double>(init_guess(1,1)), static_cast<double>(init_guess(1,2)), 
                    static_cast<double>(init_guess(2,0)), static_cast<double>(init_guess(2,1)), static_cast<double>(init_guess(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  geometry_msgs::Quaternion quat_msg;
  quaternionTFToMsg(tfqt , quat_msg);

  tf::Matrix3x3 M(tf3d);
  double roll , pitch , yaw;
  M.getRPY(roll , pitch , yaw);

  pcl::toROSMsg(*Final_cloud, car_cloud_ROS);
  car_cloud_ROS.header.frame_id = "world";
  car_cloud_ROS.header.stamp = ros::Time::now();
  pub_lidar_match.publish(car_cloud_ROS);
  map_ROS.header.frame_id = "world";
  map_ROS.header.stamp = ros::Time::now();
  pub_map.publish(map_ROS);


  localize_result.header.frame_id = "world";
  localize_result.child_frame_id = target_frame;
  localize_result.pose.pose.position.x = transformationMatrix(0, 3);
  localize_result.pose.pose.position.y = transformationMatrix(1, 3);
  localize_result.pose.pose.position.z = transformationMatrix(2, 3);
  localize_result.pose.pose.orientation = quat_msg;
  pub_predict.publish(localize_result);
  
  outfile.open(csv_path,ios::app);
  outfile<<count<<','<<localize_result.pose.pose.position.x<<','<<localize_result.pose.pose.position.y<<','<<localize_result.pose.pose.position.z<<','<<yaw<<','<<pitch<<','<<roll<<'\n';
  outfile.close();
  count=count+1;
}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "localization");
  ros::NodeHandle nh;
  localization pc(nh);
  ros::spin();
}
