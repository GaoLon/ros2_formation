#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
// #include <pcl/search/impl/organized.hpp>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <eigen3/Eigen/Eigen>
#include <random>

using namespace std;

pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _local_map_pub, _all_map_pub, click_map_pub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _pub_rate;
double _min_dist;
int fix_obs_type_;

rclcpp::Parameter _obs_num_;
rclcpp::Parameter _x_size_, _y_size_, _z_size_;
rclcpp::Parameter _w_l_, _w_h_, _h_l_, _h_h_;
rclcpp::Parameter _resolution_, _pub_rate_;
rclcpp::Parameter _min_dist_;
rclcpp::Parameter _fix_obs_type_;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;

rclcpp::Parameter _circle_num_;
rclcpp::Parameter _radius_l_, _radius_h_, _z_l_, _z_h_;
rclcpp::Parameter _theta_;

uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::msg::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::msg::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void GenerateWall(double x_l, double x_h, 
                  double y_l, double y_h, 
                  double z_l, double z_h, 
                  pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  int x_num, y_num, z_num;
  x_num = ceil((x_h - x_l)/_resolution);
  y_num = ceil((y_h - y_l)/_resolution);
  z_num = ceil((z_h - z_l)/_resolution);
  pcl::PointXYZ pt;
  for (int i=0; i<x_num; i++)
    for (int j=0; j<y_num; j++)
      for (int k=0; k<z_num; k++){
        pt.x = x_l + i * _resolution;
        pt.y = y_l + j * _resolution;
        pt.z = z_l + k * _resolution;
        cloudMap.push_back(pt);
      }
}

void RandomMapGenerate(rclcpp::Node::SharedPtr node)
{
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++)
  {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++)
        {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  RCLCPP_WARN(node->get_logger(), "Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void RandomMapGenerateCylinder(rclcpp::Node::SharedPtr node)
{
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num && rclcpp::ok(); i++)
  {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w * inf) / _resolution);
    double radius = (w * inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -10; t < heiNum; t++)
        {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  RCLCPP_WARN(node->get_logger(), "Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void FixMapGenerate(rclcpp::Node::SharedPtr node){
 
  switch (fix_obs_type_)
  {
    case 0: // NONE_FIX
    {
      break;
    }
    
    case 1: // WALL_WITH_HOLE
    { 
      double x_l, x_h, y_l, y_h, z_l, z_h;
      x_l = -3.2;
      x_h = 3.2;
      y_l = - _y_size / 2;
      y_h = -1.6;
      z_l = -1.0;
      z_h = _z_size;
      GenerateWall(x_l, x_h, y_l, y_h, z_l, z_h, cloudMap);
      x_l = -3.2;
      x_h = 3.2;
      y_l = 1.6;
      y_h = _y_size / 2;
      z_l = -1.0;
      z_h = _z_size;
      GenerateWall(x_l, x_h, y_l, y_h, z_l, z_h, cloudMap);
      break;
    }

  }

  RCLCPP_WARN(node->get_logger(), "Finished generate fixed map ");

}

void clickCallback(const geometry_msgs::msg::PoseStamped &msg)
{
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);
  double h;
  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
    {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++)
      {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_->publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

int i = 0;
void pubPoints(rclcpp::Node::SharedPtr node)
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    if (_map_ok)
      break;
  }
  
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub->publish(globalMap_pcd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("map_generator", \
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  // _local_map_pub = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 1);

  // click_map_pub_ = n->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_render_node/local_map", 1);

  n->get_parameter_or("map/x_size", _x_size_, rclcpp::Parameter("map/x_size", 50.0));
  n->get_parameter_or("map/y_size", _y_size_, rclcpp::Parameter("map/y_size", 50.0));
  n->get_parameter_or("map/z_size", _z_size_, rclcpp::Parameter("map/z_size", 5.0));
  n->get_parameter_or("map/obs_num", _obs_num_, rclcpp::Parameter("map/obs_num", 30));
  n->get_parameter_or("map/resolution", _resolution_, rclcpp::Parameter("map/resolution", 0.1));
  n->get_parameter_or("map/circle_num", _circle_num_, rclcpp::Parameter("map/circle_num", 30));

  n->get_parameter_or("ObstacleShape/lower_rad", _w_l_, rclcpp::Parameter("ObstacleShape/lower_rad", 0.3));
  n->get_parameter_or("ObstacleShape/upper_rad", _w_h_, rclcpp::Parameter("ObstacleShape/upper_rad", 0.8));
  n->get_parameter_or("ObstacleShape/lower_hei", _h_l_, rclcpp::Parameter("ObstacleShape/lower_hei", 3.0));
  n->get_parameter_or("ObstacleShape/upper_hei", _h_h_, rclcpp::Parameter("ObstacleShape/upper_hei", 7.0));

  n->get_parameter_or("ObstacleShape/radius_l", _radius_l_, rclcpp::Parameter("ObstacleShape/radius_l", 7.0));
  n->get_parameter_or("ObstacleShape/radius_h", _radius_h_, rclcpp::Parameter("ObstacleShape/radius_h", 7.0));
  n->get_parameter_or("ObstacleShape/z_l", _z_l_, rclcpp::Parameter("ObstacleShape/z_l", 7.0));
  n->get_parameter_or("ObstacleShape/z_h", _z_h_, rclcpp::Parameter("ObstacleShape/z_h", 7.0));
  n->get_parameter_or("ObstacleShape/theta", _theta_, rclcpp::Parameter("ObstacleShape/theta", 7.0));
  n->get_parameter_or("fix_obs_type", _fix_obs_type_, rclcpp::Parameter("fix_obs_type", 0));

  n->get_parameter_or("pub_rate", _pub_rate_, rclcpp::Parameter("pub_rate", 10.0));
  n->get_parameter_or("min_distance", _min_dist_, rclcpp::Parameter("min_distance", 1.0));

  _obs_num = _obs_num_.as_int();
  _x_size = _x_size_.as_double();
  _y_size = _y_size_.as_double();
  _z_size = _z_size_.as_double();
  _w_l = _w_l_.as_double();
  _w_h = _w_h_.as_double();
  _h_l = _h_l_.as_double();
  _h_h = _h_h_.as_double();
  _resolution = _resolution_.as_double();
  _pub_rate = _pub_rate_.as_double();
  _min_dist = _min_dist_.as_double();
  fix_obs_type_ = _fix_obs_type_.as_int();

  circle_num_ = _circle_num_.as_int();
  radius_l_ = _radius_l_.as_double();
  radius_h_ = _radius_h_.as_double();
  z_l_ = _z_l_.as_double();
  z_h_ = _z_h_.as_double();
  theta_ = _theta_.as_double();

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;
  
  // ros::Duration(0.5).sleep();
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  
  unsigned int seed = rd(); 

  cout << "seed = " << seed << " ~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~" << endl;
  eng.seed(seed);
  
  // RandomMapGenerate();  wait to debug
  RandomMapGenerateCylinder(n);
  // FixMapGenerate();

  rclcpp::Rate loop_rate(_pub_rate);
  
  // real map
  while (rclcpp::ok())
  {
    pubPoints(n);   
    rclcpp::spin_some(n);
    loop_rate.sleep();
  }

}