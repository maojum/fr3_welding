/**
 * @file weld_butt_seam_extracting_headfile.h
 * @brief 焊缝对接接头提取算法头文件
 * @details 包含对接焊缝点云处理、平面分割、聚类及焊缝轨迹提取的相关数据结构和函数声明
 */

#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdbool.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <boost/random.hpp>
#include <stdlib.h>
#include <string>
#include <cstdlib>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <string>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <boost/random.hpp>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <string>
#include <pcl/filters/extract_indices.h>
#include <boost/random.hpp>

#include <Eigen/Core>

// #include <pcl/surface/on_nurbs/fitting_curve_2d.h>
// #include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/thread/thread.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/keypoints/uniform_sampling.h>

// ROS2 includes (replacing ROS1)
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

/**
 * @brief 3D 点云坐标
 * 
 */
struct Point_3D
{
    double x;
    double y;
    double z;
};

/**
 * @brief 平面方程系数
 * @details 表示平面方程 Ax + By + Cz + D = 0 的系数
 */
struct Leading_Factor
{
    double A;  // x轴系数
    double B;  // y轴系数
    double C;  // z轴系数
    double D;  // 常数项
};

/**
 * @brief 4D点坐标
 * @details 包含三维坐标和一个附加参数n
 */
struct Point_4D
{
    double x;  // x坐标
    double y;  // y坐标
    double z;  // z坐标
    double n;  // 附加参数
};

/**
 * @brief 旋转变换参数
 * @details 存储用于点云旋转变换的法向量参数。包含原始平面法向量和目标平面法向量，
 *          用于计算旋转矩阵，将点云从一个坐标系旋转到另一个坐标系。
 */
struct Revolve
{
    double a;  // 原始平面法向量的x分量
    double b;  // 原始平面法向量的y分量
    double c;  // 原始平面法向量的z分量
    double d;  // 目标平面法向量的x分量
    double e;  // 目标平面法向量的y分量
    double f;  // 目标平面法向量的z分量
};

/**
 * @brief 点对比较结构
 * @details 存储两个点的索引及其距离，用于点云配对和距离比较
 */
struct Compare
{
    int index_1;      // 第一个点的索引
    int index_2;      // 第二个点的索引
    double dist_12;   // 两点之间的距离
};

/**
 * @brief 分割所有平面，为找到焊缝连接的两个面做准备
 * @details 使用RANSAC平面分割算法从输入点云中提取所有平面，直到剩余点数少于原始点数的0.5%。
 *          每个平面的法向量系数和点集都会被分别保存。
 * @param All_cloud 输入点云
 * @param All_Plane_Point 输出所有平面的点集，每个元素是一个平面的所有点
 * @param All_Factor 输出所有平面的方程系数(A,B,C)，表示平面法向量
 * @note 使用距离阈值0.2mm和最大迭代次数500进行RANSAC拟合
 * 
 */
void SeamSepAllPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr All_cloud, vector<vector<Point_3D>> &All_Plane_Point, vector<Leading_Factor> &All_Factor);

/**
 * @brief 聚类所有平面，将法向量相似的归为一类，方便后续找到焊缝连接的两个面
 * @details 根据平面法向量的相似性对分割得到的平面进行聚类。首先将所有法向量统一方向（A分量为正）
 *          并归一化，然后比较法向量的相似性（差值<0.1）将相似平面归为一类。
 * @param All_Plane_Point 所有平面的点集
 * @param All_Factor 所有平面的方程系数
 * @param All_Plane_Cluster 输出聚类后的平面点集，三维向量：[类别][平面][点]
 * @param New_All_Factor 输出更新后的归一化平面系数
 * @note 法向量相似性判断阈值为0.1
 */
void SeamCluAllPlane(vector<vector<Point_3D>> All_Plane_Point, vector<Leading_Factor> All_Factor, vector<vector<vector<Point_3D>>> &All_Plane_Cluster, vector<Leading_Factor> &New_All_Factor);

/**
 * @brief 降采样所有平面聚类
 * @details 使用均匀采样(Uniform Sampling)对每个平面进行降采样，减少点云密度以提高后续处理速度
 * @param All_Plane_Cluster 输入的平面聚类
 * @param Dwon_All_Plane_Cluster 输出降采样后的平面聚类
 * @note 采用半径为3mm的均匀采样
 */
void SeamDownSmape(vector<vector<vector<Point_3D>>> All_Plane_Cluster, vector<vector<vector<Point_3D>>> &Down_All_Plane_Cluster);

/**
 * @brief 选择两个焊缝目标平面
 * @details 通过点云间距离比较找到相邻的两个平面作为焊缝平面。
 *          遍历所有聚类中的平面对，找到点间距离<7mm的平面对，即为焊缝的两个侧面。
 * @param Dwon_All_Plane_Cluster 降采样后的平面聚类
 * @param target 输出目标聚类的索引
 * @param target_1 输出第一个目标平面在聚类中的索引
 * @param target_2 输出第二个目标平面在聚类中的索引
 * @note 距离阈值设为7mm来判断平面是否相邻
 */
void SeamTwoPlane(vector<vector<vector<Point_3D>>> Dwon_All_Plane_Cluster, int &target, int &target_1, int &target_2);

/**
 * @brief 降采样目标平面和所有平面
 * @details 对两个焊缝目标平面和所有原始平面进行统一的均匀采样降采样处理
 * @param All_Plane_Point 所有平面的点集
 * @param Down_Smape_All 输出降采样后的所有平面点集
 * @param Target_Plane_1 第一个目标焊缝平面
 * @param Target_Plane_2 第二个目标焊缝平面
 * @param Down_Target_Plane_1 输出降采样后的第一个目标平面
 * @param Down_Target_Plane_2 输出降采样后的第二个目标平面
 * @note 使用半径为3mm的均匀采样
 */
void SeamDownSmapeT(vector<vector<Point_3D>> All_Plane_Point, vector<vector<Point_3D>> &Down_Smape_All, pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_1, pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_2, pcl::PointCloud<pcl::PointXYZ>::Ptr &Down_Target_Plane_1, pcl::PointCloud<pcl::PointXYZ>::Ptr &Down_Target_Plane_2);

/**
 * @brief 查找与目标平面相邻的平面
 * @details 通过点间距离比较，找出所有与第一个目标焊缝平面相邻的平面（包括自身）
 * @param Down_Smape_All 降采样后的所有平面
 * @param Down_Target_Plane_1 降采样后的第一个目标平面
 * @param Six_Index_1 输出与目标平面相邻的平面索引集合
 * @note 距离阈值为2mm，用于判断平面是否相邻
 */
void SeamFourPlane(vector<vector<Point_3D>> Down_Smape_All, pcl::PointCloud<pcl::PointXYZ>::Ptr Down_Target_Plane_1, vector<double> &Six_Index_1);

/**
 * @brief 从相邻平面中排除自身平面
 * @details 通过法向量估计和比较，从相邻平面索引中去除与目标平面法向量相同的平面（即自身），
 *          留下真正与焊缝平面相连的其他平面（通常为4个）
 * @param Target_Plane_1 目标焊缝平面
 * @param All_Factor 所有平面的归一化法向量系数
 * @param More_Index_1 包含自身的相邻平面索引
 * @param Accurate_Index_1 输出精确的相邻平面索引（不包括自身）
 * @note 使用50个邻域点进行法向量估计，法向量差异阈值为0.1
 */
void SeamFindSelf(pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_1, vector<Leading_Factor> All_Factor, vector<double> More_Index_1, vector<double> &Accurate_Index_1);

/**
 * @brief 旋转点云
 * @details 根据给定的旋转参数将点云从一个坐标系旋转到另一个坐标系。
 *          使用轴角表示法(Angle-Axis)计算旋转矩阵，将点云按照指定的旋转轴和角度进行变换。
 * @param All_cloud 输入点云
 * @param All_cloud_Revolve 输出旋转后的点云
 * @param Parameter 旋转参数，包含原始法向量(a,b,c)和目标法向量(d,e,f)
 * @note 通过计算两个法向量的夹角和旋转轴，构建旋转矩阵进行变换
 */
void SeamRevolve(pcl::PointCloud<pcl::PointXYZ>::Ptr All_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &All_cloud_Revolve, Revolve Parameter);

/**
 * @brief 计算平面方程系数
 * @details 使用法向量估计和RANSAC平面拟合计算平面方程 Ax+By+Cz+D=0 的所有系数
 * @param cloud 输入点云
 * @param Factor 输出平面方程系数(A,B,C,D)
 * @note 使用50个邻域点进行法向量估计，RANSAC距离阈值为0.03mm
 */
void SeamCoefficient(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Leading_Factor &Factor);

/**
 * @brief 计算两平面交线
 * @details 根据两个平面的方程系数计算它们的交线。交线方向向量通过两平面法向量叉乘获得，
 *          交线上的一点通过令z=0求解x、y获得。
 * @param Actor_First 第一个平面系数
 * @param Actor_Second 第二个平面系数
 * @param coefficients_line 输出交线系数，包含直线上一点(x0,y0,z0)和方向向量(a,b,c)共6个参数
 * @note 如果两平面平行，计算结果可能无效
 */
void SeamIntersectLine(Leading_Factor Actor_First, Leading_Factor Actor_Second, pcl::ModelCoefficients::Ptr &coefficients_line);

/**
 * @brief 绘制直线点云
 * @details 根据直线的参数方程生成直线上的点云，用于可视化。参数方程为：P = P0 + t*V，
 *          其中P0为直线上一点，V为方向向量，t在[-10000, 10000]范围内变化。
 * @param coefficients_line 直线系数，包含点(x0,y0,z0)和方向向量(a,b,c)
 * @param cloud_line_max 输出直线点云
 * @note 会检测并过滤平行平面的无效交线（两端点距离<5mm）
 */
void SeamDrawLine(pcl::ModelCoefficients::Ptr coefficients_line, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_line_max);

/**
 * @brief 计算所有交线的交点
 * @details 遍历所有交线对，通过点间距离判断找出交线间的交点
 * @param Filt_Unique_All_Line 过滤后的唯一交线集合
 * @param All_IntersectPoint 输出所有交点
 * @note 交点判定的距离阈值为0.9mm
 */
void SeamAllIntersectPoint(vector<vector<Point_3D>> Filt_Unique_All_Line, pcl::PointCloud<pcl::PointXYZ>::Ptr &All_IntersectPoint);

/**
 * @brief 生成最终显示的焊缝轨迹
 * @details 将交点归属到各自的交线，然后将同一交线的两个端点连接成线段，
 *          所有线段组成完整的焊缝轮廓。点会被旋转回原始坐标系。
 * @param Filt_Unique_All_Line 过滤后的唯一交线集合
 * @param Determine_All_IntersectPoint 确定的所有交点
 * @param End_Show 输出最终显示的焊缝轨迹点云
 * @param End_Point_All 输出所有交线的端点对，格式为[交线索引][2个端点]
 * @note 交点归属判定距离阈值为2mm，线段在参数t∈[0,1]以0.1步长生成
 */
void SeamEndShow(vector<vector<Point_3D>> Filt_Unique_All_Line, pcl::PointCloud<pcl::PointXYZ>::Ptr Determine_All_IntersectPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr &End_Show, vector<vector<Point_3D>> &End_Point_All);

/**
 * @brief 连接两组交点生成焊缝中心线
 * @details 将两个焊缝平面上的交点进行配对，通过最小距离匹配找到对应点对，
 *          然后连接每对点形成焊缝中心线。配对前会将点旋转回原始坐标系。
 * @param All_IntersectPoint_1 第一个焊缝平面的交点
 * @param All_IntersectPoint_2 第二个焊缝平面的交点
 * @param End_Show_3 输出连接后的焊缝中心线点云
 * @param End_Point_All_3 输出所有配对的点对，格式为[点对索引][2个点]
 * @note 采用贪心算法为每个点找最近配对点，线段在参数t∈[0,1]以0.1步长生成
 */
void SeamLinkTwo(pcl::PointCloud<pcl::PointXYZ>::Ptr All_IntersectPoint_1, pcl::PointCloud<pcl::PointXYZ>::Ptr All_IntersectPoint_2, pcl::PointCloud<pcl::PointXYZ>::Ptr &End_Show_3, vector<vector<Point_3D>> &End_Point_All_3);

/**
 * @brief 变换点云
 * @param input_cloud 输入点云
 * @param transformed_cloud 输出变换后的点云
 */
void transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud);

/**
 * @brief 获取对接焊缝轨迹
 * @details 从PCD文件中提取对接焊缝，并返回焊缝轨迹位姿序列
 * @param pcd_file_path PCD点云文件路径
 * @return 焊缝轨迹位姿向量
 * @note 返回轨迹供调用者发布，而非直接传入Publisher
 */
std::vector<geometry_msgs::msg::Pose> get_butt_weld_seam(const std::string &pcd_file_path);
