#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 计算平面方程的完整系数
 * @details 目的：通过法向量估计和RANSAC平面拟合，计算平面方程 Ax+By+Cz+D=0 的所有系数。
 *          使用基于法向量的平面分割方法，提高拟合精度。
 * @param cloud 输入：待计算的点云
 * @param Factor 输出：平面方程系数(A,B,C,D)
 */
void SeamCoefficient(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Leading_Factor& Factor)
{
	// ============ 第一步：法向量估计 ============
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  // 法线估计对象
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;    // 基于法向量的分割对象
	pcl::ExtractIndices<pcl::PointXYZ> extract;      // 点提取对象
	pcl::ExtractIndices<pcl::Normal> extract_normals;    // 法向量提取对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>); // 存储点云的法向量

	// 配置法向量估计参数
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);  // 使用最近的50个邻域点来估计每个点的法向量
	ne.compute(*cloud_normals);  // 执行法向量估计

	// ============ 第二步：基于法向量的RANSAC平面拟合 ============
	
	seg.setOptimizeCoefficients(true);  // 优化平面系数以获得更准确的结果
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);  // 使用基于法向量的平面模型
	seg.setNormalDistanceWeight(0.1);  // 法向量距离权重
	seg.setMethodType(pcl::SAC_RANSAC);  // 采用RANSAC随机采样一致性算法
	seg.setMaxIterations(100);  // 最大迭代次数
	seg.setDistanceThreshold(0.03);  // 距离阈值0.03mm，点到平面的距离小于此值视为内点
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);

	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

	// 执行平面分割，获取平面模型的系数和内点索引
	seg.segment(*inliers_plane, *coefficients_plane);

	// ============ 第三步：提取平面上的点（用于验证） ============
	
	// 从点云中提取分割得到的处在平面上的点集
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);  // 提取内点（属于平面的点）

	extract.filter(*cloud_plane);  // 执行提取操作

	// ============ 第四步：提取平面方程的所有系数 ============
	// 平面方程: Ax + By + Cz + D = 0
	
	Factor.A = coefficients_plane->values[0];  // x轴系数
	Factor.B = coefficients_plane->values[1];  // y轴系数
	Factor.C = coefficients_plane->values[2];  // z轴系数
	Factor.D = coefficients_plane->values[3];  // 常数项

	cout << "平面的前置系数全部提取完成" << endl;
}