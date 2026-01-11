#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 从点云中分割出所有平面
 * @param All_cloud 输入点云，会被逐步修改以移除已分割的平面
 * @param All_Plane_Point 输出：存储所有分割出的平面点集
 * @param All_Factor 输出：存储所有平面的法向量系数(A, B, C)
 */
void SeamSepAllPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr All_cloud, vector<vector<Point_3D>>& All_Plane_Point, vector<Leading_Factor>& All_Factor)
{
	// 创建平面模型系数和点云索引的容器
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliners(new pcl::PointIndices);

	// 配置 RANSAC 平面分割器
	pcl::SACSegmentation<pcl::PointXYZ> sac;
	sac.setOptimizeCoefficients(true);          // 优化平面系数以获得更准确的结果
	sac.setModelType(pcl::SACMODEL_PLANE);      // 使用平面模型进行分割
	sac.setMethodType(pcl::SAC_RANSAC);         // 采用 RANSAC 随机采样一致性算法
	sac.setDistanceThreshold(0.2);              // 设置距离阈值为 0.2mm，点到平面的距离小于此值视为内点
	sac.setMaxIterations(500);                  // 设置最大迭代次数为 500

	// 初始化循环变量和临时容器
	int i = 0, nr_points = (int)All_cloud->points.size();  // 记录原始点云的总点数
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZ>);  // 存储当前分割出的平面
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZ>);  // 存储移除平面后剩余的点

	Point_3D Temp_Point;
	vector<Point_3D> One_Plane_Point;

	Leading_Factor Factor;

	// 循环分割平面，直到剩余点数少于原始点数的 0.5%
	while (All_cloud->points.size() > 0.005 * nr_points)
	{
		// 使用当前点云进行平面分割
		sac.setInputCloud(All_cloud);
		sac.segment(*inliners, *coefficients);  // 执行分割，获得内点索引和平面系数
		if (inliners->indices.size() == 0)
		{
			cout << "could not remove " << endl;
			break;  // 若无法找到有效平面则退出循环
		}

		// 提取分割出的平面点
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(All_cloud);
		extract.setIndices(inliners);
		extract.setNegative(false);             // 提取内点（属于平面的点）
		extract.filter(*cloudPlane);

		// 提取不属于该平面的点（用于下一轮迭代）
		extract.setNegative(true);              // 提取外点（不属于平面的点）
		extract.filter(*cloudT);
		cout << "提取出的平面中的点云有" << cloudPlane->size() << "个点" << endl;

		// 保存分割出的平面到 PCD 文件（用于调试和验证）
		std::stringstream ss;
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ>("/tmp/Plane" + to_string(i) + ".pcd", *cloudPlane, false); //提取出的平面
		//writer.write<pcl::PointXYZ>("/tmp/Plane" + to_string(i) + ".pcd", *cloudT, false); //提取出平面后剩下的点

		// 提取平面法向量系数 (A, B, C)，用于后续的平面聚类
		Factor.A = coefficients->values[0];
		Factor.B = coefficients->values[1];
		Factor.C = coefficients->values[2];
		cout << "平面的法向量提取完成:(" << Factor.A << "," << Factor.B << "," << Factor.C << ")" << endl;

		// 将法向量系数存储到输出向量中
		All_Factor.push_back(Factor);

		// 将平面上的所有点转换为 Point_3D 格式并存储
		for (int l = 0;l < cloudPlane->points.size();l++)
		{
			Temp_Point.x = cloudPlane->points[l].x;
			Temp_Point.y = cloudPlane->points[l].y;
			Temp_Point.z = cloudPlane->points[l].z;

			One_Plane_Point.push_back(Temp_Point);
		}

		// 将当前平面的所有点存储到输出向量中
		All_Plane_Point.push_back(One_Plane_Point);

		One_Plane_Point.resize(0);              // 清空临时容器，为下一轮迭代做准备

		i++;                                     // 平面计数器递增
		*All_cloud = *cloudT;                   // 使用剩余点继续下一轮分割
	}



}