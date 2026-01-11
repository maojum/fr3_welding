#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 从相邻平面索引中排除焊缝目标平面自身
 * @details 目的：SeamFourPlane找到的相邻平面包含焊缝平面自身，需要通过法向量比较将其排除，
 *          只保留真正与焊缝平面相连接的其他平面。
 * @param Target_Plane_1 输入：焊缝目标平面
 * @param All_Factor 输入：所有平面的归一化法向量系数
 * @param More_Index_1 输入：包含自身的相邻平面索引集合
 * @param Accurate_Index_1 输出：排除自身后的精确相邻平面索引
 */
void SeamFindSelf(pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_1, vector<Leading_Factor>  All_Factor, vector<double>  More_Index_1, vector<double>& Accurate_Index_1)
{
	// ============ 第一步：为目标焊缝平面估计法向量 ============
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  // 法线估计对象
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;    // 基于法向量的平面分割对象
	pcl::ExtractIndices<pcl::PointXYZ> extract;      // 点提取对象
	pcl::ExtractIndices<pcl::Normal> extract_normals;    // 法向量提取对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>); // 存储点云的法向量

	// 配置法向量估计
	ne.setSearchMethod(tree);
	ne.setInputCloud(Target_Plane_1);
	ne.setKSearch(50);  // 使用最近的50个邻域点来估计每个点的法向量
	ne.compute(*cloud_normals);  // 执行法向量估计

	std::cout << "原始点云的法向量计算完成" << endl;

	// ============ 第二步：使用RANSAC算法拟合平面，获取平面方程系数 ============
	
	seg.setOptimizeCoefficients(true);  // 优化平面系数
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);  // 使用基于法向量的平面模型
	seg.setNormalDistanceWeight(0.1);  // 法向量距离权重
	seg.setMethodType(pcl::SAC_RANSAC);  // 使用RANSAC算法
	seg.setMaxIterations(100);  // 最大迭代次数
	seg.setDistanceThreshold(0.03);  // 距离阈值0.03mm
	seg.setInputCloud(Target_Plane_1);
	seg.setInputNormals(cloud_normals);

	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

	// 执行平面分割，获取平面模型系数和内点
	seg.segment(*inliers_plane, *coefficients_plane);

	// 提取分割得到的平面上的点
	extract.setInputCloud(Target_Plane_1);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// 存储分割得到的平面上的点到点云文件
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_plane);
	cout << "提取平面完成" << endl;

	// ============ 第三步：统一法向量方向并归一化 ============
	
	Leading_Factor Factor;       // 原始法向量
	Leading_Factor Dan_Factor;   // 归一化后的法向量

	// 提取平面方程系数 (A, B, C)
	Factor.A = coefficients_plane->values[0];
	Factor.B = coefficients_plane->values[1];
	Factor.C = coefficients_plane->values[2];

	cout << "当前焊接平面的法向量提取完成:(" << Factor.A << "," << Factor.B << "," << Factor.C << ")" << endl;

	// 统一法向量方向：确保A分量为正
	if (Factor.A > 0)
	{
		Factor.A = Factor.A;
		Factor.B = Factor.B;
		Factor.C = Factor.C;

		// 归一化操作：将法向量长度标准化为1
		Dan_Factor.A = Factor.A / sqrt(pow(Factor.A, 2) + pow(Factor.B, 2) + pow(Factor.C, 2));
		Dan_Factor.B = Factor.B / sqrt(pow(Factor.A, 2) + pow(Factor.B, 2) + pow(Factor.C, 2));
		Dan_Factor.C = Factor.C / sqrt(pow(Factor.A, 2) + pow(Factor.B, 2) + pow(Factor.C, 2));

		cout << "归一化法向量为：（" << Dan_Factor.A << "," << Dan_Factor.B << "," << Dan_Factor.C << ")" << endl;
	}
	else  // 如果A分量为负，取反使其为正
	{
		Factor.A = -Factor.A;
		Factor.B = -Factor.B;
		Factor.C = -Factor.C;

		// 归一化操作
		Dan_Factor.A = Factor.A / sqrt(pow(Factor.A, 2) + pow(Factor.B, 2) + pow(Factor.C, 2));
		Dan_Factor.B = Factor.B / sqrt(pow(Factor.A, 2) + pow(Factor.B, 2) + pow(Factor.C, 2));
		Dan_Factor.C = Factor.C / sqrt(pow(Factor.A, 2) + pow(Factor.B, 2) + pow(Factor.C, 2));

		cout << "归一化法向量为：（" << Dan_Factor.A << "," << Dan_Factor.B << "," << Dan_Factor.C << ")" << endl;
	}
	
	// ============ 第四步：通过法向量比较排除自身平面 ============
	// 将焊接平面的法向量与所有相邻平面的法向量进行比较

	for (int i = 0;i < More_Index_1.size();i++)
	{
		// 如果法向量的三个分量都非常接近（差值<0.1），说明是同一个平面（自身）
		if (abs(All_Factor[More_Index_1[i]].A - Dan_Factor.A) < 0.1 && 
		    abs(All_Factor[More_Index_1[i]].B - Dan_Factor.B) < 0.1 && 
		    abs(All_Factor[More_Index_1[i]].C - Dan_Factor.C) < 0.1)
		{
			cout << "该面是与焊接平面相同的面" << endl;  
		}
		else  // 法向量不同，说明是真正的相邻平面
		{
			Accurate_Index_1.push_back(More_Index_1[i]);  // 保存到精确索引列表
			cout << "下标分别为" << i << endl;
		}
	}
}