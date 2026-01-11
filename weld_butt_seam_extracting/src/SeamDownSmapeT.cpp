#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 对目标焊缝平面和所有原始平面进行降采样
 * @details 目的：减少点云数据量，提高后续处理速度。先对两个焊缝目标平面降采样，
 *          再对所有原始平面进行降采样，使用半径为3mm的均匀采样滤波器。
 * @param All_Plane_Point 输入：所有原始平面的点集 [平面][点]
 * @param Down_Smape_All 输出：降采样后的所有平面点集
 * @param Target_Plane_1 输入：第一个焊缝目标平面
 * @param Target_Plane_2 输入：第二个焊缝目标平面
 * @param Down_Target_Plane_1 输出：降采样后的第一个焊缝目标平面
 * @param Down_Target_Plane_2 输出：降采样后的第二个焊缝目标平面
 */
void SeamDownSmapeT(vector <vector<Point_3D>> All_Plane_Point, vector <vector<Point_3D>>& Down_Smape_All, pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_1, pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_2, pcl::PointCloud<pcl::PointXYZ>::Ptr& Down_Target_Plane_1, pcl::PointCloud<pcl::PointXYZ>::Ptr& Down_Target_Plane_2)
{
	// ============ 第一步：对两个焊缝目标平面进行降采样 ============
	
	pcl::UniformSampling<pcl::PointXYZ> filter;		// 创建均匀采样对象

	// 降采样第一个焊缝目标平面
	filter.setInputCloud(Target_Plane_1);			// 设置待采样点云
	filter.setRadiusSearch(3);					// 设置采样半径为 3mm
	filter.filter(*Down_Target_Plane_1);			// 执行均匀采样，结果保存在 Down_Target_Plane_1 中

	// 降采样第二个焊缝目标平面
	filter.setInputCloud(Target_Plane_2);			// 设置待采样点云
	filter.setRadiusSearch(3);					// 设置采样半径为 3mm
	filter.filter(*Down_Target_Plane_2);			// 执行均匀采样，结果保存在 Down_Target_Plane_2 中

	// ============ 第二步：对所有原始平面进行降采样 ============
	// 原因：先降采样再过滤，否则数据量太大，处理速度会很慢

	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_cloud(new  pcl::PointCloud<pcl::PointXYZ>);       // 临时存放降采样前点云的容器
	pcl::PointCloud<pcl::PointXYZ>::Ptr Down_Temp_cloud(new  pcl::PointCloud<pcl::PointXYZ>); // 临时存放降采样后点云的容器

	vector<Point_3D> Temp_Points;  // 临时存储当前平面的降采样点

	// 遍历所有原始平面
	for (int i = 0;i < All_Plane_Point.size();i++)
	{
		// 将当前平面的所有点转换为 PCL 点云格式
		for (int j = 0;j < All_Plane_Point[i].size();j++)
		{
			pcl::PointXYZ p;
			p.x = All_Plane_Point[i][j].x;
			p.y = All_Plane_Point[i][j].y;
			p.z = All_Plane_Point[i][j].z;

			Temp_cloud->points.push_back(p);
		}

		// 执行均匀采样降采样
		filter.setInputCloud(Temp_cloud);			// 设置待采样点云
		filter.setRadiusSearch(3);					// 设置采样半径为 3mm
		filter.filter(*Down_Temp_cloud);			// 执行均匀采样，结果保存在 Down_Temp_cloud 中

		Temp_cloud->points.resize(0);  // 清空临时容器

		// ============ 保存降采样结果到 PCD 文件（用于调试） ============
		std::stringstream ss;
		pcl::PCDWriter writer;

		cout << "降采样完第" << i + 1 << "个面，共有" << Down_Temp_cloud->points.size() << "个点" << endl;

		Down_Temp_cloud->width = Down_Temp_cloud->points.size();
		Down_Temp_cloud->height = 1;
		writer.write<pcl::PointXYZ>("/tmp/Down_Temp_cloud" + to_string(i + 1) + ".pcd", *Down_Temp_cloud, false);

		// ============ 将降采样后的点转换回 Point_3D 格式并存储 ============
		for (int o = 0;o < Down_Temp_cloud->points.size();o++)
		{
			Point_3D P;
			P.x = Down_Temp_cloud->points[o].x;
			P.y = Down_Temp_cloud->points[o].y;
			P.z = Down_Temp_cloud->points[o].z;

			Temp_Points.push_back(P);
		}

		// 将当前平面的降采样结果添加到输出容器
		Down_Smape_All.push_back(Temp_Points);

		// 清空临时容器，为下一个平面做准备
		Temp_Points.resize(0);
		Down_Temp_cloud->points.resize(0);
	}
}