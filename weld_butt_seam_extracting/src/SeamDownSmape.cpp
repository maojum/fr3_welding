#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 对聚类后的所有平面进行均匀采样降采样
 * @details 目的：通过降采样减少点云数据量，提高后续处理的速度。使用半径为3mm的均匀采样滤波器。
 * @param All_Plane_Cluster 输入：聚类后的平面集合 [类别][平面][点]
 * @param Down_All_Plane_Cluster 输出：降采样后的平面集合，结构与输入相同
 */
void SeamDownSmape(vector<vector<vector<Point_3D>>> All_Plane_Cluster, vector<vector<vector<Point_3D>>> &Down_All_Plane_Cluster)
{
	// ============ 初始化降采样处理所需的容器 ============
	vector<vector<Point_3D>> Down_Plane_Lei;    // 临时存储当前聚类的降采样平面结果
	vector<Point_3D> Down_Plane_Point;          // 临时存储单个平面的降采样点

	// PCL 点云容器
	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_cloud(new pcl::PointCloud<pcl::PointXYZ>); //临时点云（用于存储待采样的点）
	pcl::PointCloud<pcl::PointXYZ>::Ptr Down_cloud(new pcl::PointCloud<pcl::PointXYZ>); //降采样后的点云

	// 创建均匀采样滤波器对象
	pcl::UniformSampling<pcl::PointXYZ> filter;		// 创建均匀采样对象

	std::stringstream ss;
	pcl::PCDWriter writer;

	int sum = 1;  // 计数器，用于生成唯一的文件名

	// ============ 三层循环：遍历所有聚类、每个聚类中的平面、每个平面中的点 ============
	for (int i = 0;i < All_Plane_Cluster.size();i++)  // 遍历所有聚类（类别）
	{
		for (int j = 0;j < All_Plane_Cluster[i].size();j++)  // 遍历当前聚类中的每个平面
		{
			// 将当前平面的所有点转换为 PCL 点云格式
			for (int u = 0;u < All_Plane_Cluster[i][j].size();u++)
			{
				pcl::PointXYZ p;

				p.x = All_Plane_Cluster[i][j][u].x;
				p.y = All_Plane_Cluster[i][j][u].y;
				p.z = All_Plane_Cluster[i][j][u].z;

				Temp_cloud->points.push_back(p);
			}

			// ============ 执行均匀采样降采样 ============
			filter.setInputCloud(Temp_cloud);					// 设置待采样点云
			filter.setRadiusSearch(3);					// 设置采样半径为 3mm，每个采样点周围3mm范围内只保留一个点
			filter.filter(*Down_cloud);					// 执行均匀采样，结果保存在 Down_cloud 中

			cout << "过滤完第" << sum << "个面，共有" << Down_cloud->points.size() << "个点" << endl;

			// ============ 保存降采样结果到 PCD 文件（用于调试） ============
			Down_cloud->width = Down_cloud->points.size();
			Down_cloud->height = 1;
			writer.write<pcl::PointXYZ>("/tmp/Down_Plane" + to_string(sum) + ".pcd", *Down_cloud, false);

			sum++;

			// ============ 将降采样后的点转换回 Point_3D 格式并存储 ============
			for (int w = 0;w < Down_cloud->points.size();w++)
			{
				Point_3D P;
				P.x = Down_cloud->points[w].x;  
				P.y = Down_cloud->points[w].y;
				P.z = Down_cloud->points[w].z;

				Down_Plane_Point.push_back(P);
			}

			// 将当前平面的降采样结果添加到当前聚类
			Down_Plane_Lei.push_back(Down_Plane_Point);

			// ============ 清空临时容器，为下一个平面做准备 ============
			Down_Plane_Point.resize(0);
			Temp_cloud->points.resize(0);
			Down_cloud->points.resize(0);
		}

		// 将当前聚类的所有降采样平面添加到输出结果
		Down_All_Plane_Cluster.push_back(Down_Plane_Lei);
		Down_Plane_Lei.resize(0);  // 清空，为下一个聚类做准备
	}
}