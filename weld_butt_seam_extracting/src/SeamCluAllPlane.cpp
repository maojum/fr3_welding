#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 根据法向量相似性对所有平面进行聚类
 * @param All_Plane_Point 所有平面的点集
 * @param All_Factor 所有平面的原始法向量系数
 * @param All_Plane_Cluster 输出：聚类后的平面集合，三维结构 [类别][平面][点]
 * @param New_All_Factor 输出：统一方向并归一化后的法向量
 */
void SeamCluAllPlane(vector<vector<Point_3D>> All_Plane_Point, vector<Leading_Factor> All_Factor, vector<vector<vector<Point_3D>>>& All_Plane_Cluster, vector<Leading_Factor>& New_All_Factor)
{
	// ============ 第一步：统一并归一化所有法向量 ============
	// 目的：确保法向量方向一致（A分量为正），便于后续比较相似性
	
	Leading_Factor New_Factor;        // 统一方向后的法向量
	Leading_Factor Dan_New_Factor;    // 归一化后的法向量

	// 遍历所有平面的法向量
	for (int i = 0;i < All_Factor.size();i++)
	{
		// 如果 A 分量已经是正数，直接使用
		if (All_Factor[i].A > 0)
		{
			New_Factor.A = All_Factor[i].A;
			New_Factor.B = All_Factor[i].B;
			New_Factor.C = All_Factor[i].C;

			// 归一化操作：将法向量长度标准化为1
			// 公式：v_norm = v / ||v||，其中 ||v|| = sqrt(A² + B² + C²)
			Dan_New_Factor.A = New_Factor.A / sqrt(pow(New_Factor.A, 2) + pow(New_Factor.B, 2) + pow(New_Factor.C, 2));
			Dan_New_Factor.B = New_Factor.B / sqrt(pow(New_Factor.A, 2) + pow(New_Factor.B, 2) + pow(New_Factor.C, 2));
			Dan_New_Factor.C = New_Factor.C / sqrt(pow(New_Factor.A, 2) + pow(New_Factor.B, 2) + pow(New_Factor.C, 2));

			New_All_Factor.push_back(Dan_New_Factor);

			cout << "法向量为：（" << Dan_New_Factor.A << "," << Dan_New_Factor.B << "," << Dan_New_Factor.C << ")" << endl;
		}
		else  // 如果 A 分量是负数，取反使其为正
		{
			New_Factor.A = -All_Factor[i].A;
			New_Factor.B = -All_Factor[i].B;
			New_Factor.C = -All_Factor[i].C;

			// 归一化操作
			Dan_New_Factor.A = New_Factor.A / sqrt(pow(New_Factor.A, 2) + pow(New_Factor.B, 2) + pow(New_Factor.C, 2));
			Dan_New_Factor.B = New_Factor.B / sqrt(pow(New_Factor.A, 2) + pow(New_Factor.B, 2) + pow(New_Factor.C, 2));
			Dan_New_Factor.C = New_Factor.C / sqrt(pow(New_Factor.A, 2) + pow(New_Factor.B, 2) + pow(New_Factor.C, 2));

			New_All_Factor.push_back(Dan_New_Factor);

			cout << "法向量为：（" << Dan_New_Factor.A << "," << Dan_New_Factor.B << "," << Dan_New_Factor.C << ")" << endl;
		}
	}
	cout << "所有平面的法向量统一完成" << endl;

	// ============ 第二步：根据法向量相似性进行聚类 ============
	// 目的：将法向量相似的平面归为一类
	
	vector<vector<Point_3D>> Temp_Vector;  // 临时存储当前聚类的平面点集
	vector<double> index;                   // 存储已经被分配到某个类别的平面索引，避免重复分类

	// 遍历所有平面，为每个未分类的平面创建一个新类别
	for (int i = 0;i < New_All_Factor.size() - 1;i++)
	{
		cout << "存着的下标有" << index.size() << "个" << endl;

		int a = 0;  // 标志位：判断当前平面i是否已经被分类过

		// 检查当前平面i是否已经被分配到某个类别
		for (int q = 0;q < index.size();q++)
		{
			if (index[q] == i)
			{
				a = 1;  // 已分类，跳过
				break;
			}
		}

		// 如果当前平面i还未被分类，则以它为基准创建一个新类别
		if (a == 0)
		{
			// 遍历所有平面，找出与平面i法向量相似的平面
			for (int j = i;j < New_All_Factor.size();j++)
			{
				// 相似性判断：法向量的三个分量差值都小于0.1，则认为相似
				if (abs(New_All_Factor[i].A - New_All_Factor[j].A) < 0.1 && 
				    abs(New_All_Factor[i].B - New_All_Factor[j].B) < 0.1 && 
				    abs(New_All_Factor[i].C - New_All_Factor[j].C) < 0.1)
				{
					cout << "找到与第" << i + 1 << "个平面法向量相同的平面" << endl;

					// 将相似的平面点集加入当前类别
					Temp_Vector.push_back(All_Plane_Point[j]);

					// 记录已分类的平面索引
					index.push_back(j);

					// 保存该平面到PCD文件（用于调试和可视化）
					std::stringstream ss;
					pcl::PCDWriter writer;

					pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_cloud(new pcl::PointCloud<pcl::PointXYZ>); //临时输出的点云

					// 将 Point_3D 格式转换为 PCL 点云格式
					for (int k = 0;k < All_Plane_Point[j].size();k++)
					{
						pcl::PointXYZ p;
						p.x = All_Plane_Point[j][k].x;
						p.y = All_Plane_Point[j][k].y;
						p.z = All_Plane_Point[j][k].z;

						Temp_cloud->points.push_back(p);
					}

					Temp_cloud->width = Temp_cloud->points.size();
					Temp_cloud->height = 1;
					writer.write<pcl::PointXYZ>("/tmp/Plane" + to_string(i) + "_" + to_string(j) + ".pcd", *Temp_cloud, false);

					Temp_cloud->points.resize(0);
				}
				else
				{
					cout << "这个面不是已有类别中的面的相似面" << endl;
				}
			}
		}

		// 如果平面i未被分类过，将收集到的同类平面作为一个新类别
		if (a == 0)
		{
			All_Plane_Cluster.push_back(Temp_Vector);
		}

		Temp_Vector.resize(0);  // 清空临时容器，为下一个类别做准备
	}
}