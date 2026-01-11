#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 显示焊缝端点和交线
 * @details 目的：将交点分配到各自所属的交线，旋转还原到原始坐标系，连接端点生成可视化交线。
 *          这是焊缝轨迹提取的最后一步，用于可视化焊缝的棱边和角点。
 * @param Filt_Unique_All_Line 输入：所有唯一交线的点云集合（已旋转后的坐标）
 * @param Determine_All_IntersectPoint 输入：所有去重后的交点（已旋转后的坐标）
 * @param End_Show 输出：连接端点后的交线可视化点云
 * @param End_Point_All 输出：每条交线的端点集合（旋转还原后的原始坐标）
 */
void SeamEndShow(vector<vector<Point_3D>> Filt_Unique_All_Line, pcl::PointCloud<pcl::PointXYZ>::Ptr Determine_All_IntersectPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr& End_Show, vector<vector<Point_3D>>& End_Point_All)
{
	// ============ 第一步：为每条交线分配其端点（交点） ============
	// 算法：遍历每条交线，找到距离该交线足够近（<2mm）的所有交点
	//       这些交点就是该交线的端点

	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_Cloud(new pcl::PointCloud<pcl::PointXYZ>());  // 临时存储当前交线的所有点

	vector<Point_3D> End_Point;  // 存储当前交线的端点（交点）

	// 外层循环：遍历每条交线
	for (int i = 0; i < Filt_Unique_All_Line.size(); i++)
	{
		// 将当前交线的所有点转换为pcl::PointXYZ格式并存入Temp_Cloud
		for (int j = 0; j < Filt_Unique_All_Line[i].size(); j++)
		{
			pcl::PointXYZ p;

			p.x = Filt_Unique_All_Line[i][j].x;
			p.y = Filt_Unique_All_Line[i][j].y;
			p.z = Filt_Unique_All_Line[i][j].z;

			Temp_Cloud->points.push_back(p);
		}

		// 准备辅助点云和旋转参数
		pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_Cloud_Use(new pcl::PointCloud<pcl::PointXYZ>());  // 临时存储单个交点
		pcl::PointCloud<pcl::PointXYZ>::Ptr End_Point_Cloud_Revolve(new pcl::PointCloud<pcl::PointXYZ>());  // 存储旋转还原后的点
		pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_Cloud_Point(new pcl::PointCloud<pcl::PointXYZ>());  // 未使用

		// ============ 设置反向旋转参数 ============
		// 目的：将旋转后的交点还原到原始坐标系
		// 原始法向量: (1, 1, 1) → 目标法向量: (0, 0, 1)
		// 这是SeamRevolve执行的逆变换
		Revolve Parameter_Back;

		Parameter_Back.a = 1;  // 原始法向量x分量
		Parameter_Back.b = 1;  // 原始法向量y分量
		Parameter_Back.c = 1;  // 原始法向量z分量
		Parameter_Back.d = 0;  // 目标法向量x分量
		Parameter_Back.e = 0;  // 目标法向量y分量
		Parameter_Back.f = 1;  // 目标法向量z分量

		// ============ 第二步：查找属于当前交线的交点 ============
		// 遍历所有交点，检查是否靠近当前交线
		for (int k = 0; k < Determine_All_IntersectPoint->points.size(); k++)
		{
			// 遍历当前交线的所有点
			for (int l = 0; l < Temp_Cloud->points.size(); l++)
			{
				// 计算交点与交线上某点的距离
				double dist;
				dist = sqrt(pow(Determine_All_IntersectPoint->points[k].x - Temp_Cloud->points[l].x, 2) + 
				            pow(Determine_All_IntersectPoint->points[k].y - Temp_Cloud->points[l].y, 2) + 
				            pow(Determine_All_IntersectPoint->points[k].z - Temp_Cloud->points[l].z, 2));

				// 如果距离小于2mm，说明该交点属于当前交线
				if (dist < 2)
				{
					std::cout << "这个点找到了归属,属于交线" << endl;

					// 将交点添加到临时点云
					pcl::PointXYZ p;
					p.x = Determine_All_IntersectPoint->points[k].x;
					p.y = Determine_All_IntersectPoint->points[k].y;
					p.z = Determine_All_IntersectPoint->points[k].z;

					Temp_Cloud_Use->points.push_back(p);

					// ============ 第三步：将交点旋转还原到原始坐标系 ============
					// 因为之前的处理中点云被旋转过，现在需要还原回去
					SeamRevolve(Temp_Cloud_Use, End_Point_Cloud_Revolve, Parameter_Back);

					std::cout << "旋转完成" << endl;

					// 提取旋转还原后的端点坐标
					Point_3D P;
					P.x = End_Point_Cloud_Revolve->points[0].x;
					P.y = End_Point_Cloud_Revolve->points[0].y;
					P.z = End_Point_Cloud_Revolve->points[0].z;

					std::cout << "旋转后的点为(" << P.x << "," << P.y << "," << P.z << ")" << endl;

					// 将端点添加到当前交线的端点集合
					End_Point.push_back(P);

					// 清空临时点云，准备处理下一个交点
					Temp_Cloud_Use->points.resize(0);
					End_Point_Cloud_Revolve->points.resize(0);

					break;  // 找到一个匹配点后退出内层循环
				}
			}
		}

		// ============ 第四步：保存当前交线的端点集合 ============
		// 如果找到了端点，将其添加到总的端点集合中
		if (End_Point.size() > 0)
		{
			End_Point_All.push_back(End_Point);  // 将当前交线的端点添加到总集合

			Temp_Cloud->points.resize(0);  // 清空临时点云，准备处理下一条交线

			End_Point.resize(0);  // 清空当前交线的端点集合
		}
	}

	std::cout << "交线的容器里一共有" << End_Point_All.size() << "条交线" << endl;

	// ============ 第五步：将端点保存到PCD文件（调试用） ============
	pcl::PointCloud<pcl::PointXYZ>::Ptr End_Point_Cloud(new pcl::PointCloud<pcl::PointXYZ>());  // 临时存储每条交线的端点

	pcl::PCDWriter writer;  // PCD文件写入器

	// 遍历每条交线的端点集合，保存到独立的PCD文件
	for (int i = 0; i < End_Point_All.size(); i++)
	{
		// 将当前交线的所有端点转换为pcl::PointXYZ格式
		for (int j = 0; j < End_Point_All[i].size(); j++)
		{
			pcl::PointXYZ p;
			p.x = End_Point_All[i][j].x;
			p.y = End_Point_All[i][j].y;
			p.z = End_Point_All[i][j].z;

			End_Point_Cloud->points.push_back(p);
		}

		// 设置点云尺寸（无序点云）
		End_Point_Cloud->width = End_Point_Cloud->points.size();
		End_Point_Cloud->height = 1;

		// 保存到/tmp/目录，文件名格式：End_Point_Cloud_1_1.pcd, End_Point_Cloud_1_2.pcd, ...
		writer.write<pcl::PointXYZ>("/tmp/End_Point_Cloud_1_" + to_string(i + 1) + ".pcd", *End_Point_Cloud, false);

		// 清空点云，准备处理下一条交线
		End_Point_Cloud->points.resize(0);
	}

	std::cout << "完成交线的分配" << endl;

	// ============ 第六步：连接端点生成交线的可视化表示 ============
	// 目的：用直线段连接每条交线的两个端点，生成完整的焊缝棱边可视化
	// 方法：使用参数方程 P(t) = P0 + t*(P1 - P0)，t ∈ [0, 1]，步长0.1

	for (int i = 0; i < End_Point_All.size(); i++)
	{
		// 参数方程：从端点0到端点1的线性插值
		// t=0 时为第一个端点，t=1 时为第二个端点
		for (double t = 0; t <= 1; t += 0.1)
		{
			pcl::PointXYZ p;

			// 线性插值公式：P(t) = P0 + t*(P1 - P0)
			p.x = End_Point_All[i][0].x + t * (End_Point_All[i][1].x - End_Point_All[i][0].x);
			p.y = End_Point_All[i][0].y + t * (End_Point_All[i][1].y - End_Point_All[i][0].y);
			p.z = End_Point_All[i][0].z + t * (End_Point_All[i][1].z - End_Point_All[i][0].z);

			End_Show->points.push_back(p);  // 添加到可视化点云
		}

		std::cout << "连接完成一条棱" << endl;
	}
}