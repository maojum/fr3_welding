#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 查找与第一个焊缝目标平面相邻的所有平面
 * @details 目的：找出所有与焊缝平面距离很近的平面（包括自身），这些平面构成焊缝周围的几何结构。
 *          通过点间距离判断（阈值2mm）来确定平面是否相邻。
 * @param Down_Smape_All 输入：降采样后的所有原始平面 [平面][点]
 * @param Down_Target_Plane_1 输入：降采样后的第一个焊缝目标平面
 * @param Six_Index_1 输出：与焊缝平面相邻的平面索引集合（包括焊缝平面自身）
 */
void SeamFourPlane(vector<vector<Point_3D>> Down_Smape_All, pcl::PointCloud<pcl::PointXYZ>::Ptr  Down_Target_Plane_1, vector<double>& Six_Index_1)
{
	double index;  // 临时存储相邻平面的索引

	// ============ 三层嵌套循环：遍历所有平面并检查与目标平面的距离 ============
	
	for (int i = 0;i < Down_Smape_All.size();i++)  // 遍历所有平面
	{
		for (int j = 0;j < Down_Smape_All[i].size();j++)  // 遍历当前平面的每个点
		{
			double a = 0;  // 标志位：当前平面是否与焊缝平面相邻

			// 遍历焊缝目标平面的所有点，计算与当前平面点的距离
			for (int k = 0;k < Down_Target_Plane_1->points.size();k++)
			{
				// 计算当前平面的点 (i,j) 与焊缝平面的点 k 之间的欧氏距离
				double dist;
				dist = sqrt(pow(Down_Smape_All[i][j].x - Down_Target_Plane_1->points[k].x, 2) + 
				           pow(Down_Smape_All[i][j].y - Down_Target_Plane_1->points[k].y, 2) + 
				           pow(Down_Smape_All[i][j].z - Down_Target_Plane_1->points[k].z, 2));

				// 如果距离小于2mm，则认为这个平面与焊缝平面相邻
				if (dist < 2)
				{
					// 记录相邻平面的索引
					index = i;
					Six_Index_1.push_back(index);

					cout << "找到和焊接面1相邻的面,下标为" << index << endl;

					a = 1;  // 设置标志位，表示已找到
					break;  // 找到一个近距离点即可，退出最内层循环
				}
			}

			// 如果已找到相邻平面，退出当前平面点的遍历
			if (a == 1)
			{
				break;
			}
		}
	}
}