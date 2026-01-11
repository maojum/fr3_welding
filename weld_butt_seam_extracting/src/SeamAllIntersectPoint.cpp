#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 查找所有交线之间的交点
 * @details 目的：遍历所有交线对，找到它们的交点（即多条交线汇聚的位置）。
 *          交点是焊缝角点的候选位置，对于焊缝轨迹提取很重要。
 * @param Filt_Unique_All_Line 输入：所有唯一交线的点云集合，每条交线是一个Point_3D向量
 * @param All_IntersectPoint 输出：所有找到的交点的点云
 */
void SeamAllIntersectPoint(vector<vector<Point_3D>> Filt_Unique_All_Line, pcl::PointCloud<pcl::PointXYZ>::Ptr& All_IntersectPoint)
{
	// ============ 遍历所有交线对，寻找交点 ============
	// 算法：对于任意两条不同的交线，检查它们的点之间的距离
	//       如果距离小于0.9mm，认为找到了交点
	
	// 外层循环：遍历第一条交线（索引 i）
	for (int i = 0; i < Filt_Unique_All_Line.size() - 1; i++)
	{
		// 中层循环：遍历第二条交线（索引 j，从 i+1 开始避免重复比较）
		for (int j = i + 1; j < Filt_Unique_All_Line.size(); j++)
		{
			double dist;  // 两点之间的欧式距离

			int a = 0;  // 标志变量：用于外层循环的提前退出（当前未使用）

			// 内层循环1：遍历第 i 条交线上的所有点
			for (int m = 0; m < Filt_Unique_All_Line[i].size(); m++)
			{
				int b = 0;  // 标志变量：用于内层循环的提前退出，1表示找到交点

				// 内层循环2：遍历第 j 条交线上的所有点
				for (int n = 0; n < Filt_Unique_All_Line[j].size(); n++)
				{
					// 计算第 i 条线的第 m 个点与第 j 条线的第 n 个点之间的欧式距离
					dist = sqrt(pow(Filt_Unique_All_Line[i][m].x - Filt_Unique_All_Line[j][n].x, 2) + 
					            pow(Filt_Unique_All_Line[i][m].y - Filt_Unique_All_Line[j][n].y, 2) + 
					            pow(Filt_Unique_All_Line[i][m].z - Filt_Unique_All_Line[j][n].z, 2));

					// 判断是否找到交点（距离阈值：0.9mm）
					if (dist < 0.9)
					{
						cout << "找到交点" << endl;

						// 将交点添加到输出点云
						pcl::PointXYZ p;
						p.x = Filt_Unique_All_Line[i][m].x;
						p.y = Filt_Unique_All_Line[i][m].y;
						p.z = Filt_Unique_All_Line[i][m].z;

						All_IntersectPoint->points.push_back(p);

						a = 0;  // 重置外层标志（当前实际未起作用）
						b = 1;  // 设置内层标志，表示找到交点

						break;  // 退出内层循环2（不再检查第 j 条线的其他点）
					}
				}

				// 如果找到了交点，退出内层循环1（不再检查第 i 条线的其他点）
				if (b == 1)
				{
					break;
				}
			}

			// 如果设置了外层标志，退出中层循环（当前条件永远为false，a始终为0）
			if (a == 1)
			{
				break;
			}
		}
	}

	// ============ 注意事项 ============
	// TODO: 应该还需要一个去重的过程
	// 原因：同一个交点可能被多对交线重复检测到
	//       例如：三条线交于一点，会产生3个重复的交点（Line1-Line2, Line1-Line3, Line2-Line3）
	// 建议：添加去重算法，合并距离小于某个阈值（如1mm）的重复交点
}