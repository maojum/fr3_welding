#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 从降采样后的平面聚类中选择焊缝的两个目标平面
 * @details 目的：通过点间距离判断找到相邻的两个平面，它们即为焊缝的左右两个侧面。
 *          遍历所有聚类中的平面对，找到点间距离 < 7mm 的平面对作为焊缝平面。
 * @param Dwon_All_Plane_Cluster 输入：降采样后的平面聚类 [类别][平面][点]
 * @param target 输出：目标焊缝所在的聚类索引
 * @param target_1 输出：焊缝的第一个平面在聚类中的索引
 * @param target_2 输出：焊缝的第二个平面在聚类中的索引
 */
void SeamTwoPlane(vector<vector<vector<Point_3D>>> Dwon_All_Plane_Cluster, int& target, int& target_1, int& target_2)
{
	// ============ 五层嵌套循环：查找距离最近的两个平面 ============
	// 使用多个标志位实现"找到即退出所有循环"的功能
	
	for (int i = 0;i < Dwon_All_Plane_Cluster.size();i++)  // 遍历所有聚类（类别）
	{
		double m = 0;  // 标志位m：是否在当前聚类找到焊缝平面

		for (int j = 0;j < Dwon_All_Plane_Cluster[i].size() - 1;j++)  // 遍历当前聚类中的每个平面
		{
			double d = 0;  // 标志位d：是否在当前平面j找到配对平面

			for (int w = 0;w < Dwon_All_Plane_Cluster[i][j].size();w++)  // 遍历平面j中的每个点
			{
				double c = 0;  // 标志位c：当前点是否找到近距离点

				// 将平面j与后续所有平面进行比较（避免重复比较）
				for (int e = j + 1;e < Dwon_All_Plane_Cluster[i].size();e++)  // 遍历平面j之后的所有平面
				{
					double a = 0;  // 标志位a：平面e与平面j是否匹配
					double b = 0;  // 标志位b：是否需要退出e循环

					for (int r = 0;r < Dwon_All_Plane_Cluster[i][e].size();r++)  // 遍历平面e中的每个点
					{
						// 计算平面j的点w与平面e的点r之间的欧氏距离
						double dist = 0;
						dist = sqrt(pow(Dwon_All_Plane_Cluster[i][j][w].x - Dwon_All_Plane_Cluster[i][e][r].x, 2) + 
						           pow(Dwon_All_Plane_Cluster[i][j][w].y - Dwon_All_Plane_Cluster[i][e][r].y, 2) + 
						           pow(Dwon_All_Plane_Cluster[i][j][w].z - Dwon_All_Plane_Cluster[i][e][r].z, 2));

						// 如果距离小于7mm，认为这两个平面相邻，即为焊缝的两个侧面
						if (dist < 7)
						{
							a = 1;
							b = 1;
							c = 1;
							d = 1;
							m = 1;
							cout << "第" << i + 1 << "组的第" << j + 1 << "个平面和第" << e + 1 << "个平面是焊缝平面" << endl;

							// 记录目标焊缝平面的索引
							target = i;      // 聚类索引
							target_1 = j;    // 第一个焊缝平面索引
							target_2 = e;    // 第二个焊缝平面索引

							break;  // 找到即退出最内层循环
						}
					}

					if (a == 0)
					{
						//cout << "第" << i + 1 << "组的第" << j + 1 << "个平面和第" << e + 1 << "个平面不是焊缝平面" << endl;;
					}

					if (b == 1)  // 如果找到焊缝平面，退出e循环
					{
						break;
					}
				}

				if (c == 1)  // 如果找到焊缝平面，退出w循环
				{
					break;
				}
			}
			
			if (d == 1)  // 如果找到焊缝平面，退出j循环
			{
				break;
			}
		}

		if (m == 1)  // 如果找到焊缝平面，退出i循环
		{
			break;
		}
	}
}
