#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 连接两组交点，生成对应端点对并可视化连线
 * @details 目的：对来自两个焊接平面的交点集合做最近邻匹配，每个点在另一组中找到距离最近的对应点；
 *          将匹配到的两个点反向旋转回原始坐标系后成对保存，最后用直线插值连接每一对点得到焊缝棱边。
 * @param All_IntersectPoint_1 输入：焊接平面1的交点集合（旋转后坐标系）
 * @param All_IntersectPoint_2 输入：焊接平面2的交点集合（旋转后坐标系）
 * @param End_Show_3 输出：可视化用的线段点云（每对端点用线性插值生成）
 * @param End_Point_All_3 输出：匹配得到的端点对集合（已反向旋转回原始坐标系）
 */
void SeamLinkTwo(pcl::PointCloud<pcl::PointXYZ>::Ptr All_IntersectPoint_1, pcl::PointCloud<pcl::PointXYZ>::Ptr All_IntersectPoint_2, pcl::PointCloud<pcl::PointXYZ>::Ptr& End_Show_3, vector<vector<Point_3D>>& End_Point_All_3)
{
	vector<Compare> Compare_Dist;  // 存放当前点与另一组所有点的距离信息
	int m = 0;  // 记录最近邻的索引（组1中的点）
	int n = 0;  // 记录最近邻的索引（组2中的点）

	Point_3D Temp_Point_1;  // 旋转还原后的点1
	Point_3D Temp_Point_2;  // 旋转还原后的点2

	vector<Point_3D> Temp_Point;  // 存放一对端点

	// 反向旋转参数：将(1,1,1)法向旋回到(0,0,1)
	Revolve Parameter_Back;
	Parameter_Back.a = 1;
	Parameter_Back.b = 1;
	Parameter_Back.c = 1;
	Parameter_Back.d = 0;
	Parameter_Back.e = 0;
	Parameter_Back.f = 1;

	std::cout << All_IntersectPoint_1->points.size() << endl;
	std::cout << All_IntersectPoint_2->points.size() << endl;

	// ============ 第一步：最近邻匹配（组1的每个点在组2中找最近点） ============
	for (int i = 0;i < All_IntersectPoint_1->points.size();i++)
	{
		for (int j = 0;j < All_IntersectPoint_2->points.size();j++)
		{
			double dist;  // 当前距离
			dist = sqrt(pow(All_IntersectPoint_1->points[i].x - All_IntersectPoint_2->points[j].x, 2) + pow(All_IntersectPoint_1->points[i].y - All_IntersectPoint_2->points[j].y, 2) + pow(All_IntersectPoint_1->points[i].z - All_IntersectPoint_2->points[j].z, 2));

			std::cout << dist << endl;

			Compare	compare_dist;  // 记录索引和距离

			compare_dist.index_1 = i;
			compare_dist.index_2 = j;
			compare_dist.dist_12 = dist;

			Compare_Dist.push_back(compare_dist);

			std::cout << "焊接平面1的第" << i + 1 << "个点与焊接平面2的第" << j + 1 << "g个点的距离为：" << compare_dist.dist_12 << endl;
		}

		double dist_min = 1000000;

		// 在Compare_Dist中找出距离最小的一对
		for (int a = 0;a < Compare_Dist.size();a++)
		{
			if (Compare_Dist[a].dist_12 < dist_min)
			{
				dist_min = Compare_Dist[a].dist_12;

				m = Compare_Dist[a].index_1;
				n = Compare_Dist[a].index_2;

				std::cout << "比较了一次" << endl;
			}
		}

		Compare_Dist.resize(0);  // 清空，为下一个点重新计算

		// ============ 第二步：将匹配到的一对点旋转还原回原始坐标系 ============
		pcl::PointCloud<pcl::PointXYZ>::Ptr Remove_Point_Before(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr Remove_Point_After(new pcl::PointCloud<pcl::PointXYZ>());

		pcl::PointXYZ p_1;

		p_1.x = All_IntersectPoint_1->points[m].x;
		p_1.y = All_IntersectPoint_1->points[m].y;
		p_1.z = All_IntersectPoint_1->points[m].z;

		Remove_Point_Before->points.push_back(p_1);

		SeamRevolve(Remove_Point_Before, Remove_Point_After, Parameter_Back);

		Temp_Point_1.x = Remove_Point_After->points[0].x;
		Temp_Point_1.y = Remove_Point_After->points[0].y;
		Temp_Point_1.z = Remove_Point_After->points[0].z;

		Remove_Point_Before->points.resize(0);
		Remove_Point_After->points.resize(0);

		cout << "第" << i + 1 << "个点旋转完成" << endl;

		pcl::PointXYZ p_2;

		p_2.x = All_IntersectPoint_2->points[n].x;
		p_2.y = All_IntersectPoint_2->points[n].y;
		p_2.z = All_IntersectPoint_2->points[n].z;

		Remove_Point_Before->points.push_back(p_2);

		SeamRevolve(Remove_Point_Before, Remove_Point_After, Parameter_Back);

		Temp_Point_2.x = Remove_Point_After->points[0].x;
		Temp_Point_2.y = Remove_Point_After->points[0].y;
		Temp_Point_2.z = Remove_Point_After->points[0].z;

		Remove_Point_Before->points.resize(0);
		Remove_Point_After->points.resize(0);

		cout << "第" << i + 1 << "个点的对应点旋转完成" << endl;

		Temp_Point.push_back(Temp_Point_1);
		Temp_Point.push_back(Temp_Point_2);


		cout << "第" << i + 1 << "组的点为:(" << Temp_Point_1.x << "," << Temp_Point_1.y << "，" << Temp_Point_1.z << ")" << endl;
		cout << "第" << i + 1 << "组的点为:(" << Temp_Point_2.x << "," << Temp_Point_2.y << "，" << Temp_Point_2.z << ")" << endl;

		End_Point_All_3.push_back(Temp_Point);

		Temp_Point.resize(0);

	}

	std::cout << "完成交点对应" << endl;
	std::cout << "共有" << End_Point_All_3.size() << "组" << endl;

	//for (int i = 0;i < End_Point_All_3.size();i++)
	//{
		//cout << "第" << i + 1 << "组的点为:(" << End_Point_All_3[i][0].x << "," << End_Point_All_3[i][0].y <<"，" << End_Point_All_3[i][0].z << ")" << endl;
		//cout << "第" << i + 1 << "组的点为:(" << End_Point_All_3[i][1].x << "," << End_Point_All_3[i][1].y << "，" << End_Point_All_3[i][1].z << ")" << endl;
	//}

	//要开始将交点都旋转了


	//连接交线
	// ============ 第三步：连接每对端点，生成可视化棱线 ============

	for (int i = 0;i < End_Point_All_3.size();i++)
	{

		for (double t = 0;t <= 1;t += 0.1)
		{
			pcl::PointXYZ p;

			p.x = End_Point_All_3[i][0].x + t * (End_Point_All_3[i][1].x - End_Point_All_3[i][0].x);
			p.y = End_Point_All_3[i][0].y + t * (End_Point_All_3[i][1].y - End_Point_All_3[i][0].y);
			p.z = End_Point_All_3[i][0].z + t * (End_Point_All_3[i][1].z - End_Point_All_3[i][0].z);

			End_Show_3->points.push_back(p);

		}

		std::cout << "连接完成一条棱" << endl;
	}
}