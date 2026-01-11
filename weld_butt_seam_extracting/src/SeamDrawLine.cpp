#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 绘制两平面的交线（用点云表示）
 * @details 目的：根据交线参数（点+方向向量）生成交线的点云表示，并判断两个平面是否平行。
 *          如果两平面平行，交线退化为一个点，舍弃；如果相交，保留完整交线点云。
 * @param coefficients_line 输入：交线参数，包含直线上一点(x0,y0,z0)和方向向量(a,b,c)共6个值
 * @param cloud_line_max 输出：交线的点云表示，如果两平面平行则为空
 */
void SeamDrawLine(pcl::ModelCoefficients::Ptr coefficients_line, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_line_max)
{
	// ============ 第一步：提取直线参数 ============
	// 直线的参数方程：P(t) = P0 + t*V
	// 其中 P0 = (x0, y0, z0) 是直线上的一点
	//      V = (a, b, c) 是直线的方向向量
	//      t 是参数

	Point_3D p; // 直线上的已知点 P0 = (x0, y0, z0)
	Point_3D n; // 直线的方向向量 V = (a, b, c)

	p.x = coefficients_line->values[0]; // x0：直线上点的x坐标
	p.y = coefficients_line->values[1]; // y0：直线上点的y坐标
	p.z = coefficients_line->values[2]; // z0：直线上点的z坐标

	n.x = coefficients_line->values[3]; // a：方向向量的x分量
	n.y = coefficients_line->values[4]; // b：方向向量的y分量
	n.z = coefficients_line->values[5]; // c：方向向量的z分量

	// ============ 第二步：构造交线的点云 ============
	// 方法：通过参数方程在 t ∈ [-10000, 10000] 范围内生成点
	// 步长为1，共生成约20000个点

	pcl::PointXYZ q;  // 临时点，用于存储参数方程计算的每个点

	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_Filt(new pcl::PointCloud<pcl::PointXYZ>());  // 临时点云，存储构造的直线

	// 第一个循环：生成 t ∈ [0, 10000) 范围的点（正方向）
	for (double t = 0; t < 10000; t += 1)
	{
		// 参数方程：P(t) = P0 + t*V
		q.x = p.x + t * n.x;
		q.y = p.y + t * n.y;
		q.z = p.z + t * n.z;

		Temp_Filt->points.push_back(q);
	}

	// 第二个循环：生成 t ∈ (-10000, 0) 范围的点（负方向）
	for (double t = 0; t > -10000; t -= 1)
	{
		// 参数方程：P(t) = P0 + t*V
		q.x = p.x + t * n.x;
		q.y = p.y + t * n.y;
		q.z = p.z + t * n.z;

		Temp_Filt->points.push_back(q);
	}

	cout << "直线构造完成" << endl;

	/*
	// 调试用：保存临时构造的直线点云到文件
	Temp_Filt->width = Temp_Filt->points.size();
	Temp_Filt->height = 1;
	pcl::io::savePCDFile("/tmp/Temp_Filt.pcd", *Temp_Filt);
	*/

	// ============ 第三步：判断两平面是否平行 ============
	// 原理：如果两平面平行，它们没有交线，SeamIntersectLine计算出的"方向向量"实际为零向量
	//       此时参数方程 P(t) = P0 + t*0 = P0，无论 t 取何值，都是同一个点
	//       所以构造出的所有点都重合在一起
	// 检测方法：计算直线两端点的距离，如果距离小于5mm，认为是同一个点（平行面）

	// 获取直线的第一个点（t = 0 附近）
	double a_1 = 0;
	double b_1 = 0;
	double c_1 = 0;

	a_1 = Temp_Filt->points[0].x;  // 第一个点的x坐标
	b_1 = Temp_Filt->points[0].y;  // 第一个点的y坐标
	c_1 = Temp_Filt->points[0].z;  // 第一个点的z坐标

	cout << "直线上的其中一点为(" << a_1 << "," << b_1 << "," << c_1 << ")" << endl;

	// 获取直线的最后一个点（t = -10000 附近）
	double a_2 = 0;
	double b_2 = 0;
	double c_2 = 0;

	a_2 = Temp_Filt->points[Temp_Filt->points.size() - 1].x;  // 最后一个点的x坐标
	b_2 = Temp_Filt->points[Temp_Filt->points.size() - 1].y;  // 最后一个点的y坐标
	c_2 = Temp_Filt->points[Temp_Filt->points.size() - 1].z;  // 最后一个点的z坐标

	cout << "直线上的其中另外一点为(" << a_2 << "," << b_2 << "," << c_2 << ")" << endl;

	// 计算两端点之间的欧式距离
	double dist;
	dist = sqrt(pow(a_1 - a_2, 2) + pow(b_1 - b_2, 2) + pow(c_1 - c_2, 2));

	cout << "直线的长度为" << dist << endl;

	// ============ 第四步：根据距离判断是否保留交线 ============
	if (dist < 5)  // 距离小于5mm，说明首尾点重合，两平面平行
	{
		cout << "说明这两个面是平行面,构造的直线不是交线，舍去" << endl;
		// cloud_line_max 保持为空，不添加任何点
	}
	else  // 距离大于等于5mm，说明是真正的交线
	{
		// 将临时点云中的所有点复制到输出点云
		for (int i = 0; i < Temp_Filt->points.size(); i++)
		{
			pcl::PointXYZ p;

			p.x = Temp_Filt->points[i].x;
			p.y = Temp_Filt->points[i].y;
			p.z = Temp_Filt->points[i].z;

			cloud_line_max->points.push_back(p);  // 添加到输出点云
		}
	}
}


