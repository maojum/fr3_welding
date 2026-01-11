#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 根据法向量旋转点云
 * @details 目的：将点云从一个坐标系旋转到另一个坐标系，使得原始平面的法向量旋转到目标法向量方向。
 *          使用轴角表示法(Angle-Axis)计算旋转矩阵，通过两个法向量的夹角和旋转轴来进行变换。
 * @param All_cloud 输入：待旋转的点云
 * @param All_cloud_Revolve 输出：旋转后的点云
 * @param Parameter 旋转参数，包含原始法向量(a,b,c)和目标法向量(d,e,f)
 */
void SeamRevolve(pcl::PointCloud<pcl::PointXYZ>::Ptr All_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& All_cloud_Revolve, Revolve Parameter)
{
	// ============ 提取旋转参数中的两个法向量 ============
	
	// 原始平面的法向量 (归一化后的)
	double a = Parameter.a;
	double b = Parameter.b;
	double c = Parameter.c;

	// 目标平面的法向量 (归一化后的)
	double d = Parameter.d;
	double e = Parameter.e;
	double f = Parameter.f;

	// ============ 第一步：定义法向量 ============
	
	// Standard_norm: 原始平面的法向量（要旋转的向量）
	cv::Point3d Standard_norm(a, b, c);
	
	// Original_norm: 目标平面的法向量（旋转后要达到的方向）
	cv::Point3d Original_norm(d, e, f);
	
	std::cout << "要旋转的向量: " << Standard_norm << std::endl;
	std::cout << "旋转后的目标向量: " << Original_norm << std::endl;

	// ============ 第二步：计算两个法向量之间的夹角 ============
	
	// 计算两个向量的点乘（内积）: v1·v2 = |v1||v2|cosθ
	double v1v2 = Standard_norm.dot(Original_norm);
	
	// 计算两个法向量的模长（因为都是归一化的，所以结果都是1）
	double v1_norm = Standard_norm.x * Standard_norm.x + Standard_norm.y * Standard_norm.y + Standard_norm.z * Standard_norm.z;
	double v2_norm = Original_norm.x * Original_norm.x + Original_norm.y * Original_norm.y + Original_norm.z * Original_norm.z;
	
	// 根据点乘公式计算夹角: θ = arccos(v1·v2 / (|v1||v2|))
	double theta = std::acos(v1v2 / (std::sqrt(v1_norm) * std::sqrt(v2_norm)));

	// ============ 第三步：计算旋转轴 ============
	// 通过叉乘得到同时垂直于两个法向量的向量，作为旋转轴
	// 旋转轴 = Original_norm × Standard_norm
	cv::Point3d axis_v1v2 = Original_norm.cross(Standard_norm);

	// 对旋转轴进行归一化处理，使其成为单位向量
	double v1v2_2 = axis_v1v2.x * axis_v1v2.x + axis_v1v2.y * axis_v1v2.y + axis_v1v2.z * axis_v1v2.z;
	double v1v2_n = std::sqrt(v1v2_2);
	axis_v1v2 = axis_v1v2 / v1v2_n;

	// ============ 第四步：构建旋转矩阵 ============
	// 使用 Eigen 的轴角表示法 (Angle-Axis) 构建旋转
	// 参数：旋转角度(-theta)，旋转轴(axis_v1v2)
	Eigen::AngleAxisd ro_vector(-theta, Eigen::Vector3d(axis_v1v2.x, axis_v1v2.y, axis_v1v2.z));
	
	// 将轴角表示转换为旋转矩阵
	Eigen::Matrix3d ro_matrix = ro_vector.toRotationMatrix();

	// ============ 第五步：对点云中的每个点应用旋转变换 ============
	
	std::vector<Eigen::Vector3d> new_points; // 存储旋转后的点（未使用）
	double sum = 0;  // 距离总和（用于调试，未使用）
	
	for (int i = 0; i < All_cloud->points.size(); i++)
	{
		Eigen::Vector3d newP;

		// 获取当前点的坐标
		double x = All_cloud->points[i].x;
		double y = All_cloud->points[i].y;
		double z = All_cloud->points[i].z;

		// 计算每个点到拟合平面的距离（用于调试，实际未使用）
		double dist2 = (a * x + b * y + c * z + d) * (a * x + b * y + c * z + d);
		sum = +std::sqrt(dist2);
		
		// 构建当前点的向量
		newP.x() = x;
		newP.y() = y;
		newP.z() = z;

		// 应用旋转矩阵：new_point = R * P
		Eigen::Vector3d new_point = ro_matrix * newP;

		// 将旋转后的点转换为 PCL 点格式并存储
		pcl::PointXYZ p;
		p.x = new_point.x();
		p.y = new_point.y();
		p.z = new_point.z();

		All_cloud_Revolve->points.push_back(p);
	}
}