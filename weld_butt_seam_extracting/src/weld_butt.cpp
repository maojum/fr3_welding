#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 将点云坐标转换到机械臂基坐标系（含轴交换与偏移）
 * @details 坐标变换规则：
 *          - x' = -x + offset_x （原x取反并平移到目标系x轴）
 *          - y' = -y + offset_y （原y取反并平移到目标系y轴）
 *          - z' =  z + offset_z （原z平移到目标系z轴）
 *          其中 offset_* 为手眼标定得到的平移量，单位与点云一致（mm）。
 * @param input_cloud 输入：原始点云（待变换坐标系）
 * @param transformed_cloud 输出：变换后的点云（机械臂基坐标系）
 */
void transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
						 pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud)
{
	for (const auto &point : input_cloud->points)
	{
		pcl::PointXYZ transformed_point;

		// 工件相对于机械臂基座坐标系的平移偏置（单位：mm）
		const double offset_x = -400;
		const double offset_y = -720.0;
		const double offset_z = 0;

		// 坐标变换：先取反再加入平移，保证落在机械臂基坐标系下正确象限
		transformed_point.x = point.x + offset_x;
		transformed_point.y = point.y + offset_y;
		transformed_point.z = point.z + offset_z;
		
		transformed_cloud->points.push_back(transformed_point);
	}
}

/**
 * @brief 获取对接焊缝轨迹
 * @details 从PCD文件中提取对接焊缝，并返回焊缝轨迹位姿序列
 * @param pcd_file_path PCD点云文件路径
 * @return 焊缝轨迹位姿向量
 * @note 返回轨迹供调用者发布，而非直接传入Publisher
 */
std::vector<geometry_msgs::msg::Pose> get_butt_weld_seam(const std::string &pcd_file_path)
{
	// ============ 第0步：读取输入点云 ============
	// 期望：pcd_file_path 指向对接焊缝的原始点云（机器人基坐标系或相机坐标系）
	// 输出：All_cloud 作为后续处理的基础点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr All_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 输入的原始点云

	if (pcl::io::loadPCDFile(pcd_file_path, *All_cloud) < 0)
	{
		PCL_ERROR("\a->点云文件不存在！\n");
		return std::vector<geometry_msgs::msg::Pose>();
	}

	std::cout << "对缝点云加载完成" << endl;
	std::cout << "对缝点云中共有" << All_cloud->points.size() << "个点" << endl;

	// ============ 第1步：坐标系变换并保存 ============
	// 目的：将点云转换到机械臂基坐标系，便于后续规划和标定一致。
	// 结果：保存到 /tmp/transformed_All_cloud.pcd 供调试和可视化。
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_All_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformPointCloud(All_cloud, transformed_All_cloud);
	transformed_All_cloud->width = transformed_All_cloud->size();
	transformed_All_cloud->height = 1;
	pcl::io::savePCDFile("/tmp/transformed_All_cloud.pcd", *transformed_All_cloud);

	// ============ 第2步：整体平面分割 ============
	// 目的：把原始点云分割成多个平面，并记录各平面的法向量系数。
	// 输出：All_Plane_Point 存储各平面的点；All_Factor 存储对应的平面系数。
	vector<vector<Point_3D>> All_Plane_Point;
	vector<Leading_Factor> All_Factor;

	SeamSepAllPlane(All_cloud, All_Plane_Point, All_Factor);

	cout << "将平面分割开，将每个平面的法向量单独保存起来" << endl;

	// ============ 第3步：按法向量聚类平面 ============
	// 目的：根据法向量方向对所有平面做聚类，便于识别成对的对接面。
	// 输出：All_Plane_Cluster 为分组后的平面集合；New_All_Factor 为对应平面法向量集合。
	vector<vector<vector<Point_3D>>> All_Plane_Cluster;
	vector<Leading_Factor> New_All_Factor;

	SeamCluAllPlane(All_Plane_Point, All_Factor, All_Plane_Cluster, New_All_Factor);

	cout << "平面被分成了" << All_Plane_Cluster.size() << "类" << endl;

	// ============ 第4步：对聚类结果降采样 ============
	// 目的：降低点数以加快后续运算；保留几何形状。
	vector<vector<vector<Point_3D>>> Dwon_All_Plane_Cluster;

	SeamDownSmape(All_Plane_Cluster, Dwon_All_Plane_Cluster);

	cout << "平面共分成了" << Dwon_All_Plane_Cluster.size() << "类" << endl;

	// ============ 第5步：在降采样平面中找到焊缝的两侧面 ============
	// 输出：target 为类别索引；target_1/target_2 为该类内的两个焊缝平面序号。
	int target;
	int target_1;
	int target_2;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_1(new pcl::PointCloud<pcl::PointXYZ>); // 目标焊缝平面1
	pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Plane_2(new pcl::PointCloud<pcl::PointXYZ>); // 目标焊缝平面2

	SeamTwoPlane(Dwon_All_Plane_Cluster, target, target_1, target_2);

	cout << "找到焊缝的两个面对应的类和类里的序号" << endl;
	cout << "第" << target + 1 << "组的第" << target_1 + 1 << "个平面和第" << target_2 + 1 << "个平面是焊缝平面" << endl;

	// 将目标两平面从聚类结果拷贝到新的点云对象，便于单独处理和保存。
	for (int i = 0; i < All_Plane_Cluster[target][target_1].size(); i++)
	{
		pcl::PointXYZ p_1;
		p_1.x = All_Plane_Cluster[target][target_1][i].x;
		p_1.y = All_Plane_Cluster[target][target_1][i].y;
		p_1.z = All_Plane_Cluster[target][target_1][i].z;

		Target_Plane_1->points.push_back(p_1);
	}

	for (int j = 0; j < All_Plane_Cluster[target][target_2].size(); j++)
	{
		pcl::PointXYZ p_2;
		p_2.x = All_Plane_Cluster[target][target_2][j].x;
		p_2.y = All_Plane_Cluster[target][target_2][j].y;
		p_2.z = All_Plane_Cluster[target][target_2][j].z;

		Target_Plane_2->points.push_back(p_2);
	}

	Target_Plane_1->width = Target_Plane_1->points.size();
	Target_Plane_1->height = 1;
	pcl::io::savePCDFile("/tmp/Target_Plane_1.pcd", *Target_Plane_1);

	Target_Plane_2->width = Target_Plane_2->points.size();
	Target_Plane_2->height = 1;
	pcl::io::savePCDFile("/tmp/Target_Plane_2.pcd", *Target_Plane_2);

	cout << "找到焊缝的两个面" << endl;
	cout << "已经将两个焊缝平面存进点云对象中" << endl;

	// ============ 第6步：对焊缝两侧平面做局部降采样 ============
	// 目的：既保留焊缝面又减小点数，Down_Target_Plane_* 用于后续邻面搜索。
	pcl::PointCloud<pcl::PointXYZ>::Ptr Down_Target_Plane_1(new pcl::PointCloud<pcl::PointXYZ>); // 对焊接平面1降采样
	pcl::PointCloud<pcl::PointXYZ>::Ptr Down_Target_Plane_2(new pcl::PointCloud<pcl::PointXYZ>); // 对焊接平面2降采样

	vector<vector<Point_3D>> Down_Smape_All;

	SeamDownSmapeT(All_Plane_Point, Down_Smape_All, Target_Plane_1, Target_Plane_2, Down_Target_Plane_1, Down_Target_Plane_2);

	cout << "降采样完的面一共有" << Down_Smape_All.size() << "个" << endl;
	cout << "第一个焊接面降采样后的点有" << Down_Target_Plane_1->points.size() << "个" << endl;
	cout << "第二个焊接面降采样后的点有" << Down_Target_Plane_2->points.size() << "个" << endl;

	// ============ 第7步：寻找与焊缝平面1相邻的四个面 ============
	// More_Index_1 初筛相邻面索引；Accurate_Index_1 精筛后保留真正相邻面。
	vector<double> More_Index_1;

	SeamFourPlane(Down_Smape_All, Down_Target_Plane_1, More_Index_1);

	cout << "找到与第一个焊接面相连的面的下标，共有" << More_Index_1.size() << "个" << endl;

	vector<double> Accurate_Index_1;

	SeamFindSelf(Target_Plane_1, All_Factor, More_Index_1, Accurate_Index_1);

	cout << "找到与第一个面相连的面的下标，共有" << Accurate_Index_1.size() << "个" << endl;

	// 收集相邻四个面的原始点云，便于后续旋转求交线
	vector<vector<Point_3D>> Four_Like_Plane_1;
	vector<Point_3D> Four_Temp_Cloud_1;

	for (int l = 0; l < Accurate_Index_1.size(); l++)
	{
		for (int d = 0; d < All_Plane_Point[Accurate_Index_1[l]].size(); d++)
		{
			Point_3D P;

			P.x = All_Plane_Point[Accurate_Index_1[l]][d].x;
			P.y = All_Plane_Point[Accurate_Index_1[l]][d].y;
			P.z = All_Plane_Point[Accurate_Index_1[l]][d].z;

			Four_Temp_Cloud_1.push_back(P);
		}

		Four_Like_Plane_1.push_back(Four_Temp_Cloud_1);
		cout << Four_Temp_Cloud_1.size() << endl;
		Four_Temp_Cloud_1.resize(0);
	}

	cout << "与焊接平面1相连的四个面已经放入新的容器中" << endl;
	cout << Four_Like_Plane_1.size() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_1(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i < Four_Like_Plane_1.size(); i++)
	{
		for (int j = 0; j < Four_Like_Plane_1[i].size(); j++)
		{
			pcl::PointXYZ p;
			p.x = Four_Like_Plane_1[i][j].x;
			p.y = Four_Like_Plane_1[i][j].y;
			p.z = Four_Like_Plane_1[i][j].z;

			Temp_1->points.push_back(p);
		}

		cout << "与焊接平面1相连的第" << i + 1 << "个平面有" << Temp_1->points.size() << "个点" << endl;

		std::stringstream ss;
		pcl::PCDWriter writer;

		Temp_1->width = Temp_1->points.size();
		Temp_1->height = 1;
		writer.write<pcl::PointXYZ>("/tmp/Link_1_" + to_string(i + 1) + ".pcd", *Temp_1, false);

		Temp_1->points.resize(0);
	}

	std::cout << "----------------------------------------" << endl;

	// ============ 第8步：旋转焊缝平面1到标准姿态 ============
	// 目的：把平面旋转到便于求交线的姿态；Parameter 为旋转参数。
	Revolve Parameter;

	Parameter.a = 0;
	Parameter.b = 0;
	Parameter.c = 1;
	Parameter.d = 1;
	Parameter.e = 1;
	Parameter.f = 1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Revolve_Target_Plane_1(new pcl::PointCloud<pcl::PointXYZ>);

	SeamRevolve(Target_Plane_1, Revolve_Target_Plane_1, Parameter);

	Revolve_Target_Plane_1->width = Revolve_Target_Plane_1->points.size();
	Revolve_Target_Plane_1->height = 1;
	pcl::io::savePCDFile("/tmp/Revolve_Target_Plane_1.pcd", *Revolve_Target_Plane_1);

	cout << "第一个平面已经旋转完成" << endl;
	std::cout << "----------------------------------------" << endl;

	// ============ 第9步：求旋转后平面的系数并与相邻面求交线 ============
	// Factor_First 为焊缝面旋转后的平面系数，用于与相邻面求交。
	Leading_Factor Factor_First = {0, 0, 0, 0};

	SeamCoefficient(Revolve_Target_Plane_1, Factor_First);

	cout << "第一个平面旋转后的相关系数已经全部找到" << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_Cloud_1(new pcl::PointCloud<pcl::PointXYZ>);

	vector<vector<Point_3D>> All_Line_1;
	vector<Point_3D> Line_1;

	for (int i = 0; i < Four_Like_Plane_1.size(); i++)
	{
		for (int j = 0; j < Four_Like_Plane_1[i].size(); j++)
		{
			pcl::PointXYZ p;

			p.x = Four_Like_Plane_1[i][j].x;
			p.y = Four_Like_Plane_1[i][j].y;
			p.z = Four_Like_Plane_1[i][j].z;

			Temp_Cloud_1->points.push_back(p);
		}

		// 将相邻面旋转到同一坐标系，再与焊缝面求交线
		pcl::PointCloud<pcl::PointXYZ>::Ptr Revolve_Temp_Cloud_1(new pcl::PointCloud<pcl::PointXYZ>);

		SeamRevolve(Temp_Cloud_1, Revolve_Temp_Cloud_1, Parameter);

		std::stringstream ss;
		pcl::PCDWriter writer;

		Revolve_Temp_Cloud_1->width = Revolve_Temp_Cloud_1->points.size();
		Revolve_Temp_Cloud_1->height = 1;
		writer.write<pcl::PointXYZ>("/tmp/Revolve_Temp_Cloud_1_" + to_string(i + 1) + ".pcd", *Revolve_Temp_Cloud_1, false);

		cout << "完成旋转" << endl;
		std::cout << "----------------------------------------" << endl;

		Leading_Factor Factor_Second = {0, 0, 0, 0};

		SeamCoefficient(Revolve_Temp_Cloud_1, Factor_Second);

		// 求两平面交线并写入文件，Line_1 存储交线上离散点
		Revolve_Temp_Cloud_1->points.resize(0);
		Temp_Cloud_1->points.resize(0);

		pcl::ModelCoefficients::Ptr coefficients_line(new pcl::ModelCoefficients);

		SeamIntersectLine(Factor_First, Factor_Second, coefficients_line);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_max(new pcl::PointCloud<pcl::PointXYZ>());

		SeamDrawLine(coefficients_line, cloud_line_max);

		for (int i = 0; i < cloud_line_max->points.size(); i++)
		{
			Point_3D Point;

			Point.x = cloud_line_max->points[i].x;
			Point.y = cloud_line_max->points[i].y;
			Point.z = cloud_line_max->points[i].z;

			Line_1.push_back(Point);
		}

		All_Line_1.push_back(Line_1);
		Line_1.resize(0);

		cloud_line_max->width = cloud_line_max->points.size();
		cloud_line_max->height = 1;
		writer.write<pcl::PointXYZ>("/tmp/IntersectLine_" + to_string(1) + "_" + to_string(i + 1) + ".pcd", *cloud_line_max, false);

		std::cout << "得到第一个焊接平面与相交四平面中第" << i + 1 << "个平面的交线" << endl;
	}

	cout << "得到所有的相交直线,共有" << All_Line_1.size() << "条" << endl;
	std::cout << "----------------------------------------" << endl;

	// ============ 第10步：过滤交线点，限制在焊缝面包围盒内 ============
	pcl::PointXYZ x_min_1 = *std::min_element(Revolve_Target_Plane_1->begin(), Revolve_Target_Plane_1->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.x < pt2.x; });
	pcl::PointXYZ x_max_1 = *std::max_element(Revolve_Target_Plane_1->begin(), Revolve_Target_Plane_1->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.x < pt2.x; });
	pcl::PointXYZ y_min_1 = *std::min_element(Revolve_Target_Plane_1->begin(), Revolve_Target_Plane_1->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.y < pt2.y; });
	pcl::PointXYZ y_max_1 = *std::max_element(Revolve_Target_Plane_1->begin(), Revolve_Target_Plane_1->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.y < pt2.y; });
	pcl::PointXYZ z_min_1 = *std::min_element(Revolve_Target_Plane_1->begin(), Revolve_Target_Plane_1->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.z < pt2.z; });
	pcl::PointXYZ z_max_1 = *std::max_element(Revolve_Target_Plane_1->begin(), Revolve_Target_Plane_1->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.z < pt2.z; });

	vector<double> MIXMAX_1;
	MIXMAX_1.push_back(x_min_1.x);
	MIXMAX_1.push_back(x_max_1.x);
	MIXMAX_1.push_back(y_min_1.y);
	MIXMAX_1.push_back(y_max_1.y);
	MIXMAX_1.push_back(z_min_1.z);
	MIXMAX_1.push_back(z_max_1.z);

	cout << "找到旋转后长方体的各个边界坐标" << endl;

	vector<vector<Point_3D>> Filt_All_Line_1;
	vector<Point_3D> Filt_Line_1;

	for (int i = 0; i < All_Line_1.size(); i++)
	{
		for (int j = 0; j < All_Line_1[i].size(); j++)
		{
			if (!(All_Line_1[i][j].x < (MIXMAX_1[0] - abs(2 * MIXMAX_1[0])) || All_Line_1[i][j].x > (MIXMAX_1[1] + abs(2 * MIXMAX_1[1])) || All_Line_1[i][j].y < (MIXMAX_1[2] - abs(2 * MIXMAX_1[2])) || All_Line_1[i][j].y > (MIXMAX_1[3] + abs(2 * MIXMAX_1[3]))))
			{
				Point_3D p;
				p.x = All_Line_1[i][j].x;
				p.y = All_Line_1[i][j].y;
				p.z = All_Line_1[i][j].z;

				Filt_Line_1.push_back(p);
			}
		}

		Filt_All_Line_1.push_back(Filt_Line_1);
		std::cout << "第" << i + 1 << "条交线过滤完" << endl;
		Filt_Line_1.resize(0);
	}

	cout << "所有的相交直线过滤完成" << endl;
	std::cout << "----------------------------------------" << endl;

	// ============ 第11步：求交线之间的交点 ============
	pcl::PointCloud<pcl::PointXYZ>::Ptr All_IntersectPoint_1(new pcl::PointCloud<pcl::PointXYZ>());

	SeamAllIntersectPoint(Filt_All_Line_1, All_IntersectPoint_1);

	std::cout << "此时交点容器中共有" << All_IntersectPoint_1->points.size() << "个点" << endl;

	All_IntersectPoint_1->width = All_IntersectPoint_1->points.size();
	All_IntersectPoint_1->height = 1;
	pcl::io::savePCDFile("/tmp/All_IntersectPoint_1.pcd", *All_IntersectPoint_1);

	std::cout << "----------------------------------------" << endl;

	// ============ 第12步：生成焊缝平面1的最终展示点云 ============
	pcl::PointCloud<pcl::PointXYZ>::Ptr End_Show_1(new pcl::PointCloud<pcl::PointXYZ>());
	vector<vector<Point_3D>> End_Point_All_1;

	SeamEndShow(Filt_All_Line_1, All_IntersectPoint_1, End_Show_1, End_Point_All_1);

	cout << "完成第一个焊接面最后的展示" << endl;

	End_Show_1->width = End_Show_1->points.size();
	End_Show_1->height = 1;
	pcl::io::savePCDFile("/tmp/End_Show_1.pcd", *End_Show_1);

	std::cout << "----------------------------------------" << endl;
	std::cout << "----------------------------------------" << endl;
	std::cout << "----------------------------------------" << endl;

	// ============ 第13步：处理焊缝平面2（流程同平面1） ============
	vector<double> More_Index_2;

	SeamFourPlane(Down_Smape_All, Down_Target_Plane_2, More_Index_2);

	cout << "找到与第二个焊接面相连的面的下标，共有" << More_Index_2.size() << "个" << endl;

	vector<double> Accurate_Index_2;

	SeamFindSelf(Target_Plane_2, All_Factor, More_Index_2, Accurate_Index_2);

	cout << "找到与第二个面相连的面的下标，共有" << Accurate_Index_2.size() << "个" << endl;

	vector<vector<Point_3D>> Four_Like_Plane_2;
	vector<Point_3D> Four_Temp_Cloud_2;

	for (int l = 0; l < Accurate_Index_2.size(); l++)
	{
		for (int d = 0; d < All_Plane_Point[Accurate_Index_2[l]].size(); d++)
		{
			Point_3D P;

			P.x = All_Plane_Point[Accurate_Index_2[l]][d].x;
			P.y = All_Plane_Point[Accurate_Index_2[l]][d].y;
			P.z = All_Plane_Point[Accurate_Index_2[l]][d].z;

			Four_Temp_Cloud_2.push_back(P);
		}

		Four_Like_Plane_2.push_back(Four_Temp_Cloud_2);
		cout << Four_Temp_Cloud_2.size() << endl;
		Four_Temp_Cloud_2.resize(0);
	}

	cout << "与焊接平面2相连的四个面已经放入新的容器中" << endl;
	cout << Four_Like_Plane_2.size() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_2(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i < Four_Like_Plane_2.size(); i++)
	{
		for (int j = 0; j < Four_Like_Plane_2[i].size(); j++)
		{
			pcl::PointXYZ p;
			p.x = Four_Like_Plane_2[i][j].x;
			p.y = Four_Like_Plane_2[i][j].y;
			p.z = Four_Like_Plane_2[i][j].z;

			Temp_2->points.push_back(p);
		}

		cout << "与焊接平面2相连的第" << i + 1 << "个平面有" << Temp_2->points.size() << "个点" << endl;

		std::stringstream ss;
		pcl::PCDWriter writer;

		Temp_2->width = Temp_2->points.size();
		Temp_2->height = 1;
		writer.write<pcl::PointXYZ>("/tmp/Link_2_" + to_string(i + 1) + ".pcd", *Temp_2, false);

		Temp_2->points.resize(0);
	}

	std::cout << "----------------------------------------" << endl;

	// 旋转第二个平面
	pcl::PointCloud<pcl::PointXYZ>::Ptr Revolve_Target_Plane_2(new pcl::PointCloud<pcl::PointXYZ>);

	SeamRevolve(Target_Plane_2, Revolve_Target_Plane_2, Parameter);

	Revolve_Target_Plane_2->width = Revolve_Target_Plane_2->points.size();
	Revolve_Target_Plane_2->height = 1;
	pcl::io::savePCDFile("/tmp/Revolve_Target_Plane_2.pcd", *Revolve_Target_Plane_2);

	cout << "第二个平面已经旋转完成" << endl;
	std::cout << "----------------------------------------" << endl;

	Factor_First = {0, 0, 0, 0};

	SeamCoefficient(Revolve_Target_Plane_2, Factor_First);

	cout << "第二个平面旋转后的相关系数已经全部找到" << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Temp_Cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

	vector<vector<Point_3D>> All_Line_2;
	vector<Point_3D> Line_2;

	for (int i = 0; i < Four_Like_Plane_2.size(); i++)
	{
		for (int j = 0; j < Four_Like_Plane_2[i].size(); j++)
		{
			pcl::PointXYZ p;

			p.x = Four_Like_Plane_2[i][j].x;
			p.y = Four_Like_Plane_2[i][j].y;
			p.z = Four_Like_Plane_2[i][j].z;

			Temp_Cloud_2->points.push_back(p);
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr Revolve_Temp_Cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

		SeamRevolve(Temp_Cloud_2, Revolve_Temp_Cloud_2, Parameter);

		std::stringstream ss;
		pcl::PCDWriter writer;

		Revolve_Temp_Cloud_2->width = Revolve_Temp_Cloud_2->points.size();
		Revolve_Temp_Cloud_2->height = 1;
		writer.write<pcl::PointXYZ>("/tmp/Revolve_Temp_Cloud_2_" + to_string(i + 1) + ".pcd", *Revolve_Temp_Cloud_2, false);

		cout << "完成旋转" << endl;
		std::cout << "----------------------------------------" << endl;

		Leading_Factor Factor_Second = {0, 0, 0, 0};

		SeamCoefficient(Revolve_Temp_Cloud_2, Factor_Second);

		cout << "第二个平面旋转后的相关系数已经全部找到" << endl;

		Revolve_Temp_Cloud_2->points.resize(0);
		Temp_Cloud_2->points.resize(0);

		pcl::ModelCoefficients::Ptr coefficients_line(new pcl::ModelCoefficients);

		SeamIntersectLine(Factor_First, Factor_Second, coefficients_line);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_max(new pcl::PointCloud<pcl::PointXYZ>());

		SeamDrawLine(coefficients_line, cloud_line_max);

		for (int i = 0; i < cloud_line_max->points.size(); i++)
		{
			Point_3D Point;

			Point.x = cloud_line_max->points[i].x;
			Point.y = cloud_line_max->points[i].y;
			Point.z = cloud_line_max->points[i].z;

			Line_2.push_back(Point);
		}

		All_Line_2.push_back(Line_2);
		Line_2.resize(0);

		cloud_line_max->width = cloud_line_max->points.size();
		cloud_line_max->height = 1;
		writer.write<pcl::PointXYZ>("/tmp/IntersectLine_" + to_string(2) + "_" + to_string(i + 1) + ".pcd", *cloud_line_max, false);

		std::cout << "得到第二个焊接平面与相交四平面中第" << i + 1 << "个平面的交线" << endl;
	}

	cout << "得到所有的相交直线,共有" << All_Line_2.size() << "条" << endl;
	std::cout << "----------------------------------------" << endl;

	// 过滤第二个平面的交线
	pcl::PointXYZ x_min_2 = *std::min_element(Revolve_Target_Plane_2->begin(), Revolve_Target_Plane_2->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.x < pt2.x; });
	pcl::PointXYZ x_max_2 = *std::max_element(Revolve_Target_Plane_2->begin(), Revolve_Target_Plane_2->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.x < pt2.x; });
	pcl::PointXYZ y_min_2 = *std::min_element(Revolve_Target_Plane_2->begin(), Revolve_Target_Plane_2->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.y < pt2.y; });
	pcl::PointXYZ y_max_2 = *std::max_element(Revolve_Target_Plane_2->begin(), Revolve_Target_Plane_2->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.y < pt2.y; });
	pcl::PointXYZ z_min_2 = *std::min_element(Revolve_Target_Plane_2->begin(), Revolve_Target_Plane_2->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.z < pt2.z; });
	pcl::PointXYZ z_max_2 = *std::max_element(Revolve_Target_Plane_2->begin(), Revolve_Target_Plane_2->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
											  { return pt1.z < pt2.z; });

	vector<double> MIXMAX_2;
	MIXMAX_2.push_back(x_min_2.x);
	MIXMAX_2.push_back(x_max_2.x);
	MIXMAX_2.push_back(y_min_2.y);
	MIXMAX_2.push_back(y_max_2.y);
	MIXMAX_2.push_back(z_min_2.z);
	MIXMAX_2.push_back(z_max_2.z);

	cout << "找到旋转后长方体的各个边界坐标" << endl;

	vector<vector<Point_3D>> Filt_All_Line_2;
	vector<Point_3D> Filt_Line_2;

	for (int i = 0; i < All_Line_2.size(); i++)
	{
		for (int j = 0; j < All_Line_2[i].size(); j++)
		{
			if (!(All_Line_2[i][j].x < (MIXMAX_2[0] - abs(2 * MIXMAX_2[0])) || All_Line_2[i][j].x > (MIXMAX_2[1] + abs(2 * MIXMAX_1[1])) || All_Line_2[i][j].y < (MIXMAX_2[2] - abs(2 * MIXMAX_2[2])) || All_Line_2[i][j].y > (MIXMAX_2[3] + abs(2 * MIXMAX_2[3]))))
			{
				Point_3D p;
				p.x = All_Line_2[i][j].x;
				p.y = All_Line_2[i][j].y;
				p.z = All_Line_2[i][j].z;

				Filt_Line_2.push_back(p);
			}
		}

		Filt_All_Line_2.push_back(Filt_Line_2);
		std::cout << "第" << i + 1 << "条交线过滤完" << endl;
		Filt_Line_2.resize(0);
	}

	cout << "所有的相交直线过滤完成" << endl;
	std::cout << "----------------------------------------" << endl;

	// 找第二个焊接平面的交点
	pcl::PointCloud<pcl::PointXYZ>::Ptr All_IntersectPoint_2(new pcl::PointCloud<pcl::PointXYZ>());

	SeamAllIntersectPoint(Filt_All_Line_2, All_IntersectPoint_2);

	std::cout << "此时交点容器中共有" << All_IntersectPoint_2->points.size() << "个点" << endl;

	All_IntersectPoint_2->width = All_IntersectPoint_2->points.size();
	All_IntersectPoint_2->height = 1;
	pcl::io::savePCDFile("/tmp/All_IntersectPoint_2.pcd", *All_IntersectPoint_2);

	std::cout << "----------------------------------------" << endl;

	// 生成第二个焊缝平面的最终展示
	pcl::PointCloud<pcl::PointXYZ>::Ptr End_Show_2(new pcl::PointCloud<pcl::PointXYZ>());
	vector<vector<Point_3D>> End_Point_All_2;

	SeamEndShow(Filt_All_Line_2, All_IntersectPoint_2, End_Show_2, End_Point_All_2);

	cout << "完成第二个焊接面最后的展示" << endl;

	End_Show_2->width = End_Show_2->points.size();
	End_Show_2->height = 1;
	pcl::io::savePCDFile("/tmp/End_Show_2.pcd", *End_Show_2);

	std::cout << "----------------------------------------" << endl;
	std::cout << "----------------------------------------" << endl;
	std::cout << "----------------------------------------" << endl;

	// ============ 第14步：连接两侧焊缝平面，得到整体展示 ============
	pcl::PointCloud<pcl::PointXYZ>::Ptr End_Show_3(new pcl::PointCloud<pcl::PointXYZ>());
	vector<vector<Point_3D>> End_Point_All_3;

	SeamLinkTwo(All_IntersectPoint_1, All_IntersectPoint_2, End_Show_3, End_Point_All_3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr End_Show_12(new pcl::PointCloud<pcl::PointXYZ>());

	*End_Show_12 = *End_Show_1 + *End_Show_2;

	pcl::PointCloud<pcl::PointXYZ>::Ptr End_Show(new pcl::PointCloud<pcl::PointXYZ>());

	*End_Show = *End_Show_12 + *End_Show_3;

	End_Show_12->width = End_Show_12->points.size();
	End_Show_12->height = 1;
	pcl::io::savePCDFile("/tmp/End_Show_12.pcd", *End_Show_12);

	End_Show_3->width = End_Show_3->points.size();
	End_Show_3->height = 1;
	pcl::io::savePCDFile("/tmp/End_Show_3.pcd", *End_Show_3);

	End_Show->width = End_Show->points.size();
	End_Show->height = 1;
	pcl::io::savePCDFile("/tmp/End_Show.pcd", *End_Show);

	cout << "完成两个焊接面之间的相连" << endl;
	cout << "最终展示完成" << endl;

	// ============ 第15步：提取顶层点并计算焊缝中心线 ============
	// 规则：根据 z>=19 且按 x 分左右，将左右对应点取中点形成焊缝中心线。
	pcl::PointCloud<pcl::PointXYZ>::Ptr End_point_1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr End_point_2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr weld_point(new pcl::PointCloud<pcl::PointXYZ>());

	for (const auto &point : *End_Show_12)
	{
		if (point.z >= 19)
		{
			if (point.x <= 402)
			{
				End_point_1->push_back(point);
			}
			else
			{
				End_point_2->push_back(point);
			}
		}
	}
	for (size_t i = 0; i < End_point_1->points.size(); ++i)
	{
		pcl::PointXYZ new_point;
		new_point.x = (End_point_1->points[i].x + End_point_2->points[i].x) / 2.0;
		new_point.y = (End_point_1->points[i].y + End_point_2->points[i].y) / 2.0;
		new_point.z = (End_point_1->points[i].z + End_point_2->points[i].z) / 2.0;

		weld_point->points.push_back(new_point);
	}

	// 删除重复点（去 NaN + 体素滤波）
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*weld_point, *temp_cloud, indices); // 移除NaN点
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(temp_cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*weld_point);

	cout << "获得焊缝点云" << endl;

	End_point_1->width = End_point_1->points.size();
	End_point_1->height = 1;
	pcl::io::savePCDFile("/tmp/End_point_1.pcd", *End_point_1);

	End_point_2->width = End_point_2->points.size();
	End_point_2->height = 1;
	pcl::io::savePCDFile("/tmp/End_point_2.pcd", *End_point_2);

	weld_point->width = weld_point->points.size();
	weld_point->height = 1;
	pcl::io::savePCDFile("/tmp/weld_point.pcd", *weld_point);

	// ============ 第16步：将焊缝中心线点云再变换回机械臂基坐标系 ============
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_weld_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformPointCloud(weld_point, transformed_weld_point_cloud);
	transformed_weld_point_cloud->width = transformed_weld_point_cloud->size();
	transformed_weld_point_cloud->height = 1;
	pcl::io::savePCDFile("/tmp/transformed_weld_point_cloud.pcd", *transformed_weld_point_cloud);

	// ============ 第17步：生成焊枪位姿轨迹 ============
	std::vector<geometry_msgs::msg::Pose> Welding_Trajectory;
	// 不预设姿态，使用单位四元数（让执行端根据可达性自动选择）
	
	for (const auto &point : *transformed_weld_point_cloud)
	{
		geometry_msgs::msg::Pose pose;
		pose.position.x = point.x;
		pose.position.y = point.y;
		pose.position.z = point.z;
		// 单位四元数（无旋转），执行端会自动选择可达姿态
		pose.orientation.w = 1.0;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;

		Welding_Trajectory.push_back(pose);
	}

	cout << "焊缝轨迹提取完成，共有 " << Welding_Trajectory.size() << " 个位姿点" << endl;

	return Welding_Trajectory;
}

// ROS2节点类
class WeldButtSeamExtractingNode : public rclcpp::Node
{
public:
	WeldButtSeamExtractingNode() : Node("weld_butt_seam_extracting")
	{
		// 声明参数
		this->declare_parameter<std::string>("pcd_file_path", "");
		
		// 创建发布器 - 使用与订阅端匹配的QoS设置
		auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
		cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
			"/butt_cloud_point", 10);
		pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
			"/butt_torch_pose", qos);
		
		// 创建执行开始信号发布器
		start_execution_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
			"/start_welding_execution", 10);

		// 获取PCD文件路径
		std::string pcd_file_path;
		this->get_parameter("pcd_file_path", pcd_file_path);

		if (pcd_file_path.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "PCD文件路径未设置！请通过参数 'pcd_file_path' 指定");
			return;
		}

		// 处理焊缝提取
		RCLCPP_INFO(this->get_logger(), "开始提取焊缝轨迹...");
		trajectory_ = get_butt_weld_seam(pcd_file_path);

		if (trajectory_.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "焊缝轨迹提取失败！");
			return;
		}

		RCLCPP_INFO(this->get_logger(), "焊缝轨迹提取成功，共 %zu 个位姿点", trajectory_.size());

		// 创建定时器发布轨迹 - 降低到10ms (100Hz)，与订阅端匹配
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(10),
			std::bind(&WeldButtSeamExtractingNode::publishTrajectory, this));
	}

private:
	void publishTrajectory()
	{
		if (current_index_ < trajectory_.size())
		{
			pose_publisher_->publish(trajectory_[current_index_]);
			RCLCPP_INFO(this->get_logger(), "发布位姿 [%zu/%zu]", 
						current_index_ + 1, trajectory_.size());
			current_index_++;
		}
		else if (!publish_completed_)
		{
			RCLCPP_INFO(this->get_logger(), "所有焊缝位姿已发布完成！");
			timer_->cancel();
			publish_completed_ = true;
			
			// 等待100ms确保所有数据被订阅端接收
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			
			// 发送开始执行信号
			auto start_msg = std_msgs::msg::Bool();
			start_msg.data = true;
			start_execution_publisher_->publish(start_msg);
			RCLCPP_INFO(this->get_logger(), "已发送执行开始信号");
			
			// 再等待一段时间后关闭节点
			shutdown_timer_ = this->create_wall_timer(
				std::chrono::seconds(2),
				[this]() {
					RCLCPP_INFO(this->get_logger(), "发布节点即将关闭");
					rclcpp::shutdown();
				});
		}
	}

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_execution_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr shutdown_timer_;
	std::vector<geometry_msgs::msg::Pose> trajectory_;
	size_t current_index_ = 0;
	bool publish_completed_ = false;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WeldButtSeamExtractingNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
