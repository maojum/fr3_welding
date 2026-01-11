#include "weld_butt_seam_extracting/weld_butt_seam_extracting_headfile.h"

/**
 * @brief 空间向量结构（用于存储交线的方向向量）
 */
struct Space_vector
{
	double A;  // 向量的x分量
	double B;  // 向量的y分量
	double C;  // 向量的z分量
};

/**
 * @brief 空间点结构（用于存储交线上的任意一点）
 */
struct Any_Point
{
	double x;  // 点的x坐标
	double y;  // 点的y坐标
	double z;  // 点的z坐标
};

/**
 * @brief 计算两个平面的交线
 * @details 目的：根据两个平面的方程系数计算它们的交线参数（点+方向向量）。
 *          交线方向向量通过两个平面法向量的叉乘获得，交线上的一点通过令z=0求解x、y获得。
 * @param Actor_First 输入：第一个平面的方程系数 (A1, B1, C1, D1)
 * @param Actor_Second 输入：第二个平面的方程系数 (A2, B2, C2, D2)
 * @param coefficients_line 输出：交线参数，包含直线上一点(x0,y0,z0)和方向向量(a,b,c)共6个参数
 */
void SeamIntersectLine(Leading_Factor Actor_First, Leading_Factor Actor_Second, pcl::ModelCoefficients::Ptr& coefficients_line)
{
	// ============ 第一步：计算交线的方向向量 ============
	// 方法：通过两个平面法向量的叉乘得到交线方向向量
	// 平面1法向量: N1 = (A1, B1, C1)
	// 平面2法向量: N2 = (A2, B2, C2)
	// 交线方向向量: V = N1 × N2
	
	Space_vector Space_Point;      // 原始方向向量
	Space_vector Space_Point_Dan;  // 归一化方向向量（未使用）

	cout << "计算交线的方向向量" << endl;

	// 计算叉乘（三阶行列式）
	// V.x = N1.y * N2.z - N1.z * N2.y
	// V.y = N1.z * N2.x - N1.x * N2.z
	// V.z = N1.x * N2.y - N1.y * N2.x
	Space_Point.A = Actor_First.B * Actor_Second.C - Actor_First.C * Actor_Second.B;
	Space_Point.B = Actor_First.C * Actor_Second.A - Actor_First.A * Actor_Second.C;
	Space_Point.C = Actor_First.A * Actor_Second.B - Actor_First.B * Actor_Second.A;

	cout << "交线的方向向量：" << "( " << Space_Point.A << "," << Space_Point.B << ", " << Space_Point.C << " )" << endl;

	// ============ 第二步：求交线上的任意一点 ============
	// 方法：令 z = 0，将两个平面方程联立求解 x 和 y
	// 平面1: A1*x + B1*y + C1*z + D1 = 0  →  A1*x + B1*y + D1 = 0 (z=0)
	// 平面2: A2*x + B2*y + C2*z + D2 = 0  →  A2*x + B2*y + D2 = 0 (z=0)
	
	Any_Point Point_any;

	cout << "开始任取一点" << endl; // 设z值为0

	// 使用克拉默法则求解二元一次方程组
	// x = (B1*D2 - B2*D1) / (A1*B2 - A2*B1)
	// y = (A1*D2 - A2*D1) / (A2*B1 - A1*B2)
	Point_any.x = (Actor_First.B * Actor_Second.D - Actor_Second.B * Actor_First.D) / 
	              (Actor_First.A * Actor_Second.B - Actor_Second.A * Actor_First.B);
	Point_any.y = (Actor_First.A * Actor_Second.D - Actor_Second.A * Actor_First.D) / 
	              (Actor_Second.A * Actor_First.B - Actor_First.A * Actor_Second.B);
	Point_any.z = 0;

	cout << "任取一点完成" << endl;
	cout << "坐标为：" << "(" << Point_any.x << "," << Point_any.y << "," << Point_any.z << ")" << endl;

	// ============ 第三步：保存交线参数 ============
	// 直线的参数表示：P = P0 + t*V
	// 其中 P0 = (x0, y0, z0) 是直线上的一点
	//      V = (a, b, c) 是直线的方向向量
	// coefficients_line 存储格式: [x0, y0, z0, a, b, c]
	
	coefficients_line->values.push_back(Point_any.x);    // 直线上的点 x0
	coefficients_line->values.push_back(Point_any.y);    // 直线上的点 y0
	coefficients_line->values.push_back(Point_any.z);    // 直线上的点 z0
	coefficients_line->values.push_back(Space_Point.A);  // 方向向量 a
	coefficients_line->values.push_back(Space_Point.B);  // 方向向量 b
	coefficients_line->values.push_back(Space_Point.C);  // 方向向量 c

	cout << "得到直线的参数" << endl;
}