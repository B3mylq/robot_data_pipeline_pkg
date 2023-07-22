/**
 * @file eigen_lib.h
 * @author yang (you@domain.com)
 * @brief 将C++的Eigen库简化，以及使用Eigen库实现的一些功能函数,但没有明确的分类，因此放在这里。
 * @version 0.1
 * @date 2023-03-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef EIGEN_LIB_H
#define EIGEN_LIB_H
#include<Eigen/Dense>
#include <cstring>
#include <vector>

#include "transform.h"

///  array  相比较于 vec、mat 能够实现 数组元素之间加减法等快速运算

// #define vec3d Eigen::Vector3d   
// #define arr3d Eigen::Array3d  //  array  相比较于 vec、mat 能够实现 数组元素之间加减法等快速运算

// #define mat3d Eigen::Matrix3d
// #define matXd(r,c) Eigen::Matrix<double,r,c>
// #define matX Eigen::Matrix

// #define quat4d Eigen::Quaterniond // 四元数
// #define aAxisd Eigen::AngleAxisd  // 旋转矩阵



using vecd = std::vector<double>;
using Mat3d = Eigen::Matrix3d;
using MatXd = Eigen::MatrixXd;
using Vec3d = Eigen::Vector3d;
using VecXd = Eigen::VectorXd;
using Quatd = Eigen::Quaterniond;


namespace eigenLib{


	// 斜对称矩阵  向量叉乘用 
	inline Eigen::Matrix3d skew(const Eigen::Vector3d &v){
		Eigen::Matrix3d M;
		M<<   0.0, -v[2],  v[1], 
				v[2],   0.0, -v[0], 
			-v[1],	v[0],	0.0;
		return M;
	};
	inline void skew(Eigen::Matrix3d &M,const Eigen::Vector3d &v){
		M<<0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
	};

	inline Eigen::Matrix3d pInverse(Eigen::Matrix3d &m){
		return m.completeOrthogonalDecomposition().pseudoInverse();
	}
	
	/// 矩阵x向量， 3x3*3x1  传入的是行优先的数组
	inline void matrixProductVector(double *vector_out, double *matrix, double *vector_in){
		Eigen::Matrix3d m = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(matrix);
		Eigen::Vector3d v_in(vector_in[0],vector_in[1],vector_in[2]);
		Eigen::Vector3d v_out = m * v_in;		
		memcpy(vector_out,v_out.data(),24);
	}


}  // namespace eigenLib
#endif  // EIGEN_LIB_H












	// //R to axis*angle   旋转矩阵 转  绕轴旋转向量 
	// inline Eigen::Vector3d R2na(const mat3d & R){
	//     vec3d na;
	//     double angle=acos(R.trace()/2.0-1/2.0);
	//     if(std::abs(angle)<1e-6){
	//         na=vec3d::Zero();
	//     }else{
	// 		na=angle/(2.0*sin(angle)) *vec3d(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
	// 	}
	// 	return na;
	// }

	//==one order filter============
	// class FilterOne{
	// public:
	// 	FilterOne(double dt, double cut_freq){
	// 		this->dt = dt;
	// 		k=cut_freq*dt;
	// 		ks=k/50;
	// 		y.setZero();
	// 		ySum.setZero();
	// 	};

	// 	void init(double dt,double cutF,vec3d y0){
	// 		this->dt=dt;
	// 		k=cutF*dt;
	// 		ks=k/50;
	// 		y=y0;
	// 	};
	// 	void setCutF(double cutF){
	// 		k=cutF*dt;
	// 		ks=k/50;
	// 	};
	// 	const vec3d & filt(const vec3d & x){
	// 		y+=k*(x-y)-ks*ySum;
	// 		ySum+=y-x;
	// 		return y;
	// 	};
	// private:
	// 	Eigen::Vector3d  y,ySum;
	// 	double dt,k,ks;
	// };
