/**
 * @file transform.h
 * @author yang (you@domain.com)
 * @brief 基于Eigen库的旋转姿态变化等相关功能函数
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef TRANSFORM_H
#define TRANSFORM_H

using vecd = std::vector<double>;
using Mat3d = Eigen::Matrix3d;
using Vec3d = Eigen::Vector3d;
using VecXd = Eigen::VectorXd;
using Quatd = Eigen::Quaterniond;


/**
 * @brief 欧拉角有许多中表示方式，内旋、外旋、旋转轴顺序等。
 * 本系统中以固定轴 X(Roll)-Y(Pitch)-Z(Yaw) 轴旋转方式定义使用。 
 * 固定轴旋转 X(roll)-Y(pitch)-Z(yaw) 和 绕自身轴 先 Z（Yaw） Y（Pitch）X（Roll）等价保持一致。
 * 即 绕固定轴旋转 Rxyz=Rz(yaw)*Ry(pitch)*Rx(roll)
 *    绕自身轴旋转 R = Rx(roll)*Ry(pitch)*Rz(yaw)    等价
 * 
 * 欧拉角的缺点：万向节死锁：Pitch=-90，90时，第一轴和第三轴旋转重合，一般限制Pitch （-90，90）
 * 插值误差大，  相比较，  四元数  非常适合插值方法的使用。 而且四元数表述唯一。
 * 
 * Eigen旋转变换实现的参考：https://blog.csdn.net/lemonxiaoxiao/article/details/123596114
 * 旋转向量（轴角），旋转矩阵、欧拉角、四元数 互相转换都很简便的实现/
 * 
 * zy:个人对eigen库的旋转矩阵记录： 从坐标系A 经过 rpy 到 坐标系B， 旋转矩阵R 表示 从 B 到 A 的旋转矩阵 
 *                                              eg:     P_a = R * P_b 
 */
///  旋转向量到 四元数 是 绕固定轴 旋转的，  



namespace eigenLib{


        /// 注意 eigen中 四元数 内部顺序为 xyzw，初始化为 wxyz；
        /// 参数 rpy 为 绕固定轴x-y-z 旋转的 （roll-pitch-yaw）
        inline void rpy2Quat(Eigen::Quaterniond &quat, Eigen::Vector3d &rpy){
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())); 
            quat=yawAngle*pitchAngle*rollAngle;    /// 确认过了  是按照固定轴旋转的,
                    ////  quat.matrix()  是  从坐标系A 经过 rpy 到 坐标系B， 旋转矩阵R 表示 从 B 到 A 的旋转矩阵 
                                          //     eg:     P_a = R * P_b 
        }
        inline void rpy2ZYXEuler(Eigen::Vector3d &ZYXEuler, Eigen::Vector3d &rpy){
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())); 
            Eigen::Quaterniond quat=yawAngle*pitchAngle*rollAngle;  
            ZYXEuler = quat.matrix().eulerAngles(0,1,2);/// Z-Y-X 欧拉角，先绕Z轴旋转rpy[0],,,
        }
        inline void rpy2ZYXEuler2Rpy(Eigen::Vector3d &rpy){
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())); 
            Eigen::Quaterniond quat=yawAngle*pitchAngle*rollAngle;  
            Eigen::Vector3d ZYXEuler = quat.matrix().eulerAngles(0,1,2);/// Z-Y-X 欧拉角，先绕Z轴旋转rpy[0],,,
            rpy[0] = ZYXEuler[2];
            rpy[1] = ZYXEuler[1];
            rpy[2] = ZYXEuler[0];  
        }
        /// ZYXEuler：  ZYX 欧拉角     ZYXEuler[0] 为绕Z轴旋转角
        inline void quat2ZYXEuler(Eigen::Vector3d &ZYXEuler, Eigen::Quaterniond &quat){
            ZYXEuler = quat.matrix().eulerAngles(0,1,2);/// Z-Y-X 欧拉角，先绕Z轴旋转rpy[0],,,
        }
        /// 参数 rpy 为 绕固定轴x-y-z 旋转的 （roll-pitch-yaw）   这个存在数值不稳定
        // inline void quat2Rpy(Eigen::Vector3d &rpy, Eigen::Quaterniond &quat){
        //     Eigen::Vector3d ZYXEuler = quat.matrix().eulerAngles(0,1,2);/// Z-Y-X 欧拉角，先绕Z轴旋转rpy[0],,,
        //     rpy[0] = ZYXEuler[2];
        //     rpy[1] = ZYXEuler[1];
        //     rpy[2] = ZYXEuler[0];           
        // }
        inline void quat2Rpy(Eigen::Vector3d &rpy, Eigen::Quaterniond &quat){
            Mat3d rot = quat.toRotationMatrix();
            rpy(0) = atan2(rot(2,1), rot(2,2));
            double s = sqrt(rot(2,1)*rot(2,1)+ rot(2,2)*rot(2,2));
            rpy(1) = atan2(-rot(2,0), s);
            rpy(2) = atan2(rot(1,0), rot(0,0));
        }


} // namespace eigenLib



#endif //#define TRANSFORM_H