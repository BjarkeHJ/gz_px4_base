/*

Utility class for handling frame convention transform between PX4 NED convention and ROS2 ENU convention

*/

#ifndef FRAME_TRANSFORM_
#define FRAME_TRANSFORM_

#include <Eigen/Core>
#include <Eigen/Geometry>

class FrameTF {
public:
    void vector_ENU_NED(Eigen::Vector3d& v); // Transform vector NED -> ENU // ENU -> NED (Symmetric)
    void vector_FLU_FRD(Eigen::Vector3d& v); // Transform vector FRD -> FLU // FLU -> FRD (Symmetric)
    void quat_ENU_NED(Eigen::Quaterniond& q); // Trasnform quaternion NED -> ENU // ENU -> NED (Symmetric)
    void quat_FLU_FRD(Eigen::Quaterniond& q); // Transform quaternion FRD -> FLU // FLU -> FRD (Symmetric)
    void quat_PX4_ROS2(Eigen::Quaterniond& q); // Transform quaternion PX4 -> ROS2 

    void vector_BODY_OPT(Eigen::Vector3d& v);

private:
    Eigen::Matrix3d R_FRD_FLU();    
    Eigen::Matrix3d R_NED_ENU();
    Eigen::Matrix3d R_BODY_OPT(); // for optical convention
};

// Transform Utils
void FrameTF::quat_PX4_ROS2(Eigen::Quaterniond& q) {
    Eigen::Matrix3d R_in = q.toRotationMatrix();
    Eigen::Matrix3d S_world = R_NED_ENU();
    Eigen::Matrix3d S_body = R_FRD_FLU();
    // Eigen::Matrix3d R_out = S_body * R_in * S_world.transpose();
    Eigen::Matrix3d R_out = S_world * R_in * S_body.transpose();
    q = Eigen::Quaterniond(R_out);
    q.normalize();
}

void FrameTF::vector_ENU_NED(Eigen::Vector3d& v) {
    v = FrameTF::R_NED_ENU() * v;
}

void FrameTF::vector_FLU_FRD(Eigen::Vector3d& v) {
    v = FrameTF::R_FRD_FLU() * v;
}

void FrameTF::vector_BODY_OPT(Eigen::Vector3d& v) {
    v = FrameTF::R_BODY_OPT() * v;
}

void FrameTF::quat_ENU_NED(Eigen::Quaterniond& q) {
    Eigen::Matrix3d R_in = q.toRotationMatrix();
    Eigen::Matrix3d S = R_NED_ENU();
    Eigen::Matrix3d R_out = S * R_in * S.transpose();
    q = Eigen::Quaterniond(R_out);
    q.normalize();
}

void FrameTF::quat_FLU_FRD(Eigen::Quaterniond& q) {
    Eigen::Matrix3d R_in = q.toRotationMatrix();
    Eigen::Matrix3d S = R_FRD_FLU();
    Eigen::Matrix3d R_out = S * R_in * S.transpose();
    q = Eigen::Quaterniond(R_out);
    q.normalize();
}

// Static transform matrices // 
Eigen::Matrix3d FrameTF::R_FRD_FLU() {
    Eigen::Matrix3d m;
    m << 1.0, 0.0, 0.0,
         0.0, -1.0, 0.0,
         0.0, 0.0, -1.0;
    return m;
}

Eigen::Matrix3d FrameTF::R_NED_ENU() {
    // Corresponds to a pi/2 rotation about z followed by pi rotation about (new) x.
    // Full transform of vector would then be: Mx*Mz*vec
    Eigen::Matrix3d m;
    m << 0.0, 1.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 0.0, -1.0;
    return m;
}

Eigen::Matrix3d FrameTF::R_BODY_OPT() {
    Eigen::Matrix3d m;
    m <<  0.0,  0.0,  1.0,
          0.0,  1.0,  0.0,
         -1.0,  0.0,  0.0; 
    return m;
}

#endif