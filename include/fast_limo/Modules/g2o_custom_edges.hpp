#ifndef G2O_CUSTOM_EDGES_HPP
#define G2O_CUSTOM_EDGES_HPP

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace g2o {

/**
 * Edge that constrains a pose vertex to align with a floor plane
 * The measurement is the plane parameters (normal, distance) in world frame
 */
class EdgeSE3Plane : public BaseUnaryEdge<3, Eigen::Vector4d, VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeSE3Plane() : BaseUnaryEdge<3, Eigen::Vector4d, VertexSE3>() {}
    
    void computeError() override {
        const VertexSE3* pose = static_cast<const VertexSE3*>(_vertices[0]);
        
        // Plane parameters in world frame
        Eigen::Vector4d plane = _measurement;
        
        // Rotation from world to pose
        Eigen::Matrix3d R = pose->estimate().rotation();
        
        // Transform plane to local frame
        Eigen::Vector3d normal_world(plane[0], plane[1], plane[2]);
        Eigen::Vector3d normal_local = R.transpose() * normal_world;
        
        // Error: normal should be (0,0,1) in local frame
        _error[0] = normal_local.x();
        _error[1] = normal_local.y();
        _error[2] = normal_local.z() - 1.0; // z should be 1
    }
    
    void linearizeOplus() override {
        const VertexSE3* pose = static_cast<const VertexSE3*>(_vertices[0]);
        
        // Plane parameters
        Eigen::Vector3d normal_world(_measurement[0], _measurement[1], _measurement[2]);
        
        // Rotation from world to pose
        Eigen::Matrix3d R = pose->estimate().rotation();
        
        // Derivative of the error w.r.t. the pose
        _jacobianOplusXi.setZero();
        
        // Only the rotation part affects the normal direction
        _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3,3>(0,3) = -R.transpose() * Eigen::Matrix3d::Identity() * normal_world.asDiagonal();
    }
    
    bool read(std::istream& is) override {
        Eigen::Vector4d v;
        is >> v[0] >> v[1] >> v[2] >> v[3];
        setMeasurement(v);
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i,j);
                if (i != j)
                    information()(j,i) = information()(i,j);
            }
        return true;
    }
    
    bool write(std::ostream& os) const override {
        Eigen::Vector4d v = _measurement;
        os << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " ";
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i,j);
        return os.good();
    }
};

/**
 * Edge that constrains a pose vertex to align with gravity direction
 * The measurement is the gravity vector in world frame (usually 0,0,1)
 */
class EdgeSE3GravityDirection : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeSE3GravityDirection() : BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3>() {}
    
    void computeError() override {
        const VertexSE3* pose = static_cast<const VertexSE3*>(_vertices[0]);
        
        // Gravity in world frame (usually 0,0,1)
        Eigen::Vector3d gravity_world = _measurement;
        
        // Rotation from world to pose
        Eigen::Matrix3d R = pose->estimate().rotation();
        
        // In the local frame, gravity should be along the z-axis (0,0,1)
        Eigen::Vector3d local_gravity = R.transpose() * gravity_world;
        Eigen::Vector3d expected_gravity(0, 0, 1);
        
        // Error is the difference between expected and actual gravity
        _error = local_gravity - expected_gravity;
    }
    
    void linearizeOplus() override {
        const VertexSE3* pose = static_cast<const VertexSE3*>(_vertices[0]);
        
        // Gravity in world frame
        Eigen::Vector3d gravity_world = _measurement;
        
        // Rotation from world to pose
        Eigen::Matrix3d R = pose->estimate().rotation();
        
        // Jacobian is 3x6 (3 for error, 6 for SE3 parameters)
        _jacobianOplusXi.setZero();
        
        // Only the rotation part affects gravity direction
        Eigen::Matrix3d skew = skewSymmetric(R.transpose() * gravity_world);
        _jacobianOplusXi.block<3,3>(0,0) = -skew;
        
        // Translation has no effect
        _jacobianOplusXi.block<3,3>(0,3).setZero();
    }
    
    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return skew;
    }
    
    bool read(std::istream& is) override {
        Eigen::Vector3d v;
        is >> v[0] >> v[1] >> v[2];
        setMeasurement(v);
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i,j);
                if (i != j)
                    information()(j,i) = information()(i,j);
            }
        return true;
    }
    
    bool write(std::ostream& os) const override {
        Eigen::Vector3d v = _measurement;
        os << v[0] << " " << v[1] << " " << v[2] << " ";
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i,j);
        return os.good();
    }
};

} // namespace g2o

#endif // G2O_CUSTOM_EDGES_HPP 