#pragma once

#ifndef Q_MOC_RUN
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>
#include <QScrollArea>
#include <QListWidget>
#include <QStyledItemDelegate>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <rviz/panel.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include <common_utilities/rviz_visualization.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <roboy_simulation_msgs/Tendon.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <roboy_simulation_msgs/JointState.h>

#include <map>
#include <thread>

#endif

using namespace std;
using namespace Eigen;


class CardsflowRviz : public rviz::Panel, public rviz_visualization {
    Q_OBJECT

public:
    CardsflowRviz(QWidget *parent = 0);

    ~CardsflowRviz();

    /**
     * Load all configuration data for this panel from the given Config object.
     * @param config rviz config file
     */
    virtual void load(const rviz::Config &config);

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    /**
     * Toggles mesh visualization
     */
    void show_mesh();

    /**
     * Toggles collision visualization
     */
    void show_collision();

    /**
     * Toggles tendon visualization
     */
    void show_tendon();
    /**
     * Toggles tendon length visualization
     */
    void show_tendon_length();

    /**
     * Toggles force visualization
     */
    void show_force();

    /**
     * Toggles torque visualization
     */
    void show_torque();

    /**
     * Visualization of Pose
     */
    void visualizePose();

    /**
     * Visualization of Collision Shapes
     */
    void visualizeCollision();

    /**
     * Visualization of Pose Target
     */
    void visualizePoseTarget();

    /**
     * Visualization of Tendon
     */
    void visualizeTendon();

    /**
     * Visualization of Tendon Target
     */
    void visualizeTendonTarget();

    /**
     * Visualization of Torque
     */
    void visualizeTorque();

    /**
     * Visualization of Torque Target
     */
    void visualizeTorqueTarget();

private:
    /**
     * Callback to robot state messages
     * @param msg
     */
    void RobotState(const geometry_msgs::PoseStampedConstPtr &msg);
    /**
     * Callback to robot state target messages
     * @param msg
     */
    void RobotStateTarget(const geometry_msgs::PoseStampedConstPtr &msg);
    /**
     * Callback for Tendon state messages
     * @param msg
     */
    void TendonState(const roboy_simulation_msgs::TendonConstPtr &msg);
    /**
     * Callback for Tendon state target messages
     * @param msg
     */
    void TendonStateTarget(const roboy_simulation_msgs::TendonConstPtr &msg);
    /**
     * Callback for Joint state messages
     * @param msg
     */
    void JointState(const roboy_simulation_msgs::JointStateConstPtr &msg);
    /**
     * Callback for Joint state target messages
     * @param msg
     */
    void JointStateTarget(const roboy_simulation_msgs::JointStateConstPtr &msg);
Q_SIGNALS:
    void visualizePoseSignal();
    void visualizeCollisionSignal();
    void visualizePoseTargetSignal();
    void visualizeTendonSignal();
    void visualizeTendonTargetSignal();
    void visualizeTorqueSignal();
    void visualizeTorqueTargetSignal();
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber robot_state, tendon_state, joint_state, robot_state_target, tendon_state_target, joint_state_target;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    map<string, geometry_msgs::Pose> pose, pose_target;
    struct Tendon{
        float force;
        float l;
        float ld;
        vector<geometry_msgs::Vector3> viaPoints;
    };
    map<string, Tendon> tendon, tendon_target;
    map<string, geometry_msgs::Vector3> joint_origin, joint_origin_target;
    map<string, geometry_msgs::Vector3> joint_axis, joint_axis_target;
    map<string, double> torque, torque_target;
    bool visualize_pose = true, visualize_collisions = true, visualize_tendon = true, visualize_tendon_length = true, visualize_force = false, visualize_torque = false;
    bool visualize_pose_target = true, visualize_tendon_target = true, visualize_tendon_length_target = true, visualize_force_target = false, visualize_torque_target = false;
    QPushButton *show_mesh_button, *show_collision_button, *show_tendon_button, *show_force_button, *show_torque_button, *show_tendon_length_button;
    QSlider *mesh_transparency, *cable_thickness, *tendon_length_text_size;
    string model_name;
};
