#pragma once

#ifndef Q_MOC_RUN
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
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
#endif

using namespace std;
using namespace Eigen;


class CardsflowRviz : public rviz::Panel {
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
     * Connects to sensor_port and IP given via GUI
     */
    void show_mesh();

private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher visualization_pub;
    ros::Subscriber pose_correction_sub;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
};
