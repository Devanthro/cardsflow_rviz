#include "include/cardsflow_rviz/cardsflow_rviz.hpp"

CardsflowRviz::CardsflowRviz(QWidget *parent)
        : rviz::Panel(parent) {

    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QTabWidget *tabPage = new QTabWidget;

    // ################################################### connect TAB
    QWidget *connectWidget = new QWidget;
    connectWidget->setLayout(new QVBoxLayout);
    tabPage->addTab(connectWidget, "connect");

    show_mesh_button = new QPushButton(tr("show_mesh"));
    show_mesh_button->setObjectName("show_mesh");
    show_mesh_button->setCheckable(true);
    show_mesh_button->setChecked(true);
    connect(show_mesh_button, SIGNAL(clicked()), this, SLOT(show_mesh()));
    connectWidget->layout()->addWidget(show_mesh_button);

    mesh_transparency = new QSlider(Qt::Orientation::Horizontal);
    mesh_transparency->setObjectName("mesh_transparency");
    mesh_transparency->resize(100,30);
    mesh_transparency->setTickPosition(QSlider::TicksBelow);
    mesh_transparency->setTickInterval(1);
    mesh_transparency->setSingleStep(1);
    mesh_transparency->setRange(0,100);
    mesh_transparency->setValue(100);
    connectWidget->layout()->addWidget(mesh_transparency);

    show_collision_button = new QPushButton(tr("show_collision"));
    show_collision_button->setObjectName("show_collision");
    show_collision_button->setCheckable(true);
    show_collision_button->setChecked(true);
    connect(show_collision_button, SIGNAL(clicked()), this, SLOT(show_collision()));
    connectWidget->layout()->addWidget(show_collision_button);

    show_target_button = new QPushButton(tr("show_target"));
    show_target_button->setObjectName("show_target");
    show_target_button->setCheckable(true);
    show_target_button->setChecked(true);
    connect(show_target_button, SIGNAL(clicked()), this, SLOT(show_target()));
    connectWidget->layout()->addWidget(show_target_button);

    show_tendon_button = new QPushButton(tr("show_tendon"));
    show_tendon_button->setObjectName("show_tendon");
    show_tendon_button->setCheckable(true);
    show_tendon_button->setChecked(true);
    connect(show_tendon_button, SIGNAL(clicked()), this, SLOT(show_tendon()));
    connectWidget->layout()->addWidget(show_tendon_button);

    cable_thickness = new QSlider(Qt::Orientation::Horizontal);
    cable_thickness->setObjectName("cable_thickness");
    cable_thickness->resize(100,30);
    cable_thickness->setTickPosition(QSlider::TicksBelow);
    cable_thickness->setTickInterval(1);
    cable_thickness->setSingleStep(1);
    cable_thickness->setRange(0,100);
    cable_thickness->setValue(6);
    connectWidget->layout()->addWidget(cable_thickness);

    show_tendon_length_button = new QPushButton(tr("show_tendon_length"));
    show_tendon_length_button->setObjectName("show_tendon_length");
    show_tendon_length_button->setCheckable(true);
    show_tendon_length_button->setChecked(true);
    connect(show_tendon_length_button, SIGNAL(clicked()), this, SLOT(show_tendon_length()));
    connectWidget->layout()->addWidget(show_tendon_length_button);

    tendon_length_text_size = new QSlider(Qt::Orientation::Horizontal);
    tendon_length_text_size->setObjectName("tendon_length_text_size");
    tendon_length_text_size->resize(100,30);
    tendon_length_text_size->setTickPosition(QSlider::TicksBelow);
    tendon_length_text_size->setTickInterval(1);
    tendon_length_text_size->setSingleStep(1);
    tendon_length_text_size->setRange(0,100);
    tendon_length_text_size->setValue(6);
    connectWidget->layout()->addWidget(tendon_length_text_size);

    show_force_button = new QPushButton(tr("show_force"));
    show_force_button->setObjectName("show_force");
    show_force_button->setCheckable(true);
    show_force_button->setChecked(false);
    connect(show_force_button, SIGNAL(clicked()), this, SLOT(show_force()));
    connectWidget->layout()->addWidget(show_force_button);

    show_torque_button = new QPushButton(tr("show_torque"));
    show_torque_button->setObjectName("show_torque");
    show_torque_button->setCheckable(true);
    show_torque_button->setChecked(false);
    connect(show_torque_button, SIGNAL(clicked()), this, SLOT(show_torque()));
    connectWidget->layout()->addWidget(show_torque_button);

    frameLayout->addWidget(tabPage);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "CardsflowRvizPlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));

    robot_state = nh->subscribe("/robot_state", 100, &CardsflowRviz::RobotState, this);
    tendon_state = nh->subscribe("/tendon_state", 100, &CardsflowRviz::TendonState, this);
    joint_state = nh->subscribe("/rviz_joint_states", 100, &CardsflowRviz::JointState, this);

    robot_state_target = nh->subscribe("/robot_state_target", 100, &CardsflowRviz::RobotStateTarget, this);
    tendon_state_target = nh->subscribe("/tendon_state_target", 100, &CardsflowRviz::TendonStateTarget, this);
    joint_state_target = nh->subscribe("/joint_state_target", 100, &CardsflowRviz::JointStateTarget, this);

    if (!nh->hasParam("model_name"))
        ROS_FATAL("model_name could not be found on parameter server!!! ");
    else
        nh->getParam("model_name", model_name);

    QObject::connect(this, SIGNAL(visualizePoseSignal()), this, SLOT(visualizePose()));
//    QObject::connect(this, SIGNAL(visualizeTargetSignal()), this, SLOT(visualizeTarget()));
    QObject::connect(this, SIGNAL(visualizeCollisionSignal()), this, SLOT(visualizeCollision()));
    QObject::connect(this, SIGNAL(visualizePoseTargetSignal()), this, SLOT(visualizePoseTarget()));
    QObject::connect(this, SIGNAL(visualizeTendonSignal()), this, SLOT(visualizeTendon()));
    QObject::connect(this, SIGNAL(visualizeTendonTargetSignal()), this, SLOT(visualizeTendonTarget()));
    QObject::connect(this, SIGNAL(visualizeTorqueSignal()), this, SLOT(visualizeTorque()));
    QObject::connect(this, SIGNAL(visualizeTorqueTargetSignal()), this, SLOT(visualizeTorqueTarget()));

    publish_as_marker_array = true;

    number_of_markers_to_publish_at_once = 1000;
}

CardsflowRviz::~CardsflowRviz() {
}

void CardsflowRviz::load(const rviz::Config &config) {
    rviz::Panel::load(config);
    QVariant value;
    config.mapGetValue(show_mesh_button->objectName(), &value);
    show_mesh_button->setChecked(value.toBool());
    config.mapGetValue(show_collision_button->objectName(), &value);
    show_collision_button->setChecked(value.toBool());
    config.mapGetValue(show_target_button->objectName(), &value);
    show_target_button->setChecked(value.toBool());
    config.mapGetValue(show_tendon_button->objectName(), &value);
    show_tendon_button->setChecked(value.toBool());
    config.mapGetValue(show_tendon_length_button->objectName(), &value);
    show_tendon_length_button->setChecked(value.toBool());
    config.mapGetValue(show_force_button->objectName(), &value);
    show_force_button->setChecked(value.toBool());
    config.mapGetValue(show_torque_button->objectName(), &value);
    show_torque_button->setChecked(value.toBool());
    config.mapGetValue(mesh_transparency->objectName(), &value);
    mesh_transparency->setValue(value.toInt());
    config.mapGetValue(cable_thickness->objectName(), &value);
    cable_thickness->setValue(value.toInt());
    config.mapGetValue(tendon_length_text_size->objectName(), &value);
    tendon_length_text_size->setValue(value.toInt());
    show_mesh();
    show_collision();
    show_target();
    show_tendon();
    show_tendon_length();
    show_force();
    show_torque();
}

void CardsflowRviz::save(rviz::Config config) const {
    config.mapSetValue(show_mesh_button->objectName(), show_mesh_button->isChecked());
    config.mapSetValue(show_collision_button->objectName(), show_collision_button->isChecked());
    config.mapSetValue(show_target_button->objectName(), show_target_button->isChecked());
    config.mapSetValue(show_tendon_button->objectName(), show_tendon_button->isChecked());
    config.mapSetValue(show_tendon_length_button->objectName(), show_tendon_length_button->isChecked());
    config.mapSetValue(show_force_button->objectName(), show_force_button->isChecked());
    config.mapSetValue(show_torque_button->objectName(), show_torque_button->isChecked());
    config.mapSetValue(mesh_transparency->objectName(), mesh_transparency->value());
    config.mapSetValue(cable_thickness->objectName(), cable_thickness->value());
    config.mapSetValue(tendon_length_text_size->objectName(), tendon_length_text_size->value());
    rviz::Panel::save(config);
}

void CardsflowRviz::show_mesh() {
    visualize_pose = show_mesh_button->isChecked();
}

void CardsflowRviz::show_collision() {
    visualize_collisions = show_collision_button->isChecked();
}

void CardsflowRviz::show_target() {
    visualize_targets = show_target_button->isChecked();
}

void CardsflowRviz::show_tendon() {
    visualize_tendon = show_tendon_button->isChecked();
}

void CardsflowRviz::show_tendon_length() {
    visualize_tendon_length = show_tendon_length_button->isChecked();
}

void CardsflowRviz::show_force() {
    visualize_force = show_force_button->isChecked();
}

void CardsflowRviz::show_torque() {
    visualize_torque = show_torque_button->isChecked();
}

void CardsflowRviz::RobotState(const geometry_msgs::PoseStampedConstPtr &msg) {
    pose[msg->header.frame_id] = msg->pose;
    if (visualize_pose)
            emit visualizePoseSignal();
    if (visualize_collisions)
            emit visualizeCollisionSignal();
}

void CardsflowRviz::RobotStateTarget(const geometry_msgs::PoseStampedConstPtr &msg) {
    pose_target[msg->header.frame_id] = msg->pose;
    if (visualize_targets)
            emit visualizePoseTargetSignal();
}

void CardsflowRviz::TendonState(const roboy_simulation_msgs::TendonConstPtr &msg) {
    int offset = 0;
    for (int i = 0; i < msg->name.size(); i++) {
        Tendon t;
        t.force = msg->force[i];
        t.l = msg->l[i];
        t.ld = msg->ld[i];
        for (int v = 0; v < msg->number_of_viapoints[i]; v++) {
            t.viaPoints.push_back(msg->viapoints[v + offset]);
        }
        tendon[msg->name[i]] = t;
        offset += msg->number_of_viapoints[i];
    }
    if (visualize_tendon||visualize_tendon_length||visualize_force)
            emit visualizeTendonSignal();
}

void CardsflowRviz::TendonStateTarget(const roboy_simulation_msgs::TendonConstPtr &msg) {
    int offset = 0;
    for (int i = 0; i < msg->name.size(); i++) {
        Tendon t;
        t.force = msg->force[i];
        t.l = msg->l[i];
        t.ld = msg->ld[i];
        for (int v = 0; v < msg->number_of_viapoints[i]; v++) {
            t.viaPoints.push_back(msg->viapoints[v + offset]);
        }
        tendon_target[msg->name[i]] = t;
        offset += msg->number_of_viapoints[i];
    }
    if (visualize_tendon_target||visualize_tendon_length_target||visualize_force_target)
        emit visualizeTendonTargetSignal();
}

void CardsflowRviz::JointState(const roboy_simulation_msgs::JointStateConstPtr &msg) {
    int j = 0;
    for (auto name:msg->names) {
        joint_origin[name] = msg->origin[j];
        joint_axis[name] = msg->axis[j];
        torque[name] = msg->torque[j];
        j++;
    }
    if (visualize_torque)
        emit visualizeTorqueSignal();
}

void CardsflowRviz::JointStateTarget(const roboy_simulation_msgs::JointStateConstPtr &msg) {
    int j = 0;
    for (auto name:msg->names) {
        joint_origin[name] = msg->origin[j];
        joint_axis[name] = msg->axis[j];
        torque[name] = msg->torque[j];
        j++;
    }
    if (visualize_torque_target)
        emit visualizeTorqueTargetSignal();
}

void CardsflowRviz::visualizePose() {
    // Allow change of model_name during visualization
    if (nh->hasParam("model_name")) {
        string prev_robot_name;
        prev_robot_name.assign(model_name);
        nh->getParam("model_name", model_name);
        // Init pose and tendon if changed robot
        if (model_name.compare(prev_robot_name)) {
            ROS_INFO_STREAM("Changing robot from " << prev_robot_name << " to " << model_name);
            pose.clear();
            tendon.clear();
        }
    }
    int message_id = 0;
    for (auto p:pose) {
        publishMesh("robots", (model_name + "/meshes/visuals").c_str(), (p.first + ".stl").c_str(), p.second, 0.001,
                    "world", "pose", message_id++, 1, COLOR(1,1,1,(mesh_transparency->value()/100.0)));
        tf::Transform bt;
        PoseMsgToTF(p.second, bt);
        tf_broadcaster.sendTransform(tf::StampedTransform(bt, ros::Time::now(), "world", p.first));
    }
}

void CardsflowRviz::visualizeCollision() {
    // Allow change of model_name during visualization
    if (nh->hasParam("model_name")) {
        string prev_robot_name;
        prev_robot_name.assign(model_name);
        nh->getParam("model_name", model_name);
        // Init pose and tendon if changed robot
        if (model_name.compare(prev_robot_name)) {
            ROS_INFO_STREAM("Changing robot from " << prev_robot_name << " to " << model_name);
            pose.clear();
            tendon.clear();
        }
    }
    int message_id = 500;
    for (auto p:pose) {
        publishMesh("robots", (model_name + "/meshes/collisions").c_str(), (p.first + ".stl").c_str(), p.second, 0.001,
                    "world", "pose", message_id++, 1, COLOR(0.5,0.5,0,0.2));
        tf::Transform bt;
        PoseMsgToTF(p.second, bt);
        tf_broadcaster.sendTransform(tf::StampedTransform(bt, ros::Time::now(), "world", p.first));
    }
}

void CardsflowRviz::visualizePoseTarget() {
    // Allow change of model_name during visualization
    if (nh->hasParam("model_name")) {
        string prev_robot_name;
        prev_robot_name.assign(model_name);
        nh->getParam("model_name", model_name);
        // Init pose and tendon if changed robot
        if (model_name.compare(prev_robot_name)) {
            ROS_INFO_STREAM("Changing robot from " << prev_robot_name << " to " << model_name);
            pose.clear();
            tendon.clear();
        }
    }
    int message_id = 1000;
    for (auto p:pose_target) {
        publishMesh("robots", (model_name + "/meshes/visuals").c_str(), (p.first + ".stl").c_str(), p.second, 0.001,
                    "world", "pose_target", message_id++, 0, COLOR(0,1,0,0.3) );
        tf::Transform bt;
        PoseMsgToTF(p.second, bt);
        tf_broadcaster.sendTransform(tf::StampedTransform(bt, ros::Time::now(), "world", p.first));
    }
}

void CardsflowRviz::visualizeTendon() {
    int message_id = 10000;
    if (visualize_tendon) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "world";
        line_strip.ns = "tendon";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = (cable_thickness->value()/100.0)*0.05;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.lifetime = ros::Duration(1);
        for (auto t:tendon) {
            line_strip.header.stamp = ros::Time::now();
            line_strip.points.clear();
            line_strip.id = message_id++;
            for (int i = 1; i < t.second.viaPoints.size(); i++) {
                geometry_msgs::Point p;
                p.x = t.second.viaPoints[i - 1].x;
                p.y = t.second.viaPoints[i - 1].y;
                p.z = t.second.viaPoints[i - 1].z;
                line_strip.points.push_back(p);
                p.x = t.second.viaPoints[i].x;
                p.y = t.second.viaPoints[i].y;
                p.z = t.second.viaPoints[i].z;
                line_strip.points.push_back(p);
            }
            visualization_pub.publish(line_strip);
        }
    }
    message_id = 20000;
    if (visualize_tendon_length) {
        for (auto t:tendon) {
            if(t.second.viaPoints.empty())
                continue;
            Vector3d pos = convertGeometryToEigen(t.second.viaPoints[0]);
            char str[100];
            sprintf(str, "%s l=%.3fm ld=%.3fm/s", t.first.c_str(), t.second.l, t.second.ld);
            publishText(pos, str, "world", "tendon_length", message_id++, COLOR(1, 1, 1, 1), 1, (tendon_length_text_size->value()/100.0)*0.1);
        }
    }
    message_id = 30000;
    if (visualize_force) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.ns = "force";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration(1);
        arrow.scale.x = 0.005;
        arrow.scale.y = 0.01;
        arrow.scale.z = 0.01;
        arrow.pose.orientation.w = 1;
        arrow.pose.orientation.x = 0;
        arrow.pose.orientation.y = 0;
        arrow.pose.orientation.z = 0;

        arrow.action = visualization_msgs::Marker::ADD;

        for (auto t:tendon) {
            for (int i = 1; i < t.second.viaPoints.size(); i++) {
                geometry_msgs::Vector3 dir;
                dir.x = t.second.viaPoints[i].x - t.second.viaPoints[i - 1].x;
                dir.y = t.second.viaPoints[i].y - t.second.viaPoints[i - 1].y;
                dir.z = t.second.viaPoints[i].z - t.second.viaPoints[i - 1].z;
                // actio
                arrow.id = message_id++;
                arrow.color.r = 0.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.header.stamp = ros::Time::now();
                arrow.points.clear();
                geometry_msgs::Point p;
                p.x = t.second.viaPoints[i - 1].x;
                p.y = t.second.viaPoints[i - 1].y;
                p.z = t.second.viaPoints[i - 1].z;
                arrow.points.push_back(p);
                p.x += dir.x * t.second.force * 0.001; // show fraction of force
                p.y += dir.y * t.second.force * 0.001;
                p.z += dir.z * t.second.force * 0.001;
                arrow.points.push_back(p);
                visualization_pub.publish(arrow);
                // reactio
                arrow.id = message_id++;
                arrow.color.r = 1.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.header.stamp = ros::Time::now();
                arrow.points.clear();
                p.x = t.second.viaPoints[i].x;
                p.y = t.second.viaPoints[i].y;
                p.z = t.second.viaPoints[i].z;
                arrow.points.push_back(p);
                p.x -= dir.x * t.second.force * 0.001;
                p.y -= dir.y * t.second.force * 0.001;
                p.z -= dir.z * t.second.force * 0.001;
                arrow.points.push_back(p);
                visualization_pub.publish(arrow);
            }
        }
    }
}

void CardsflowRviz::visualizeTendonTarget() {
    int message_id = 15000;
    if (visualize_tendon_target) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "world";
        line_strip.ns = "tendon_target";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = (cable_thickness->value()/100.0)*0.05;
        line_strip.color.g = 1.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.lifetime = ros::Duration(0);
        for (auto t:tendon_target) {
            line_strip.header.stamp = ros::Time::now();
            line_strip.points.clear();
            line_strip.id = message_id++;
            for (int i = 1; i < t.second.viaPoints.size(); i++) {
                geometry_msgs::Point p;
                p.x = t.second.viaPoints[i - 1].x;
                p.y = t.second.viaPoints[i - 1].y;
                p.z = t.second.viaPoints[i - 1].z;
                line_strip.points.push_back(p);
                p.x = t.second.viaPoints[i].x;
                p.y = t.second.viaPoints[i].y;
                p.z = t.second.viaPoints[i].z;
                line_strip.points.push_back(p);
            }
            visualization_pub.publish(line_strip);
        }
    }
    message_id = 25000;
    if (visualize_tendon_length_target) {
        for (auto t:tendon_target) {
            if(t.second.viaPoints.empty())
                continue;
            Vector3d pos = convertGeometryToEigen(t.second.viaPoints[0]);
            char str[100];
            sprintf(str, "l=%.3fm ld=%.3fm/s", t.second.l, t.second.ld);
            publishText(pos, str, "world", "tendon_length_target", message_id++, COLOR(1, 1, 1, 1), 1, (tendon_length_text_size->value()/100.0)*0.1);
        }
    }
    message_id = 35000;
    if (visualize_force_target) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.ns = "force_target";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration(0);
        arrow.scale.x = 0.005;
        arrow.scale.y = 0.01;
        arrow.scale.z = 0.01;
        arrow.pose.orientation.w = 1;
        arrow.pose.orientation.x = 0;
        arrow.pose.orientation.y = 0;
        arrow.pose.orientation.z = 0;

        arrow.action = visualization_msgs::Marker::ADD;

        for (auto t:tendon_target) {
            for (int i = 1; i < t.second.viaPoints.size(); i++) {
                geometry_msgs::Vector3 dir;
                dir.x = t.second.viaPoints[i].x - t.second.viaPoints[i - 1].x;
                dir.y = t.second.viaPoints[i].y - t.second.viaPoints[i - 1].y;
                dir.z = t.second.viaPoints[i].z - t.second.viaPoints[i - 1].z;
                // actio
                arrow.id = message_id++;
                arrow.color.r = 0.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.header.stamp = ros::Time::now();
                arrow.points.clear();
                geometry_msgs::Point p;
                p.x = t.second.viaPoints[i - 1].x;
                p.y = t.second.viaPoints[i - 1].y;
                p.z = t.second.viaPoints[i - 1].z;
                arrow.points.push_back(p);
                p.x += dir.x * t.second.force * 0.001; // show fraction of force
                p.y += dir.y * t.second.force * 0.001;
                p.z += dir.z * t.second.force * 0.001;
                arrow.points.push_back(p);
                visualization_pub.publish(arrow);
                // reactio
                arrow.id = message_id++;
                arrow.color.r = 1.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.header.stamp = ros::Time::now();
                arrow.points.clear();
                p.x = t.second.viaPoints[i].x;
                p.y = t.second.viaPoints[i].y;
                p.z = t.second.viaPoints[i].z;
                arrow.points.push_back(p);
                p.x -= dir.x * t.second.force * 0.001;
                p.y -= dir.y * t.second.force * 0.001;
                p.z -= dir.z * t.second.force * 0.001;
                arrow.points.push_back(p);
                visualization_pub.publish(arrow);
            }
        }
    }
}

void CardsflowRviz::visualizeTorque() {
    int message_id = 40000;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.ns = "torque";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration(1);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.01;
    arrow.scale.z = 0.01;
    arrow.pose.orientation.w = 1;
    arrow.pose.orientation.x = 0;
    arrow.pose.orientation.y = 0;
    arrow.pose.orientation.z = 0;

    arrow.action = visualization_msgs::Marker::ADD;

    for (auto origin:joint_origin) {
        geometry_msgs::Vector3 axis = joint_axis[origin.first];
        double t = torque[origin.first];
        arrow.id = message_id++;
        arrow.color.r = 1.0f;
        arrow.color.g = 0.0f;
        arrow.color.b = 0.0f;
        arrow.header.stamp = ros::Time::now();
        arrow.points.clear();
        geometry_msgs::Point p;
        p.x = origin.second.x;
        p.y = origin.second.y;
        p.z = origin.second.z;
        arrow.points.push_back(p);
        p.x += axis.x * t * 0.0001; // show fraction of torque
        p.y += axis.y * t * 0.0001;
        p.z += axis.z * t * 0.0001;
        arrow.points.push_back(p);
        visualization_pub.publish(arrow);
    }
}

void CardsflowRviz::visualizeTorqueTarget() {
    int message_id = 45000;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.ns = "torque_target";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration(1);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.01;
    arrow.scale.z = 0.01;
    arrow.pose.orientation.w = 1;
    arrow.pose.orientation.x = 0;
    arrow.pose.orientation.y = 0;
    arrow.pose.orientation.z = 0;

    arrow.action = visualization_msgs::Marker::ADD;

    for (auto origin:joint_origin_target) {
        geometry_msgs::Vector3 axis = joint_axis_target[origin.first];
        double t = torque_target[origin.first];
        arrow.id = message_id++;
        arrow.color.r = 1.0f;
        arrow.color.g = 0.0f;
        arrow.color.b = 0.0f;
        arrow.header.stamp = ros::Time::now();
        arrow.points.clear();
        geometry_msgs::Point p;
        p.x = origin.second.x;
        p.y = origin.second.y;
        p.z = origin.second.z;
        arrow.points.push_back(p);
        p.x += axis.x * t * 0.0001; // show fraction of torque
        p.y += axis.y * t * 0.0001;
        p.z += axis.z * t * 0.0001;
        arrow.points.push_back(p);
        visualization_pub.publish(arrow);
    }
}

PLUGINLIB_EXPORT_CLASS(CardsflowRviz, rviz::Panel)
