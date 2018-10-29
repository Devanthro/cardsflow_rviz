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
    connect(show_mesh_button, SIGNAL(clicked()), this, SLOT(show_mesh()));
    connectWidget->layout()->addWidget(show_mesh_button);

    show_tendon_button = new QPushButton(tr("show_tendon"));
    show_tendon_button->setObjectName("show_tendon");
    show_tendon_button->setCheckable(true);
    connect(show_tendon_button, SIGNAL(clicked()), this, SLOT(show_tendon()));
    connectWidget->layout()->addWidget(show_tendon_button);

    show_tendon_length_button = new QPushButton(tr("show_tendon_length"));
    show_tendon_length_button->setObjectName("show_tendon_length");
    show_tendon_length_button->setCheckable(true);
    connect(show_tendon_length_button, SIGNAL(clicked()), this, SLOT(show_tendon_length()));
    connectWidget->layout()->addWidget(show_tendon_length_button);

    show_force_button = new QPushButton(tr("show_force"));
    show_force_button->setObjectName("show_force");
    show_force_button->setCheckable(true);
    connect(show_force_button, SIGNAL(clicked()), this, SLOT(show_force()));
    connectWidget->layout()->addWidget(show_force_button);

    show_torque_button = new QPushButton(tr("show_torque"));
    show_torque_button->setObjectName("show_torque");
    show_torque_button->setCheckable(true);
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

    robot_state = nh->subscribe("/robot_state", 500, &CardsflowRviz::RobotState, this);
    tendon_state = nh->subscribe("/tendon_state", 500, &CardsflowRviz::TendonState, this);

    if (!nh->hasParam("robot_name"))
        ROS_FATAL("robot_name could not be found on parameter server!!! ");
    else
        nh->getParam("robot_name", robot_name);

    QObject::connect(this, SIGNAL(newData()), this, SLOT(visualize()));
}

CardsflowRviz::~CardsflowRviz() {
}

void CardsflowRviz::load(const rviz::Config &config) {
    rviz::Panel::load(config);
    QVariant ticked;
    config.mapGetValue(show_mesh_button->objectName(), &ticked);
    show_mesh_button->setChecked(ticked.toBool());
    config.mapGetValue(show_tendon_button->objectName(), &ticked);
    show_tendon_button->setChecked(ticked.toBool());
    config.mapGetValue(show_tendon_length_button->objectName(), &ticked);
    show_tendon_length_button->setChecked(ticked.toBool());
    config.mapGetValue(show_force_button->objectName(), &ticked);
    show_force_button->setChecked(ticked.toBool());
    config.mapGetValue(show_torque_button->objectName(), &ticked);
    show_torque_button->setChecked(ticked.toBool());
    show_mesh();
    show_tendon();
    show_force();
    show_torque();
}

void CardsflowRviz::save(rviz::Config config) const {
    config.mapSetValue(show_mesh_button->objectName(), show_mesh_button->isChecked());
    config.mapSetValue(show_tendon_button->objectName(), show_tendon_button->isChecked());
    config.mapSetValue(show_tendon_length_button->objectName(), show_tendon_length_button->isChecked());
    config.mapSetValue(show_force_button->objectName(), show_force_button->isChecked());
    config.mapSetValue(show_torque_button->objectName(), show_torque_button->isChecked());
    rviz::Panel::save(config);
}

void CardsflowRviz::show_mesh() {
    visualize_mesh = show_mesh_button->isChecked();
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
}

void CardsflowRviz::TendonState(const roboy_communication_simulation::TendonConstPtr &msg) {
    Tendon t;
    t.force = msg->force;
    t.l = msg->l;
    t.ld = msg->ld;
    t.viaPoints = msg->viaPoints;
    tendon[msg->name] = t;
    Q_EMIT newData();
}

void CardsflowRviz::visualize() {
    int message_id = 6666;
    if (visualize_mesh) {
        for (auto p:pose) {
            publishMesh("robots", (robot_name + "/meshes/CAD").c_str(), (p.first + ".stl").c_str(), p.second, 0.001,
                        "world", "mesh", message_id++, 1);
            tf::Transform bt;
            PoseMsgToTF(p.second,bt);
            tf_broadcaster.sendTransform(tf::StampedTransform(bt, ros::Time::now(), "world", p.first));
        }
    }
    if (visualize_tendon) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "world";
        line_strip.ns = "tendon";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.003;
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
//    if(visualize_tendon_length){
//        for (auto t:tendon) {
//            Vector3d pos ((t.second.viaPoints[1].x + t.second.viaPoints[0].x)/2.0,
//             (t.second.viaPoints[1].y + t.second.viaPoints[0].y)/2.0,
//             (t.second.viaPoints[1].z + t.second.viaPoints[0].z)/2.0);
//            char str[100];
//            sprintf(str, "%.3f", t.second.l);
//            publishText(pos,str,"world","tendon_length",message_id++,COLOR(1,1,1,1),1,0.01);
//        }
//    }
    if(visualize_force){
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
                dir.x = t.second.viaPoints[i].x-t.second.viaPoints[i - 1].x;
                dir.y = t.second.viaPoints[i].y-t.second.viaPoints[i - 1].y;
                dir.z = t.second.viaPoints[i].z-t.second.viaPoints[i - 1].z;
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
                p.x += dir.x * 0.001; // show fraction of force
                p.y += dir.y * 0.001;
                p.z += dir.z * 0.001;
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
                p.x -= dir.x * 0.001;
                p.y -= dir.y * 0.001;
                p.z -= dir.z * 0.001;
                arrow.points.push_back(p);
                visualization_pub.publish(arrow);
            }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(CardsflowRviz, rviz::Panel)
