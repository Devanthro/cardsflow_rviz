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

    QPushButton *sm = new QPushButton(tr("show_mesh"));
    connect(sm, SIGNAL(clicked()), this, SLOT(show_mesh()));
    connectWidget->layout()->addWidget(sm);

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

}

CardsflowRviz::~CardsflowRviz() {

}

void CardsflowRviz::save(rviz::Config config) const {
//        QLineEdit *w = this->findChild<QLineEdit *>("IP");
//        config.mapSetValue(w->objectName(), w->text());
//        w = this->findChild<QLineEdit *>("client_port");
//        config.mapSetValue(w->objectName(), w->text());
//        rviz::Panel::save(config);
}

void CardsflowRviz::load(const rviz::Config &config) {
//        rviz::Panel::load(config);
//        QLineEdit *w = this->findChild<QLineEdit *>("IP");
//        QVariant text;
//        config.mapGetValue(w->objectName(), &text);
//        w->setText(text.toString());
//        w = this->findChild<QLineEdit *>("client_port");
//        config.mapGetValue(w->objectName(), &text);
//        w->setText(text.toString());
}

PLUGINLIB_EXPORT_CLASS(CardsflowRviz, rviz::Panel)
