#include "rqt_roi_viewpoint_planner/roi_viewpoint_planner_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMessageBox>
#include <QTimer>
#include <QtGlobal>
#include <QThread>
#include <QFileDialog>
#include <yaml-cpp/yaml.h>

Q_DECLARE_METATYPE(roi_viewpoint_planner::PlannerConfig)
Q_DECLARE_METATYPE(roi_viewpoint_planner_msgs::PlannerStateConstPtr)

namespace rqt_roi_viewpoint_planner
{

RoiViewpointPlannerRqtPlugin::RoiViewpointPlannerRqtPlugin() :
  rqt_gui_cpp::Plugin(),
  widget(0)
{
  setObjectName("RoiViewpointPlannerRqtPlugin");
}

void RoiViewpointPlannerRqtPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui.setupUi(widget);
  // add widget to the user interface
  context.addWidget(widget);

  qRegisterMetaType<roi_viewpoint_planner::PlannerConfig>();
  qRegisterMetaType<roi_viewpoint_planner_msgs::PlannerStateConstPtr>();

  connect(this, SIGNAL(configChangedSignal(const roi_viewpoint_planner::PlannerConfig&)), this, SLOT(configChanged(const roi_viewpoint_planner::PlannerConfig&)));
  connect(this, SIGNAL(planRequestSignal(bool)), this, SLOT(planRequest(bool)));
  connect(this, SIGNAL(plannerStateSignal(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &)), this, SLOT(plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &)));

  connect(ui.saveMapPushButton, SIGNAL(clicked()), this, SLOT(on_saveMapPushButton_clicked()));
  connect(ui.loadMapPushButton, SIGNAL(clicked()), this, SLOT(on_loadMapPushButton_clicked()));
  connect(ui.resetMapPushButton, SIGNAL(clicked()), this, SLOT(on_resetMapPushButton_clicked()));
  connect(ui.moveToHomePushButton, SIGNAL(clicked()), this, SLOT(on_moveToHomePushButton_clicked()));
  connect(ui.moveToTransportPushButton, SIGNAL(clicked()), this, SLOT(on_moveToTransportPushButton_clicked()));

  saveOctomapClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::SaveOctomap>("/roi_viewpoint_planner/save_octomap");
  loadOctomapClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::LoadOctomap>("/roi_viewpoint_planner/load_octomap");
  resetOctomapClient = getNodeHandle().serviceClient<std_srvs::Empty>("/roi_viewpoint_planner/reset_octomap");
  moveToStateClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::MoveToState>("/roi_viewpoint_planner/move_to_state");

  plannerStateSub = getNodeHandle().subscribe("/roi_viewpoint_planner/planner_state", 10, &RoiViewpointPlannerRqtPlugin::plannerStateCallback, this);

  confirmPlanExecutionServer = getNodeHandle().advertiseService("/roi_viewpoint_planner/request_execution_confirmation", &RoiViewpointPlannerRqtPlugin::confirmPlanExecutionCallback, this);

  configClient = new dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig>("/roi_viewpoint_planner",
                 boost::bind(&RoiViewpointPlannerRqtPlugin::configCallback, this, _1));

  //ROS_INFO_STREAM("Init is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));

  initConfigGui();
}

void RoiViewpointPlannerRqtPlugin::initConfigGui()
{
    const std::vector<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr> &params = roi_viewpoint_planner::PlannerConfig::__getParamDescriptions__();
    //const std::vector<roi_viewpoint_planner::PlannerConfig::AbstractGroupDescriptionConstPtr> &groups = roi_viewpoint_planner::PlannerConfig::__getGroupDescriptions__();
    const roi_viewpoint_planner::PlannerConfig &config_def = roi_viewpoint_planner::PlannerConfig::__getDefault__();
    const roi_viewpoint_planner::PlannerConfig &config_min = roi_viewpoint_planner::PlannerConfig::__getMin__();
    const roi_viewpoint_planner::PlannerConfig &config_max = roi_viewpoint_planner::PlannerConfig::__getMax__();
    for (const roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr &param : params)
    {
        parameter_map[param->name] = param;
        boost::any val, min, max;
        param->getValue(config_def, val);
        param->getValue(config_max, max);
        param->getValue(config_min, min);
        if (param->edit_method != "") // param is enum
        {
            initEnumParam(param->name, param->edit_method, param->type, val);
        }
        else if (param->type == "bool")
        {
            initBoolParam(param->name, boost::any_cast<bool>(val));
        }
        else if (param->type == "int")
        {
            initIntParam(param->name, boost::any_cast<int>(val), boost::any_cast<int>(min), boost::any_cast<int>(max));
        }
        else if (param->type == "double")
        {
            initDoubleParam(param->name, boost::any_cast<double>(val), boost::any_cast<double>(min), boost::any_cast<double>(max));
        }
        else if (param->type == "str")
        {
            initStringParam(param->name, boost::any_cast<std::string>(val));
        }
        else
        {
            ROS_WARN_STREAM("Type " << param->type << " of parameter " << param->name << " not implemented");
        }
        //ROS_INFO_STREAM("Found param " << param->name << " of type " << param->type);
    }
    /*for (const roi_viewpoint_planner::PlannerConfig::AbstractGroupDescriptionConstPtr &group : groups)
    {
        ROS_INFO_STREAM("Found group " << group->name);
        for (const auto &param : group->parameters)
        {
            ROS_INFO_STREAM("Found param " << param.name << " of type " << param.type << " in group " << group->name);
        }
    }*/

}

void RoiViewpointPlannerRqtPlugin::initEnumParam(const std::string &name, const std::string &enum_description_str, const std::string &type, const boost::any &val)
{
    QComboBox *cb = new QComboBox();
    YAML::Node enum_description = YAML::Load(enum_description_str);
    for (const YAML::Node &node : enum_description["enum"])
    {
        QString entry_name = QString::fromStdString(node["name"].as<std::string>());
        QVariant userdata;
        if (type == "bool")
        {
            userdata = QVariant(node["value"].as<bool>());
        }
        else if (type == "int")
        {
            userdata = QVariant(node["value"].as<int>());
        }
        else if (type == "double")
        {
            userdata = QVariant(node["value"].as<double>());
        }
        else if (type == "str")
        {
            userdata = QVariant(QString::fromStdString(node["value"].as<std::string>()));
        }
        entry_name += QString(" (") + userdata.toString() + QString(")");
        cb->addItem(entry_name, userdata);
    }

    QVariant default_val;
    if (type == "bool")
    {
        default_val = QVariant(boost::any_cast<bool>(val));
        connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_boolComboBox_activated, this, cb, name, _1));
    }
    else if (type == "int")
    {
        default_val = QVariant(boost::any_cast<int>(val));
        connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intComboBox_activated, this, cb, name, _1));
    }
    else if (type == "double")
    {
        default_val = QVariant(boost::any_cast<double>(val));
        connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleComboBox_activated, this, cb, name, _1));
    }
    else if (type == "str")
    {
        default_val = QVariant(QString::fromStdString(boost::any_cast<std::string>(val)));
        connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_strComboBox_activated, this, cb, name, _1));
    }
    int default_index = cb->findData(default_val);
    if (default_index >= 0)
        cb->setCurrentIndex(default_index);

    comboBox_map[name] = cb;

    ui.configLayout->addRow(new QLabel(QString::fromStdString(name)), cb);
}

void RoiViewpointPlannerRqtPlugin::initBoolParam(const std::string &name, bool val)
{
    QCheckBox *cb = new QCheckBox();
    cb->setChecked(val);
    connect(cb, &QCheckBox::clicked, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_checkBox_clicked, this, name, _1));
    checkBox_map[name] = cb;
    ui.configLayout->addRow(new QLabel(QString::fromStdString(name)), cb);
}

void RoiViewpointPlannerRqtPlugin::initIntParam(const std::string &name, int val, int min, int max)
{
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *minLabel = new QLabel(QString::number(min));
    QLabel *maxLabel = new QLabel(QString::number(max));
    QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
    slider->setMaximum(100);
    QSpinBox *spinBox = new QSpinBox();
    layout->addWidget(minLabel);
    layout->addWidget(slider);
    layout->addWidget(maxLabel);
    layout->addWidget(spinBox);
    connect(slider, &QSlider::sliderMoved, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intSlider_sliderMoved, this, spinBox, name, _1));
    connect(slider, &QSlider::sliderReleased, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intSlider_sliderReleased, this, spinBox, name));
    connect(spinBox, &QSpinBox::editingFinished, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intSpinBox_editingFinished, this, spinBox, slider, name));
    slider_map[name] = slider;
    spinBox_map[name] = spinBox;
    ui.configLayout->addRow(new QLabel(QString::fromStdString(name)), layout);
}

void RoiViewpointPlannerRqtPlugin::initDoubleParam(const std::string &name, double val, double min, double max)
{
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *minLabel = new QLabel(QString::number(min));
    QLabel *maxLabel = new QLabel(QString::number(max));
    QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
    slider->setMaximum(100);
    QDoubleSpinBox *spinBox = new QDoubleSpinBox();
    layout->addWidget(minLabel);
    layout->addWidget(slider);
    layout->addWidget(maxLabel);
    layout->addWidget(spinBox);
    connect(slider, &QSlider::sliderMoved, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderMoved, this, spinBox, name, _1));
    connect(slider, &QSlider::sliderReleased, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderReleased, this, spinBox, name));
    connect(spinBox, &QDoubleSpinBox::editingFinished, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleSpinBox_editingFinished, this, spinBox, slider, name));
    slider_map[name] = slider;
    spinBox_map[name] = spinBox;
    ui.configLayout->addRow(new QLabel(QString::fromStdString(name)), layout);
}

void RoiViewpointPlannerRqtPlugin::initStringParam(const std::string &name, const std::string &val)
{
    QLineEdit *le = new QLineEdit();
    le->setText(QString::fromStdString(val));
    connect(le, &QLineEdit::textEdited, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_lineEdit_textEdited, this, name, _1));
    lineEdit_map[name] = le;
    ui.configLayout->addRow(new QLabel(QString::fromStdString(name)), le);
}

void RoiViewpointPlannerRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  plannerStateSub.shutdown();
  confirmPlanExecutionServer.shutdown();
  saveOctomapClient.shutdown();
}

void RoiViewpointPlannerRqtPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void RoiViewpointPlannerRqtPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void RoiViewpointPlannerRqtPlugin::intSlider_setValue(QSlider *slider, const std::string &param, int value)
{
  int minVal = getMin<int>(param);
  int maxVal = getMax<int>(param);
  int position = qBound(0, static_cast<int>(static_cast<double>(value - minVal) / static_cast<double>(maxVal - minVal) * 100.0), 100);
  slider->setValue(position);
}

void RoiViewpointPlannerRqtPlugin::intSpinBox_setPosition(QSpinBox *spinBox, const std::string &param, int position)
{
  int minVal = getMin<int>(param);
  int maxVal = getMax<int>(param);
  spinBox->setValue(minVal + position * (maxVal - minVal) / 100);
}

void RoiViewpointPlannerRqtPlugin::intValue_sendConfig(QSpinBox *spinBox, const std::string &param)
{
  setValue<int>(param, spinBox->value());
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::on_intSlider_sliderMoved(QSpinBox *spinBox, const std::string &param, int position)
{
  intSpinBox_setPosition(spinBox, param, position);
}

void RoiViewpointPlannerRqtPlugin::on_intSlider_sliderReleased(QSpinBox *spinBox, const std::string &param)
{
  intValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::on_intSpinBox_editingFinished(QSpinBox *spinBox, QSlider *slider, const std::string &param)
{
  intSlider_setValue(slider, param, spinBox->value());
  intValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::doubleSlider_setValue(QSlider *slider, const std::string &param, double value)
{
  double minVal = getMin<double>(param);
  double maxVal = getMax<double>(param);
  int position = qBound(0, static_cast<int>((value - minVal) / (maxVal - minVal) * 100.0), 100);
  slider->setValue(position);
}

void RoiViewpointPlannerRqtPlugin::doubleSpinBox_setPosition(QDoubleSpinBox *spinBox, const std::string &param, int position)
{
  double minVal = getMin<double>(param);
  double maxVal = getMax<double>(param);
  spinBox->setValue(minVal + static_cast<double>(position) / 100.0 * (maxVal - minVal));
}

void RoiViewpointPlannerRqtPlugin::doubleValue_sendConfig(QDoubleSpinBox *spinBox, const std::string &param)
{
  setValue<double>(param, spinBox->value());
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderMoved(QDoubleSpinBox *spinBox, const std::string &param, int position)
{
  doubleSpinBox_setPosition(spinBox, param, position);
}

void RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderReleased(QDoubleSpinBox *spinBox, const std::string &param)
{
  doubleValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::on_doubleSpinBox_editingFinished(QDoubleSpinBox *spinBox, QSlider *slider, const std::string &param)
{
  doubleSlider_setValue(slider, param, spinBox->value());
  doubleValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::on_boolComboBox_activated(QComboBox *comboBox, const std::string &param, int index)
{
  QVariant val = comboBox->itemData(index);
  setValue<bool>(param, val.toBool());
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::on_intComboBox_activated(QComboBox *comboBox, const std::string &param, int index)
{
  QVariant val = comboBox->itemData(index);
  setValue<int>(param, val.toInt());
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::on_doubleComboBox_activated(QComboBox *comboBox, const std::string &param, int index)
{
  QVariant val = comboBox->itemData(index);
  setValue<double>(param, val.toDouble());
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::on_strComboBox_activated(QComboBox *comboBox, const std::string &param, int index)
{
  QVariant val = comboBox->itemData(index);
  setValue<std::string>(param, val.toString().toStdString());
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::on_checkBox_clicked(const std::string &param, bool checked)
{
  setValue<bool>(param, checked);
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::on_lineEdit_textEdited(const std::string &param, const QString &text)
{
  setValue<std::string>(param, text.toStdString());
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText(QString::fromStdString(param) + " change failed");
    return;
  }
  ui.statusTextBox->setText(QString::fromStdString(param) + " change successful");
}

void RoiViewpointPlannerRqtPlugin::configChanged(const roi_viewpoint_planner::PlannerConfig &received_config)
{
  // ROS_INFO_STREAM("Config changed slot is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  current_config = received_config;
  for (const auto &pair : parameter_map)
  {
    const std::string &name = pair.first;
    const roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr &param = pair.second;
    if (param->edit_method != "") // param is enum
    {
        QComboBox *cb = comboBox_map[name];
        QVariant val;
        if (param->type == "bool")
            val = QVariant(getValue<bool>(name));
        else if (param->type == "int")
            val = QVariant(getValue<int>(name));
        else if (param->type == "double")
            val = QVariant(getValue<double>(name));
        else if (param->type == "str")
            val = QVariant(QString::fromStdString(getValue<std::string>(name)));

        int index = cb->findData(val);
        if (index >= 0)
            cb->setCurrentIndex(index);
    }
    else if (param->type == "bool")
    {
        QCheckBox *cb = checkBox_map[name];
        cb->setChecked(getValue<bool>(name));
    }
    else if (param->type == "int")
    {
        QSlider *slider = slider_map[name];
        QSpinBox *spinBox = reinterpret_cast<QSpinBox*>(spinBox_map[name]);
        intSlider_setValue(slider, name, getValue<int>(name));
        spinBox->setValue(getValue<int>(name));
    }
    else if (param->type == "double")
    {
        QSlider *slider = slider_map[name];
        QDoubleSpinBox *spinBox = reinterpret_cast<QDoubleSpinBox*>(spinBox_map[name]);
        doubleSlider_setValue(slider, name, getValue<double>(name));
        spinBox->setValue(getValue<double>(name));
    }
    else if (param->type == "str")
    {
        QLineEdit *le = lineEdit_map[name];
        le->setText(QString::fromStdString(getValue<std::string>(name)));
    }
    else
    {
        ROS_WARN_STREAM("Type " << param->type << " of parameter " << param->name << " not implemented");
    }
  }
  ui.moveToHomePushButton->setEnabled(current_config.mode < 2);
  ui.moveToTransportPushButton->setEnabled(current_config.mode < 2);
}

void RoiViewpointPlannerRqtPlugin::planRequest(bool enable)
{
  ROS_INFO_STREAM("Plan request slot is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  ui.planRequestLed->setState(enable);
  ui.planAcceptPushButton->setEnabled(enable);
  ui.planDeclinePushButton->setEnabled(enable);
  if (enable)
    ui.statusTextBox->setText("Plan requested; check rviz for plan");
  else
    ui.statusTextBox->setText("Plan request answered");
}

void RoiViewpointPlannerRqtPlugin::plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state)
{
  ROS_INFO_STREAM("Planner state slot is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  ui.planningLed->setState(false);
  ui.movingLed->setState(state->robot_is_moving);
  ui.occScanLed->setState(state->occupancy_scanned);
  ui.roiScanLed->setState(state->roi_scanned);
}

void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc)
{
  ROS_INFO_STREAM("Description callback called");
}

// DO NOT DIRECTLY UPDATE UI ELEMENTS HERE
void RoiViewpointPlannerRqtPlugin::configCallback(const roi_viewpoint_planner::PlannerConfig &conf)
{
  ROS_INFO_STREAM("Config callback is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  ROS_INFO_STREAM("Config callback called");
  emit configChangedSignal(conf);
}

// DO NOT DIRECTLY UPDATE UI ELEMENTS HERE
bool RoiViewpointPlannerRqtPlugin::confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO_STREAM("Confirm plan callback is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  emit planRequestSignal(true);

  QEventLoop loop;
  bool accepted = false;
  connect( ui.planAcceptPushButton, &QPushButton::clicked, [&]() { accepted = true; loop.quit(); });
  connect( ui.planDeclinePushButton, &QPushButton::clicked, [&]() { accepted = false; loop.quit(); });
  ROS_INFO_STREAM("Loop starting");
  loop.exec();
  ROS_INFO_STREAM("Loop left");
  emit planRequestSignal(false);
  ROS_INFO_STREAM("Plan request signal emitted starting");
  if (accepted)
    res.success = true;
  else
    res.success = false;

  return true;
}

// DO NOT DIRECTLY UPDATE UI ELEMENTS HERE
void RoiViewpointPlannerRqtPlugin::plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state)
{
  //ROS_INFO_STREAM("Planner state callback is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  emit plannerStateSignal(state);
}

void RoiViewpointPlannerRqtPlugin::on_saveMapPushButton_clicked()
{
  roi_viewpoint_planner_msgs::SaveOctomap srv;
  if (saveOctomapClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Map saved successfully");
    else
      ui.statusTextBox->setText("Map could not be saved");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call save map service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_loadMapPushButton_clicked()
{
  QString file_path = QFileDialog::getOpenFileName(widget, QString(), QString(), "Octree (*.ot)");
  if (file_path.isEmpty())
    return;

  roi_viewpoint_planner_msgs::LoadOctomap srv;
  srv.request.filename = file_path.toStdString();
  if (loadOctomapClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Map loaded successfully");
    else
      ui.statusTextBox->setText("Error loading map: " + QString::fromStdString(srv.response.error_message));
  }
  else
  {
    ui.statusTextBox->setText("Failed to call load map service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_resetMapPushButton_clicked()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(widget, "Confirm reset", "Do you really want to delete the current map?", QMessageBox::Yes|QMessageBox::No);
  if (reply == QMessageBox::No)
    return;

  std_srvs::Empty srv;
  if (resetOctomapClient.call(srv))
  {
    ui.statusTextBox->setText("Map reset successfully");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call load map service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_moveToHomePushButton_clicked()
{
  roi_viewpoint_planner_msgs::MoveToState srv;
  srv.request.joint_values = {0, -0.7854, 1.5708, -0.7854, 0, 0};
  if (moveToStateClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Moving to home position");
    else
      ui.statusTextBox->setText("Couldn't plan to specified position");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call move to state service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_moveToTransportPushButton_clicked()
{
  roi_viewpoint_planner_msgs::MoveToState srv;
  srv.request.joint_values = {0, 0, 0, 0, -1.5707, 0};
  if (moveToStateClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Moving to transport position");
    else
      ui.statusTextBox->setText("Couldn't plan to specified position");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call move to state service");
  }
}

} // namespace roi_viewpoint_planner_rqt_plugin

PLUGINLIB_EXPORT_CLASS(rqt_roi_viewpoint_planner::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(roi_viewpoint_planner_rqt_plugin, RoiViewpointPlannerRqtPlugin, roi_viewpoint_planner_rqt_plugin::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
