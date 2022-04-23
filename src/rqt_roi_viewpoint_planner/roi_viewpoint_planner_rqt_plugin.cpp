#include "rqt_roi_viewpoint_planner/roi_viewpoint_planner_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMessageBox>
#include <QTimer>
#include <QtGlobal>
#include <QFileDialog>

Q_DECLARE_METATYPE(roi_viewpoint_planner::PlannerConfig)
Q_DECLARE_METATYPE(view_motion_planner::VmpConfig)
Q_DECLARE_METATYPE(roi_viewpoint_planner_msgs::PlannerStateConstPtr)

namespace rqt_roi_viewpoint_planner
{

RoiViewpointPlannerRqtPlugin::RoiViewpointPlannerRqtPlugin() :
  rqt_gui_cpp::Plugin(),
  widget(nullptr)
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
  qRegisterMetaType<view_motion_planner::VmpConfig>();
  qRegisterMetaType<roi_viewpoint_planner_msgs::PlannerStateConstPtr>();

  connect(this, SIGNAL(rvpConfigChangedSignal(const roi_viewpoint_planner::PlannerConfig&)), this, SLOT(rvpConfigChanged(const roi_viewpoint_planner::PlannerConfig&)));
  connect(this, SIGNAL(planRequestSignal(bool)), this, SLOT(planRequest(bool)));
  connect(this, SIGNAL(plannerStateSignal(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &)), this, SLOT(plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &)));

  connect(ui.saveMapPushButton, SIGNAL(clicked()), this, SLOT(on_saveMapPushButton_clicked()));
  connect(ui.loadMapPushButton, SIGNAL(clicked()), this, SLOT(on_loadMapPushButton_clicked()));
  connect(ui.resetMapPushButton, SIGNAL(clicked()), this, SLOT(on_resetMapPushButton_clicked()));
  connect(ui.moveArmPushButton, SIGNAL(clicked()), this, SLOT(on_moveArmPushButton_clicked()));
  connect(ui.randomizePlantsPushButton, SIGNAL(clicked()), this, SLOT(on_randomizePlantsPushButton_clicked()));
  connect(ui.startEvaluatorPushButton, SIGNAL(clicked()), this, SLOT(on_startEvaluatorPushButton_clicked()));

  moveToStateThread.reset(new MoveToStateThread(getNodeHandle()));
  connect(moveToStateThread.get(), SIGNAL(success(QString)), ui.statusTextBox, SLOT(setText(QString)));
  connect(moveToStateThread.get(), SIGNAL(error(QString)), ui.statusTextBox, SLOT(setText(QString)));

  saveOctomapClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::SaveOctomap>("/roi_viewpoint_planner/save_octomap");
  loadOctomapClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::LoadOctomap>("/roi_viewpoint_planner/load_octomap");
  resetOctomapClient = getNodeHandle().serviceClient<std_srvs::Empty>("/roi_viewpoint_planner/reset_octomap");
  randomizePlantPositionsClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::RandomizePlantPositions>("/roi_viewpoint_planner/randomize_plant_positions");
  startEvaluatorClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::StartEvaluator>("/roi_viewpoint_planner/start_evaluator");

  plannerStateSub = getNodeHandle().subscribe("/roi_viewpoint_planner/planner_state", 10, &RoiViewpointPlannerRqtPlugin::plannerStateCallback, this);

  confirmPlanExecutionServer = getNodeHandle().advertiseService("/roi_viewpoint_planner/request_execution_confirmation", &RoiViewpointPlannerRqtPlugin::confirmPlanExecutionCallback, this);

  rvpConfigClient = new dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig>("/roi_viewpoint_planner",
                 boost::bind(&RoiViewpointPlannerRqtPlugin::rvpConfigCallback, this, _1));

  vmpConfigClient = new dynamic_reconfigure::Client<view_motion_planner::VmpConfig>("/view_motion_planner",
                 boost::bind(&RoiViewpointPlannerRqtPlugin::vmpConfigCallback, this, _1));

  //ROS_INFO_STREAM("Init is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));

  initConfigGui();
}

void RoiViewpointPlannerRqtPlugin::initConfigGui()
{
    for (const roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr &param : roi_viewpoint_planner::PlannerConfig::__getParamDescriptions__())
    {
      AbstractParamPtr p = initializeParam(param, rvp_current_config);
      rvp_params.push_back(p);
      boost::any val, min, max;
      param->getValue(roi_viewpoint_planner::PlannerConfig::__getDefault__(), val);
      param->getValue(roi_viewpoint_planner::PlannerConfig::__getMax__(), max);
      param->getValue(roi_viewpoint_planner::PlannerConfig::__getMin__(), min);
      if (param->edit_method != "") // param is enum
      {
          initEnumParam(p, ui.configRvpLayout);
      }
      else if (param->type == "bool")
      {
          initBoolParam(p, ui.configRvpLayout);
      }
      else if (param->type == "int")
      {
          initIntParam(p, ui.configRvpLayout);
      }
      else if (param->type == "double")
      {
          initDoubleParam(p, ui.configRvpLayout);
      }
      else if (param->type == "str")
      {
          initStringParam(p, ui.configRvpLayout);
      }
      else
      {
          ROS_WARN_STREAM("Type " << param->type << " of parameter " << param->name << " not implemented");
      }
    }
    /*for (const roi_viewpoint_planner::PlannerConfig::AbstractGroupDescriptionConstPtr &group : roi_viewpoint_planner::PlannerConfig::__getGroupDescriptions__())
    {
        ROS_INFO_STREAM("Found group " << group->name);
        for (const auto &param : group->parameters)
        {
            ROS_INFO_STREAM("Found param " << param.name << " of type " << param.type << " in group " << group->name);
        }
    }*/
    for (const view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr &param : view_motion_planner::VmpConfig::__getParamDescriptions__())
    {
      AbstractParamPtr p = initializeParam(param, vmp_current_config);
      vmp_params.push_back(p);
      boost::any val, min, max;
      param->getValue(view_motion_planner::VmpConfig::__getDefault__(), val);
      param->getValue(view_motion_planner::VmpConfig::__getMax__(), max);
      param->getValue(view_motion_planner::VmpConfig::__getMin__(), min);
      if (param->edit_method != "") // param is enum
      {
          initEnumParam(p, ui.configVmpLayout);
      }
      else if (param->type == "bool")
      {
          initBoolParam(p, ui.configVmpLayout);
      }
      else if (param->type == "int")
      {
          initIntParam(p, ui.configVmpLayout);
      }
      else if (param->type == "double")
      {
          initDoubleParam(p, ui.configVmpLayout);
      }
      else if (param->type == "str")
      {
          initStringParam(p, ui.configVmpLayout);
      }
      else
      {
          ROS_WARN_STREAM("Type " << param->type << " of parameter " << param->name << " not implemented");
      }
    }
}

void RoiViewpointPlannerRqtPlugin::initEnumParam(const AbstractParamPtr &param, QFormLayout *configLayout)
{
  QComboBox *cb = new QComboBox();
  YAML::Node enum_description = YAML::Load(param->edit_method);
  for (const YAML::Node &node : enum_description["enum"])
  {
      QString entry_name = QString::fromStdString(node["name"].as<std::string>());
      QVariant userdata;
      if (param->type == "bool")
      {
          userdata = QVariant(node["value"].as<bool>());
      }
      else if (param->type == "int")
      {
          userdata = QVariant(node["value"].as<int>());
      }
      else if (param->type == "double")
      {
          userdata = QVariant(node["value"].as<double>());
      }
      else if (param->type == "str")
      {
          userdata = QVariant(QString::fromStdString(node["value"].as<std::string>()));
      }
      entry_name += QString(" (") + userdata.toString() + QString(")");
      cb->addItem(entry_name, userdata);
  }

  QVariant default_val;
  if (param->type == "bool")
  {
      default_val = QVariant(boost::any_cast<bool>(param->getDefault()));
      connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_boolComboBox_activated, this, cb, param, _1));
  }
  else if (param->type == "int")
  {
      default_val = QVariant(boost::any_cast<int>(param->getDefault()));
      connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intComboBox_activated, this, cb, param, _1));
  }
  else if (param->type == "double")
  {
      default_val = QVariant(boost::any_cast<double>(param->getDefault()));
      connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleComboBox_activated, this, cb, param, _1));
  }
  else if (param->type == "str")
  {
      default_val = QVariant(QString::fromStdString(boost::any_cast<std::string>(param->getDefault())));
      connect(cb, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_strComboBox_activated, this, cb, param, _1));
  }
  int default_index = cb->findData(default_val);
  if (default_index >= 0)
      cb->setCurrentIndex(default_index);

  param->widget = cb;

  configLayout->addRow(new QLabel(QString::fromStdString(param->name)), cb);
}

void RoiViewpointPlannerRqtPlugin::initBoolParam(const AbstractParamPtr &param, QFormLayout *configLayout)
{
  QCheckBox *cb = new QCheckBox();
  cb->setChecked(boost::any_cast<bool>(param->getDefault()));
  connect(cb, &QCheckBox::clicked, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_checkBox_clicked, this, param, _1));
  param->widget = cb;
  configLayout->addRow(new QLabel(QString::fromStdString(param->name)), cb);
}

void RoiViewpointPlannerRqtPlugin::initIntParam(const AbstractParamPtr &param, QFormLayout *configLayout)
{
  int min = boost::any_cast<int>(param->getMin());
  int max = boost::any_cast<int>(param->getMax());
  int def = boost::any_cast<int>(param->getDefault());
  QHBoxLayout *layout = new QHBoxLayout();
  QLabel *minLabel = new QLabel(QString::number(min));
  QLabel *maxLabel = new QLabel(QString::number(max));
  QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
  slider->setMaximum(100);
  intSlider_setValue(slider, param, boost::any_cast<int>(def));
  QSpinBox *spinBox = new QSpinBox();
  spinBox->setMinimum(boost::any_cast<int>(min));
  spinBox->setMaximum(boost::any_cast<int>(max));
  spinBox->setValue(boost::any_cast<int>(def));
  layout->addWidget(minLabel);
  layout->addWidget(slider);
  layout->addWidget(maxLabel);
  layout->addWidget(spinBox);
  connect(slider, &QSlider::sliderMoved, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intSlider_sliderMoved, this, spinBox, param, _1));
  connect(slider, &QSlider::sliderReleased, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intSlider_sliderReleased, this, spinBox, param));
  connect(spinBox, &QSpinBox::editingFinished, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_intSpinBox_editingFinished, this, spinBox, slider, param));
  configLayout->addRow(new QLabel(QString::fromStdString(param->name)), layout);
  param->widget = spinBox;
  param->slider = slider;
}

void RoiViewpointPlannerRqtPlugin::initDoubleParam(const AbstractParamPtr &param, QFormLayout *configLayout)
{
  double min = boost::any_cast<double>(param->getMin());
  double max = boost::any_cast<double>(param->getMax());
  double def = boost::any_cast<double>(param->getDefault());
  QHBoxLayout *layout = new QHBoxLayout();
  QLabel *minLabel = new QLabel(QString::number(min));
  QLabel *maxLabel = new QLabel(QString::number(max));
  QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
  slider->setMaximum(100);
  doubleSlider_setValue(slider, param, def);
  QDoubleSpinBox *spinBox = new QDoubleSpinBox();
  spinBox->setMinimum(min);
  spinBox->setMaximum(max);
  int exponent = QString::number(max-min, 'e', 0).split('e').last().toInt();
  spinBox->setSingleStep(std::pow(10.0, exponent-1));
  spinBox->setValue(def);
  layout->addWidget(minLabel);
  layout->addWidget(slider);
  layout->addWidget(maxLabel);
  layout->addWidget(spinBox);
  connect(slider, &QSlider::sliderMoved, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderMoved, this, spinBox, param, _1));
  connect(slider, &QSlider::sliderReleased, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderReleased, this, spinBox, param));
  connect(spinBox, &QDoubleSpinBox::editingFinished, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_doubleSpinBox_editingFinished, this, spinBox, slider, param));
  configLayout->addRow(new QLabel(QString::fromStdString(param->name)), layout);
  param->widget = spinBox;
  param->slider = slider;
}

void RoiViewpointPlannerRqtPlugin::initStringParam(const AbstractParamPtr &param, QFormLayout *configLayout)
{
  QLineEdit *le = new QLineEdit();
  le->setText(QString::fromStdString(boost::any_cast<std::string>(param->getDefault())));
  connect(le, &QLineEdit::textEdited, this, boost::bind(&RoiViewpointPlannerRqtPlugin::on_lineEdit_textEdited, this, param, _1));
  configLayout->addRow(new QLabel(QString::fromStdString(param->name)), le);
  param->widget = le;
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

void RoiViewpointPlannerRqtPlugin::intSlider_setValue(QSlider *slider, const AbstractParamPtr &param, int value)
{
  int minVal = boost::any_cast<int>(param->getMin());
  int maxVal = boost::any_cast<int>(param->getMax());
  int position = qBound(0, static_cast<int>(static_cast<double>(value - minVal) / static_cast<double>(maxVal - minVal) * 100.0), 100);
  slider->setValue(position);
}

void RoiViewpointPlannerRqtPlugin::intSpinBox_setPosition(QSpinBox *spinBox, const AbstractParamPtr &param, int position)
{
  int minVal = boost::any_cast<int>(param->getMin());
  int maxVal = boost::any_cast<int>(param->getMax());
  spinBox->setValue(minVal + position * (maxVal - minVal) / 100);
}

void RoiViewpointPlannerRqtPlugin::intValue_sendConfig(QSpinBox *spinBox, const AbstractParamPtr &param)
{
  param->setValue(spinBox->value());
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::on_intSlider_sliderMoved(QSpinBox *spinBox, const AbstractParamPtr &param, int position)
{
  intSpinBox_setPosition(spinBox, param, position);
}

void RoiViewpointPlannerRqtPlugin::on_intSlider_sliderReleased(QSpinBox *spinBox, const AbstractParamPtr &param)
{
  intValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::on_intSpinBox_editingFinished(QSpinBox *spinBox, QSlider *slider, const AbstractParamPtr &param)
{
  intSlider_setValue(slider, param, spinBox->value());
  intValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::doubleSlider_setValue(QSlider *slider, const AbstractParamPtr &param, double value)
{
  double minVal = boost::any_cast<double>(param->getMin());
  double maxVal = boost::any_cast<double>(param->getMax());
  int position = qBound(0, static_cast<int>((value - minVal) / (maxVal - minVal) * 100.0), 100);
  slider->setValue(position);
}

void RoiViewpointPlannerRqtPlugin::doubleSpinBox_setPosition(QDoubleSpinBox *spinBox, const AbstractParamPtr &param, int position)
{
  double minVal = boost::any_cast<double>(param->getMin());
  double maxVal = boost::any_cast<double>(param->getMax());
  spinBox->setValue(minVal + static_cast<double>(position) / 100.0 * (maxVal - minVal));
}

void RoiViewpointPlannerRqtPlugin::doubleValue_sendConfig(QDoubleSpinBox *spinBox, const AbstractParamPtr &param)
{
  param->setValue(spinBox->value());
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderMoved(QDoubleSpinBox *spinBox, const AbstractParamPtr &param, int position)
{
  doubleSpinBox_setPosition(spinBox, param, position);
}

void RoiViewpointPlannerRqtPlugin::on_doubleSlider_sliderReleased(QDoubleSpinBox *spinBox, const AbstractParamPtr &param)
{
  doubleValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::on_doubleSpinBox_editingFinished(QDoubleSpinBox *spinBox, QSlider *slider, const AbstractParamPtr &param)
{
  doubleSlider_setValue(slider, param, spinBox->value());
  doubleValue_sendConfig(spinBox, param);
}

void RoiViewpointPlannerRqtPlugin::on_boolComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index)
{
  QVariant val = comboBox->itemData(index);
  param->setValue(val.toBool());
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::on_intComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index)
{
  QVariant val = comboBox->itemData(index);
  param->setValue(val.toInt());
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::on_doubleComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index)
{
  QVariant val = comboBox->itemData(index);
  param->setValue(val.toDouble());
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::on_strComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index)
{
  QVariant val = comboBox->itemData(index);
  param->setValue(val.toString().toStdString());
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::on_checkBox_clicked(const AbstractParamPtr &param, bool checked)
{
  param->setValue(checked);
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::on_lineEdit_textEdited(const AbstractParamPtr &param, const QString &text)
{
  param->setValue(text.toStdString());
  sendConfig(param);
}

void RoiViewpointPlannerRqtPlugin::sendConfig(const AbstractParamPtr &changed_param)
{
  if (dynamic_cast<AbstractConfigParam<roi_viewpoint_planner::PlannerConfig>*>(changed_param.get()))
  {
    if (!rvpConfigClient->setConfiguration(rvp_current_config))
      ui.statusTextBox->setText(QString::fromStdString(changed_param->name) + " change failed");
    else
      ui.statusTextBox->setText(QString::fromStdString(changed_param->name) + " change successful");
    return;
  }
  if (dynamic_cast<AbstractConfigParam<view_motion_planner::VmpConfig>*>(changed_param.get()))
  {
    if (!vmpConfigClient->setConfiguration(vmp_current_config))
      ui.statusTextBox->setText(QString::fromStdString(changed_param->name) + " change failed");
    else
      ui.statusTextBox->setText(QString::fromStdString(changed_param->name) + " change successful");
    return;
  }

}

void RoiViewpointPlannerRqtPlugin::rvpConfigChanged(const roi_viewpoint_planner::PlannerConfig &received_config)
{
  // ROS_INFO_STREAM("Config changed slot is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  rvp_current_config = received_config;
  for (const AbstractParamPtr &param : rvp_params)
  {
    if (param->edit_method != "") // param is enum
    {
        QComboBox *cb = reinterpret_cast<QComboBox*>(param->widget);
        QVariant val;
        if (param->type == "bool")
            val = QVariant(boost::any_cast<bool>(param->getValue()));
        else if (param->type == "int")
            val = QVariant(boost::any_cast<int>(param->getValue()));
        else if (param->type == "double")
            val = QVariant((boost::any_cast<double>(param->getValue())));
        else if (param->type == "str")
            val = QVariant(QString::fromStdString(boost::any_cast<std::string>(param->getValue())));

        int index = cb->findData(val);
        if (index >= 0)
            cb->setCurrentIndex(index);
    }
    else if (param->type == "bool")
    {
        QCheckBox *cb = reinterpret_cast<QCheckBox*>(param->widget);
        cb->setChecked(boost::any_cast<bool>(param->getValue()));
    }
    else if (param->type == "int")
    {
        QSlider *slider = param->slider;
        QSpinBox *spinBox = reinterpret_cast<QSpinBox*>(param->widget);
        intSlider_setValue(slider, param, boost::any_cast<int>(param->getValue()));
        spinBox->setValue(boost::any_cast<int>(param->getValue()));
    }
    else if (param->type == "double")
    {
        QSlider *slider = param->slider;
        QDoubleSpinBox *spinBox = reinterpret_cast<QDoubleSpinBox*>(param->widget);
        doubleSlider_setValue(slider, param, boost::any_cast<double>(param->getValue()));
        spinBox->setValue(boost::any_cast<double>(param->getValue()));
    }
    else if (param->type == "str")
    {
        QLineEdit *le = reinterpret_cast<QLineEdit*>(param->widget);
        le->setText(QString::fromStdString(boost::any_cast<std::string>(param->getValue())));
    }
    else
    {
        ROS_WARN_STREAM("Type " << param->type << " of parameter " << param->name << " not implemented");
    }
  }
  ui.moveArmComboBox->setEnabled(rvp_current_config.mode < 2);
  ui.moveArmPushButton->setEnabled(rvp_current_config.mode < 2);
}

void RoiViewpointPlannerRqtPlugin::vmpConfigChanged(const view_motion_planner::VmpConfig &received_config)
{
  // ROS_INFO_STREAM("Config changed slot is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  vmp_current_config = received_config;
  for (const AbstractParamPtr &param : vmp_params)
  {
    if (param->edit_method != "") // param is enum
    {
        QComboBox *cb = reinterpret_cast<QComboBox*>(param->widget);
        QVariant val;
        if (param->type == "bool")
            val = QVariant(boost::any_cast<bool>(param->getValue()));
        else if (param->type == "int")
            val = QVariant(boost::any_cast<int>(param->getValue()));
        else if (param->type == "double")
            val = QVariant(boost::any_cast<double>(param->getValue()));
        else if (param->type == "str")
            val = QVariant(QString::fromStdString(boost::any_cast<std::string>(param->getValue())));

        int index = cb->findData(val);
        if (index >= 0)
            cb->setCurrentIndex(index);
    }
    else if (param->type == "bool")
    {
        QCheckBox *cb = reinterpret_cast<QCheckBox*>(param->widget);
        cb->setChecked(boost::any_cast<bool>(param->getValue()));
    }
    else if (param->type == "int")
    {
        QSlider *slider = param->slider;
        QSpinBox *spinBox = reinterpret_cast<QSpinBox*>(param->widget);
        intSlider_setValue(slider, param, boost::any_cast<int>(param->getValue()));
        spinBox->setValue(boost::any_cast<int>(param->getValue()));
    }
    else if (param->type == "double")
    {
        QSlider *slider = param->slider;
        QDoubleSpinBox *spinBox = reinterpret_cast<QDoubleSpinBox*>(param->widget);
        doubleSlider_setValue(slider, param, boost::any_cast<double>(param->getValue()));
        spinBox->setValue(boost::any_cast<double>(param->getValue()));
    }
    else if (param->type == "str")
    {
        QLineEdit *le = reinterpret_cast<QLineEdit*>(param->widget);
        le->setText(QString::fromStdString(boost::any_cast<std::string>(param->getValue())));
    }
    else
    {
        ROS_WARN_STREAM("Type " << param->type << " of parameter " << param->name << " not implemented");
    }
  }
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
  ui.scanLed->setState(state->scan_inserted);
}

void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc)
{
  ROS_INFO_STREAM("Description callback called");
}

// DO NOT DIRECTLY UPDATE UI ELEMENTS HERE
void RoiViewpointPlannerRqtPlugin::rvpConfigCallback(const roi_viewpoint_planner::PlannerConfig &conf)
{
  //ROS_INFO_STREAM("Config callback is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  ROS_INFO_STREAM("RVP config callback called");
  emit rvpConfigChangedSignal(conf);
}

// DO NOT DIRECTLY UPDATE UI ELEMENTS HERE
void RoiViewpointPlannerRqtPlugin::vmpConfigCallback(const view_motion_planner::VmpConfig &conf)
{
  //ROS_INFO_STREAM("Config callback is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  ROS_INFO_STREAM("VMP config callback called");
  emit vmpConfigChangedSignal(conf);
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
  QString file_path = QFileDialog::getOpenFileName(widget, QString(), QString(), "Octree (*.ot *.bt)");
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

void RoiViewpointPlannerRqtPlugin::on_moveArmPushButton_clicked()
{
  if (moveToStateThread->isRunning())
  {
    ui.statusTextBox->setText("Arm is already moving");
    return;
  }
  int index = ui.moveArmComboBox->currentIndex();
  if (index < 0 || index >= MOVE_CONFIGS.size())
  {
    ui.statusTextBox->setText("Invalid move config");
    return;
  }
  if (!moveToStateThread->setGoalState(MOVE_CONFIGS[index], ui.moveArmComboBox->currentText()))
  {
    ui.statusTextBox->setText("Couldn't set move config");
    return;
  }
  moveToStateThread->start();
}

void RoiViewpointPlannerRqtPlugin::on_randomizePlantsPushButton_clicked()
{
  roi_viewpoint_planner_msgs::RandomizePlantPositions srv;
  srv.request.min_point.x = ui.randMinXSpinBox->value();
  srv.request.min_point.y = ui.randMinYSpinBox->value();
  srv.request.min_point.z = ui.randMinZSpinBox->value();
  srv.request.max_point.x = ui.randMaxXSpinBox->value();
  srv.request.max_point.y = ui.randMaxYSpinBox->value();
  srv.request.max_point.z = ui.randMaxZSpinBox->value();
  srv.request.min_dist = ui.randMinDistSpinBox->value();
  if (randomizePlantPositionsClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Plant positions randomized");
    else
      ui.statusTextBox->setText("Couldn't randomize plant positions");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call start randomize plant positions service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_startEvaluatorPushButton_clicked()
{
  roi_viewpoint_planner_msgs::StartEvaluator srv;
  srv.request.num_evals = ui.evalTrialsSpinBox->value();
  srv.request.episode_end_param = static_cast<uint8_t>(ui.evalEndParamComboBox->currentIndex());
  srv.request.episode_duration = ui.evalDurationSpinBox->value();
  srv.request.starting_index = ui.startingIndexSpinBox->value();
  srv.request.randomize_plants = ui.evalRandomizeCheckBox->isChecked();
  srv.request.min_point.x = ui.randMinXSpinBox->value();
  srv.request.min_point.y = ui.randMinYSpinBox->value();
  srv.request.min_point.z = ui.randMinZSpinBox->value();
  srv.request.max_point.x = ui.randMaxXSpinBox->value();
  srv.request.max_point.y = ui.randMaxYSpinBox->value();
  srv.request.max_point.z = ui.randMaxZSpinBox->value();
  srv.request.min_dist = ui.randMinDistSpinBox->value();
  if (startEvaluatorClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Evaluator started");
    else
      ui.statusTextBox->setText("Couldn't start evaluator");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call start evaluator service");
  }
}

constexpr std::array<std::array<double, 6>, 5> RoiViewpointPlannerRqtPlugin::MOVE_CONFIGS;

} // namespace roi_viewpoint_planner_rqt_plugin

PLUGINLIB_EXPORT_CLASS(rqt_roi_viewpoint_planner::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(roi_viewpoint_planner_rqt_plugin, RoiViewpointPlannerRqtPlugin, roi_viewpoint_planner_rqt_plugin::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
