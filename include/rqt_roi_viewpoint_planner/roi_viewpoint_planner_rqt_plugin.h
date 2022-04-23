#ifndef ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
#define ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H

#include <QWidget>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLineEdit>
#include <QThread>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner_msgs/PlannerConfig.h>
#include <roi_viewpoint_planner_msgs/EvaluatorConfig.h>
#include <roi_viewpoint_planner_msgs/PlannerState.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <roi_viewpoint_planner_msgs/LoadOctomap.h>
#include <roi_viewpoint_planner_msgs/MoveToState.h>
#include <roi_viewpoint_planner_msgs/RandomizePlantPositions.h>
#include <roi_viewpoint_planner_msgs/StartEvaluator.h>
#include <roi_viewpoint_planner_msgs/VmpConfig.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <boost/unordered_map.hpp>
#include "ui_roi_viewpoint_planner_rqt_plugin.h"
#include <yaml-cpp/yaml.h>

namespace rqt_roi_viewpoint_planner
{

class AbstractParam;
using AbstractParamPtr = boost::shared_ptr<AbstractParam>;

class AbstractReconfigureClient : public QObject
{
  Q_OBJECT
protected:
  std::vector<AbstractParamPtr> params;
  virtual void changedConfig();

signals:
  void configChanged();

public:
  virtual ~AbstractReconfigureClient() {}

  virtual void sendConfig(const AbstractParam *changed_param) = 0;
};

class AbstractParam : public QObject
{
  Q_OBJECT

protected:
  AbstractReconfigureClient *client;

protected slots:
  virtual void on_comboBox_activated(int index) {}
  virtual void on_checkBox_clicked(bool checked) {}
  virtual void on_slider_sliderMoved(int position) {}
  virtual void on_slider_sliderReleased() {}
  virtual void on_spinBox_editingFinished() {}
  virtual void on_lineEdit_textEdited(const QString &text) {}

public:
  const std::string name;
  const std::string type;
  const uint32_t level;
  const std::string description;
  const std::string edit_method;

  AbstractParam(const std::string name, const std::string type, const uint32_t level, const std::string description, const std::string edit_method,
                AbstractReconfigureClient *client)
    : name(name),
      type(type),
      level(level),
      description(description),
      edit_method(edit_method),
      client(client)
  {}

  virtual ~AbstractParam() {}

  virtual boost::any getDefault() const = 0;
  virtual boost::any getMin() const = 0;
  virtual boost::any getMax() const = 0;
  virtual boost::any getValue() const = 0;
  virtual void setValue(const boost::any &val) = 0;
  virtual void updateGuiValue() = 0;
};

template<typename C>
class AbstractConfigParam : public AbstractParam
{
protected:
  boost::any def;
  boost::any min;
  boost::any max;


public:
  AbstractConfigParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client)
    : AbstractParam (p->name, p->type, p->level, p->description, p->edit_method, client)
  {
    p->getValue(C::__getDefault__(), def);
    p->getValue(C::__getMin__(), min);
    p->getValue(C::__getMax__(), max);
  }

  virtual ~AbstractConfigParam() {}

  virtual boost::any getDefault() const override
  {
    return def;
  }

  virtual boost::any getMin() const override
  {
    return min;
  }

  virtual boost::any getMax() const override
  {
    return max;
  }

  virtual boost::any getValue() const = 0;
  virtual void setValue(const boost::any &val) = 0;
};

template<typename C, typename T>
class Param : public AbstractConfigParam<C>
{
private:
  using ParamDescription = typename C::template ParamDescription<T>;
  const ParamDescription *pd;
  T &field;

public:
  Param(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config)
    : AbstractConfigParam<C>(p, client),
      pd(reinterpret_cast<const ParamDescription*>(p.get())),
      field(config.*(pd->field))
  {}

  virtual ~Param() {}

  virtual boost::any getValue() const override
  {
    return field;
  }

  virtual void setValue(const boost::any &val) override
  {
    field = boost::any_cast<T>(val);
  }
};

template<typename T>
QVariant toQVariant(const T &val)
{
  return QVariant(val);
}

template<typename T>
T fromQVariant(const QVariant &val)
{
  return val.value<T>();
}

template<typename C, typename T>
class EnumParam : public Param<C, T>
{
private:
  QComboBox *comboBox = nullptr;

protected:
  virtual void on_comboBox_activated(int index) override
  {
    QVariant val = comboBox->itemData(index);
    this->setValue(fromQVariant<T>(val));
    this->client->sendConfig(this);
  }

public:
  EnumParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, T>(p, client, config), comboBox(new QComboBox())
  {
    YAML::Node enum_description = YAML::Load(p->edit_method);
    for (const YAML::Node &node : enum_description["enum"])
    {
        QString entry_name = QString::fromStdString(node["name"].as<std::string>());
        QVariant userdata = toQVariant<T>(node["value"].as<T>());
        entry_name += QString(" (") + userdata.toString() + QString(")");
        comboBox->addItem(entry_name, userdata);
    }

    QVariant default_val;
    default_val = toQVariant<T>(boost::any_cast<T>(this->getDefault()));
    QObject::connect(comboBox, QOverload<int>::of(&QComboBox::activated), this, boost::bind(&EnumParam<C, T>::on_comboBox_activated, this, _1));

    int default_index = comboBox->findData(default_val);
    if (default_index >= 0)
        comboBox->setCurrentIndex(default_index);

    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), comboBox);
  }

  virtual void updateGuiValue()
  {
    QVariant val = toQVariant<T>(boost::any_cast<T>(this->getValue()));
    int index = comboBox->findData(val);
    if (index >= 0)
        comboBox->setCurrentIndex(index);
  }
};

template<typename C>
class BoolParam : public Param<C, bool>
{
private:
  QCheckBox *checkBox = nullptr;

protected:
  virtual void on_checkBox_clicked(bool checked) override
  {
    this->setValue(checked);
    this->client->sendConfig(this);
  }

public:

  BoolParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, bool>(p, client, config), checkBox(new QCheckBox())
  {
    checkBox->setChecked(boost::any_cast<bool>(this->getDefault()));
    QObject::connect(checkBox, &QCheckBox::clicked, this, boost::bind(&BoolParam<C>::on_checkBox_clicked, this, _1));
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), checkBox);
  }

  virtual void updateGuiValue()
  {
    checkBox->setChecked(boost::any_cast<bool>(this->getValue()));
  }

};

template<typename C>
class IntParam : public Param<C, int>
{
private:
  QSpinBox *spinBox = nullptr;
  QSlider *slider = nullptr;

  void intSlider_setValue(int value)
  {
    int minVal = boost::any_cast<int>(this->getMin());
    int maxVal = boost::any_cast<int>(this->getMax());
    int position = qBound(0, static_cast<int>(static_cast<double>(value - minVal) / static_cast<double>(maxVal - minVal) * 100.0), 100);
    slider->setValue(position);
  }

  void intSpinBox_setPosition(int position)
  {
    int minVal = boost::any_cast<int>(this->getMin());
    int maxVal = boost::any_cast<int>(this->getMax());
    spinBox->setValue(minVal + position * (maxVal - minVal) / 100);
  }

  void intValue_sendConfig()
  {
    this->setValue(spinBox->value());
    this->client->sendConfig(this);
  }

protected:
  virtual void on_slider_sliderMoved(int position) override
  {
    intSpinBox_setPosition(position);
  }

  virtual void on_slider_sliderReleased() override
  {
    intValue_sendConfig();
  }

  virtual void on_spinBox_editingFinished() override
  {
    intSlider_setValue(spinBox->value());
    intValue_sendConfig();
  }

public:
  IntParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, int>(p, client, config), spinBox(new QSpinBox()), slider(new QSlider(Qt::Orientation::Horizontal))
  {
    int min = boost::any_cast<int>(this->getMin());
    int max = boost::any_cast<int>(this->getMax());
    int def = boost::any_cast<int>(this->getDefault());
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *minLabel = new QLabel(QString::number(min));
    QLabel *maxLabel = new QLabel(QString::number(max));
    slider->setMaximum(100);
    intSlider_setValue(def);
    spinBox->setMinimum(min);
    spinBox->setMaximum(max);
    spinBox->setValue(def);
    layout->addWidget(minLabel);
    layout->addWidget(slider);
    layout->addWidget(maxLabel);
    layout->addWidget(spinBox);
    QObject::connect(slider, &QSlider::sliderMoved, this, boost::bind(&IntParam<C>::on_slider_sliderMoved, this, _1));
    QObject::connect(slider, &QSlider::sliderReleased, this, boost::bind(&IntParam<C>::on_slider_sliderReleased, this));
    QObject::connect(spinBox, &QSpinBox::editingFinished, this, boost::bind(&IntParam<C>::on_spinBox_editingFinished, this));
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), layout);
  }

  virtual void updateGuiValue()
  {
    intSlider_setValue(boost::any_cast<int>(this->getValue()));
    spinBox->setValue(boost::any_cast<int>(this->getValue()));
  }

};

template <typename C>
class DoubleParam : public Param<C, double>
{
private:
  QDoubleSpinBox *spinBox = nullptr;
  QSlider *slider = nullptr;

  void doubleSlider_setValue(double value)
  {
    double minVal = boost::any_cast<double>(this->getMin());
    double maxVal = boost::any_cast<double>(this->getMax());
    int position = qBound(0, static_cast<int>((value - minVal) / (maxVal - minVal) * 100.0), 100);
    slider->setValue(position);
  }

  void doubleSpinBox_setPosition(int position)
  {
    double minVal = boost::any_cast<double>(this->getMin());
    double maxVal = boost::any_cast<double>(this->getMax());
    spinBox->setValue(minVal + static_cast<double>(position) / 100.0 * (maxVal - minVal));
  }

  void doubleValue_sendConfig()
  {
    this->setValue(spinBox->value());
    this->client->sendConfig(this);
  }

protected:
  virtual void on_slider_sliderMoved(int position) override
  {
    doubleSpinBox_setPosition(position);
  }

  virtual void on_slider_sliderReleased() override
  {
    doubleValue_sendConfig();
  }

  virtual void on_spinBox_editingFinished() override
  {
    doubleSlider_setValue(spinBox->value());
    doubleValue_sendConfig();
  }

public:
  DoubleParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, double>(p, client, config), spinBox(new QDoubleSpinBox()), slider(new QSlider(Qt::Orientation::Horizontal))
  {
    double min = boost::any_cast<double>(this->getMin());
    double max = boost::any_cast<double>(this->getMax());
    double def = boost::any_cast<double>(this->getDefault());
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *minLabel = new QLabel(QString::number(min));
    QLabel *maxLabel = new QLabel(QString::number(max));
    slider->setMaximum(100);
    doubleSlider_setValue(def);
    spinBox->setMinimum(min);
    spinBox->setMaximum(max);
    int exponent = QString::number(max-min, 'e', 0).split('e').last().toInt();
    spinBox->setSingleStep(std::pow(10.0, exponent-1));
    spinBox->setValue(def);
    layout->addWidget(minLabel);
    layout->addWidget(slider);
    layout->addWidget(maxLabel);
    layout->addWidget(spinBox);
    QObject::connect(slider, &QSlider::sliderMoved, this, boost::bind(&DoubleParam<C>::on_slider_sliderMoved, this, _1));
    QObject::connect(slider, &QSlider::sliderReleased, this, boost::bind(&DoubleParam<C>::on_slider_sliderReleased, this));
    QObject::connect(spinBox, &QDoubleSpinBox::editingFinished, this, boost::bind(&DoubleParam<C>::on_spinBox_editingFinished, this));
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), layout);
  }

  virtual void updateGuiValue()
  {
    doubleSlider_setValue(boost::any_cast<double>(this->getValue()));
    spinBox->setValue(boost::any_cast<double>(this->getValue()));
  }
};

template <typename C>
class StringParam : public Param<C, std::string>
{
private:
  QLineEdit *lineEdit = nullptr;

protected:
  virtual void on_lineEdit_textEdited(const QString &text) override
  {
    this->setValue(text.toStdString());
    this->client->sendConfig(this);
  }

public:
  StringParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, std::string>(p, client, config), lineEdit(new QLineEdit())
  {
    lineEdit->setText(QString::fromStdString(boost::any_cast<std::string>(this->getDefault())));
    QObject::connect(lineEdit, &QLineEdit::textEdited, this, boost::bind(&StringParam<C>::on_lineEdit_textEdited, this, _1));
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), lineEdit);
  }

  virtual void updateGuiValue()
  {
    lineEdit->setText(QString::fromStdString(boost::any_cast<std::string>(this->getValue())));
  }

};

template<typename C>
AbstractParamPtr initializeParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
{
  bool is_enum = p->edit_method != "";
  if (p->type == "bool")
  {
    if (is_enum)
      return boost::make_shared<EnumParam<C, bool>>(p, client, config, configLayout);
    else
      return boost::make_shared<BoolParam<C>>(p, client, config, configLayout);
  }
  else if (p->type == "int")
  {
    if (is_enum)
      return boost::make_shared<EnumParam<C, int>>(p, client, config, configLayout);
    else
      return boost::make_shared<IntParam<C>>(p, client, config, configLayout);
  }
  else if (p->type == "double")
  {
    if (is_enum)
      return boost::make_shared<EnumParam<C, double>>(p, client, config, configLayout);
    else
      return boost::make_shared<DoubleParam<C>>(p, client, config, configLayout);
  }
  else if (p->type == "str")
  {
    if (is_enum)
      return boost::make_shared<EnumParam<C, std::string>>(p, client, config, configLayout);
    else
      return boost::make_shared<StringParam<C>>(p, client, config, configLayout);
  }
  else
  {
    ROS_WARN_STREAM("Type " << p->type << " of parameter " << p->name << " not implemented");
    return nullptr;
  }
}

template<typename C>
class ReconfigureClient : public AbstractReconfigureClient
{
private:
  C current_config;
  dynamic_reconfigure::Client<C> *config_client;
  QLineEdit *statusTextBox;

  void configCallback(const C &conf)
  {
    ROS_INFO_STREAM("Config callback called");
    current_config = conf;
    emit configChanged();
  }

public:
  ReconfigureClient(const std::string& name, QFormLayout *configLayout, QLineEdit *statusTextBox)
    : config_client(new dynamic_reconfigure::Client<C>(name, boost::bind(&ReconfigureClient::configCallback, this, _1))),
      statusTextBox(statusTextBox)
  {
    QObject::connect(this, &ReconfigureClient::configChanged, this, &ReconfigureClient::changedConfig);

    for (const typename C::AbstractParamDescriptionConstPtr &param : C::__getParamDescriptions__())
    {
      AbstractParamPtr p = initializeParam(param, this, current_config, configLayout);
      params.push_back(p);
    }
  }

  virtual void sendConfig(const AbstractParam *changed_param)
  {
    if (!config_client->setConfiguration(current_config))
      statusTextBox->setText(QString::fromStdString(changed_param->name) + " change failed");
    else
      statusTextBox->setText(QString::fromStdString(changed_param->name) + " change successful");
  }

};

class MoveToStateThread : public QThread {
  Q_OBJECT
public:
  MoveToStateThread(ros::NodeHandle &nh) : QThread(),
    moveToStateClient(nh.serviceClient<roi_viewpoint_planner_msgs::MoveToState>("/roi_viewpoint_planner/move_to_state")),
    name("pose")
  {
  }

  bool setGoalState(const std::array<double, 6> &state, const QString &name = "pose")
  {
    if (isRunning())
      return false;

    this->name = name;
    srv.request.joint_values.resize(state.size());
    std::copy(state.begin(), state.end(), srv.request.joint_values.begin());
    return true;
  }

  void run() override
  {
    if (!moveToStateClient.call(srv))
    {
      emit error("Failed to call move to state service");
      return;
    }
    if (!srv.response.success)
    {
      emit error("Couldn't plan to specified position");
      return;
    }
    emit success("Moving to " + name + " successful");
  }


signals:
  void success(QString msg);
  void error(QString msg);

private:
  ros::ServiceClient moveToStateClient;
  QString name;
  roi_viewpoint_planner_msgs::MoveToState srv;
};

class RoiViewpointPlannerRqtPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  RoiViewpointPlannerRqtPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:
  // UI slots
  void on_saveMapPushButton_clicked();
  void on_loadMapPushButton_clicked();
  void on_resetMapPushButton_clicked();
  void on_moveArmPushButton_clicked();

  // Internal slots
  void rvpConfigChanged(const roi_viewpoint_planner::PlannerConfig &received_config);
  void vmpConfigChanged(const view_motion_planner::VmpConfig &received_config);
  void planRequest(bool enable);
  void plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

  void on_randomizePlantsPushButton_clicked();
  void on_startEvaluatorPushButton_clicked();

signals:
  // Internal signals
  void rvpConfigChangedSignal(const roi_viewpoint_planner::PlannerConfig &received_config);
  void vmpConfigChangedSignal(const view_motion_planner::VmpConfig &received_config);
  void planRequestSignal(bool enable);
  void plannerStateSignal(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

private:

  static constexpr std::array<std::array<double, 6>, 5> MOVE_CONFIGS =
  {{
    {0, -0.7854, 1.5708, -0.7854, 0, 0}, // HOME
    {0, 0, 0, 0, -1.5707, 0}, // TRANSPORT
    {0, -1.5707, 0, -1.5707, 0, -3.1415}, // UP
    {0, -3.1415, 0, -1.5707, 0, -1.5707}, // LEFT
    {0, 0, 0, 0, 0, 0} // ZERO
  }};

  Ui::RoiViewpointPlannerRqtPlugin ui;
  QWidget* widget;
  std::unique_ptr<MoveToStateThread> moveToStateThread;
  roi_viewpoint_planner::PlannerConfig rvp_current_config;
  view_motion_planner::VmpConfig vmp_current_config;
  ros::ServiceClient saveOctomapClient;
  ros::ServiceClient loadOctomapClient;
  ros::ServiceClient resetOctomapClient;
  ros::ServiceClient randomizePlantPositionsClient;
  ros::ServiceClient startEvaluatorClient;

  ros::Subscriber plannerStateSub;

  ros::ServiceServer confirmPlanExecutionServer;

  ReconfigureClient<roi_viewpoint_planner::PlannerConfig> *rvpConfigClient;
  ReconfigureClient<view_motion_planner::VmpConfig> *vmpConfigClient;

  void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc);
  bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);
};

}

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
