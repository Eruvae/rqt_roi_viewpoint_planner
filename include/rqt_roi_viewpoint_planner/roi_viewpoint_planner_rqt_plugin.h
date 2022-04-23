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


class AbstractParam
{
public:
  const std::string name;
  const std::string type;
  const uint32_t level;
  const std::string description;
  const std::string edit_method;

  QWidget *widget = nullptr;
  QSlider *slider = nullptr;

  AbstractParam(const std::string name, const std::string type, const uint32_t level, const std::string description, const std::string edit_method)
    : name(name),
      type(type),
      level(level),
      description(description),
      edit_method(edit_method)
  {}

  virtual ~AbstractParam() {}

  virtual boost::any getDefault() const = 0;
  virtual boost::any getMin() const = 0;
  virtual boost::any getMax() const = 0;
  virtual boost::any getValue() const = 0;
  virtual void setValue(const boost::any &val) = 0;
};

using AbstractParamPtr = boost::shared_ptr<AbstractParam>;

template<typename C>
class AbstractConfigParam : public AbstractParam
{
private:
  boost::any def;
  boost::any min;
  boost::any max;

public:
  AbstractConfigParam(const typename C::AbstractParamDescriptionConstPtr &p)
    : AbstractParam (p->name, p->type, p->level, p->description, p->edit_method)
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
  Param(const typename C::AbstractParamDescriptionConstPtr &p, C &config)
    : AbstractConfigParam<C>(p),
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

template<typename C>
AbstractParamPtr initializeParam(const typename C::AbstractParamDescriptionConstPtr &p, C &config)
{
  if (p->type == "bool")
  {
    return boost::make_shared<Param<C, bool>>(p, config);
  }
  else if (p->type == "int")
  {
    return boost::make_shared<Param<C, int>>(p, config);
  }
  else if (p->type == "double")
  {
    return boost::make_shared<Param<C, double>>(p, config);
  }
  else if (p->type == "str")
  {
    return boost::make_shared<Param<C, std::string>>(p, config);
  }
  else
  {
    ROS_WARN_STREAM("Type " << p->type << " of parameter " << p->name << " not implemented");
    return nullptr;
  }
}

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

  void on_boolComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index);
  void on_intComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index);
  void on_doubleComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index);
  void on_strComboBox_activated(QComboBox *comboBox, const AbstractParamPtr &param, int index);
  void on_checkBox_clicked(const AbstractParamPtr &param, bool checked);
  void on_lineEdit_textEdited(const AbstractParamPtr &param, const QString &text);
  void on_intSlider_sliderMoved(QSpinBox *spinBox, const AbstractParamPtr &param, int position);
  void on_intSlider_sliderReleased(QSpinBox *spinBox, const AbstractParamPtr &param);
  void on_intSpinBox_editingFinished(QSpinBox *spinBox, QSlider *slider, const AbstractParamPtr &param);
  void on_doubleSlider_sliderMoved(QDoubleSpinBox *spinBox, const AbstractParamPtr &param, int position);
  void on_doubleSlider_sliderReleased(QDoubleSpinBox *spinBox, const AbstractParamPtr &param);
  void on_doubleSpinBox_editingFinished(QDoubleSpinBox *spinBox, QSlider *slider, const AbstractParamPtr &param);

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

  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> *rvpConfigClient;
  dynamic_reconfigure::Client<view_motion_planner::VmpConfig> *vmpConfigClient;

  std::vector<AbstractParamPtr> rvp_params;
  std::vector<AbstractParamPtr> vmp_params;

  void initConfigGui();

  void initEnumParam(const AbstractParamPtr &param, QFormLayout *configLayout);
  void initBoolParam(const AbstractParamPtr &param, QFormLayout *configLayout);
  void initIntParam(const AbstractParamPtr &param, QFormLayout *configLayout);
  void initDoubleParam(const AbstractParamPtr &param, QFormLayout *configLayout);
  void initStringParam(const AbstractParamPtr &param, QFormLayout *configLayout);

  void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc);
  void rvpConfigCallback(const roi_viewpoint_planner::PlannerConfig &conf);
  void vmpConfigCallback(const view_motion_planner::VmpConfig &conf);
  bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

  void intSlider_setValue(QSlider *slider, const AbstractParamPtr &param, int value);
  void intSpinBox_setPosition(QSpinBox *spinBox, const AbstractParamPtr &param, int position);
  void intValue_sendConfig(QSpinBox *spinBox, const AbstractParamPtr &param);

  void doubleSlider_setValue(QSlider *slider, const AbstractParamPtr &param, double value);
  void doubleSpinBox_setPosition(QDoubleSpinBox *spinBox, const AbstractParamPtr &param, int position);
  void doubleValue_sendConfig(QDoubleSpinBox *spinBox, const AbstractParamPtr &param);

  void sendConfig(const AbstractParamPtr &changed_param);
};

}

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
