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

struct GenericParamDescription
{
  std::variant<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr, view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr> param;

  const std::string &name;
  const std::string &type;
  const uint32_t &level;
  const std::string &description;
  const std::string &edit_method;

  GenericParamDescription(const roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr &p)
    : param(p),
      name(p->name),
      type(p->type),
      level(p->level),
      description(p->description),
      edit_method(p->edit_method)
  {}

  GenericParamDescription(const view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr &p)
    : param(p),
      name(p->name),
      type(p->type),
      level(p->level),
      description(p->description),
      edit_method(p->edit_method)
  {}
};

using GenericParamDescriptionConstPtr = boost::shared_ptr<const GenericParamDescription>;

using RvpParamMap = boost::unordered_map<std::string, roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>;
using VmpParamMap = boost::unordered_map<std::string, view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>;

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

  void on_boolComboBox_activated(QComboBox *comboBox, const GenericParamDescriptionConstPtr &param, int index);
  void on_intComboBox_activated(QComboBox *comboBox, const GenericParamDescriptionConstPtr &param, int index);
  void on_doubleComboBox_activated(QComboBox *comboBox, const GenericParamDescriptionConstPtr &param, int index);
  void on_strComboBox_activated(QComboBox *comboBox, const GenericParamDescriptionConstPtr &param, int index);
  void on_checkBox_clicked(const GenericParamDescriptionConstPtr &param, bool checked);
  void on_lineEdit_textEdited(const GenericParamDescriptionConstPtr &param, const QString &text);
  void on_intSlider_sliderMoved(QSpinBox *spinBox, const GenericParamDescriptionConstPtr &param, int position);
  void on_intSlider_sliderReleased(QSpinBox *spinBox, const GenericParamDescriptionConstPtr &param);
  void on_intSpinBox_editingFinished(QSpinBox *spinBox, QSlider *slider, const GenericParamDescriptionConstPtr &param);
  void on_doubleSlider_sliderMoved(QDoubleSpinBox *spinBox, const GenericParamDescriptionConstPtr &param, int position);
  void on_doubleSlider_sliderReleased(QDoubleSpinBox *spinBox, const GenericParamDescriptionConstPtr &param);
  void on_doubleSpinBox_editingFinished(QDoubleSpinBox *spinBox, QSlider *slider, const GenericParamDescriptionConstPtr &param);

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

  struct ParamWidget
  {
    QWidget *widget = nullptr;
    QSlider *slider = nullptr;
  };

  using ParamWidgetMap = boost::unordered_map<GenericParamDescriptionConstPtr, ParamWidget>;

  std::vector<GenericParamDescriptionConstPtr> rvp_params;
  std::vector<GenericParamDescriptionConstPtr> vmp_params;
  ParamWidgetMap param_widgets;

  void initConfigGui();

  void initEnumParam(const GenericParamDescriptionConstPtr &param, const boost::any &val, QFormLayout *configLayout);
  void initBoolParam(const GenericParamDescriptionConstPtr &param, bool val, QFormLayout *configLayout);
  void initIntParam(const GenericParamDescriptionConstPtr &param, int val, int min, int max, QFormLayout *configLayout);
  void initDoubleParam(const GenericParamDescriptionConstPtr &param, double val, double min, double max, QFormLayout *configLayout);
  void initStringParam(const GenericParamDescriptionConstPtr &param, const std::string &val, QFormLayout *configLayout);

  void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc);
  void rvpConfigCallback(const roi_viewpoint_planner::PlannerConfig &conf);
  void vmpConfigCallback(const view_motion_planner::VmpConfig &conf);
  bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

  void intSlider_setValue(QSlider *slider, const GenericParamDescriptionConstPtr &param, int value);
  void intSpinBox_setPosition(QSpinBox *spinBox, const GenericParamDescriptionConstPtr &param, int position);
  void intValue_sendConfig(QSpinBox *spinBox, const GenericParamDescriptionConstPtr &param);

  void doubleSlider_setValue(QSlider *slider, const GenericParamDescriptionConstPtr &param, double value);
  void doubleSpinBox_setPosition(QDoubleSpinBox *spinBox, const GenericParamDescriptionConstPtr &param, int position);
  void doubleValue_sendConfig(QDoubleSpinBox *spinBox, const GenericParamDescriptionConstPtr &param);

  void sendConfig(const GenericParamDescriptionConstPtr &changed_param);

  template <typename T>
  T getValue(const GenericParamDescriptionConstPtr &param)
  {
    if (std::holds_alternative<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(rvp_current_config, val);
      return boost::any_cast<T>(val);
    }
    if (std::holds_alternative<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(vmp_current_config, val);
      return boost::any_cast<T>(val);
    }
    return T();
  }

  template <typename T>
  T getMin(const GenericParamDescriptionConstPtr &param)
  {
    if (std::holds_alternative<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(roi_viewpoint_planner::PlannerConfig::__getMin__(), val);
      return boost::any_cast<T>(val);
    }
    if (std::holds_alternative<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(view_motion_planner::VmpConfig::__getMin__(), val);
      return boost::any_cast<T>(val);
    }
    return T();
  }

  template <typename T>
  T getMax(const GenericParamDescriptionConstPtr &param)
  {
    if (std::holds_alternative<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(roi_viewpoint_planner::PlannerConfig::__getMax__(), val);
      return boost::any_cast<T>(val);
    }
    if (std::holds_alternative<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(view_motion_planner::VmpConfig::__getMax__(), val);
      return boost::any_cast<T>(val);
    }
    return T();
  }

  template <typename T>
  T getDefault(const GenericParamDescriptionConstPtr &param)
  {
    if (std::holds_alternative<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(roi_viewpoint_planner::PlannerConfig::__getDefault__(), val);
      return boost::any_cast<T>(val);
    }
    if (std::holds_alternative<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      boost::any val;
      std::get<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param)->getValue(view_motion_planner::VmpConfig::__getDefault__(), val);
      return boost::any_cast<T>(val);
    }
    return T();
  }

  template <typename T>
  void setValue(const GenericParamDescriptionConstPtr &param, T val)
  {
    if (std::holds_alternative<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      const roi_viewpoint_planner::PlannerConfig::ParamDescription<T> *pd = reinterpret_cast<const roi_viewpoint_planner::PlannerConfig::ParamDescription<T>*>(std::get<roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr>(param->param).get());
      rvp_current_config.*(pd->field) = val;
      return;
    }
    if (std::holds_alternative<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param))
    {
      const view_motion_planner::VmpConfig::ParamDescription<T> *pd = reinterpret_cast<const view_motion_planner::VmpConfig::ParamDescription<T>*>(std::get<view_motion_planner::VmpConfig::AbstractParamDescriptionConstPtr>(param->param).get());
      vmp_current_config.*(pd->field) = val;
      return;
    }
  }
};

}

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
