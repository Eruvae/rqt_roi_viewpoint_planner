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
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <unordered_map>
#include "ui_roi_viewpoint_planner_rqt_plugin.h"

namespace rqt_roi_viewpoint_planner
{

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

  void on_boolComboBox_activated(QComboBox *comboBox, const std::string &param, int index);
  void on_intComboBox_activated(QComboBox *comboBox, const std::string &param, int index);
  void on_doubleComboBox_activated(QComboBox *comboBox, const std::string &param, int index);
  void on_strComboBox_activated(QComboBox *comboBox, const std::string &param, int index);
  void on_checkBox_clicked(const std::string &param, bool checked);
  void on_lineEdit_textEdited(const std::string &param, const QString &text);
  void on_intSlider_sliderMoved(QSpinBox *spinBox, const std::string &param, int position);
  void on_intSlider_sliderReleased(QSpinBox *spinBox, const std::string &param);
  void on_intSpinBox_editingFinished(QSpinBox *spinBox, QSlider *slider, const std::string &param);
  void on_doubleSlider_sliderMoved(QDoubleSpinBox *spinBox, const std::string &param, int position);
  void on_doubleSlider_sliderReleased(QDoubleSpinBox *spinBox, const std::string &param);
  void on_doubleSpinBox_editingFinished(QDoubleSpinBox *spinBox, QSlider *slider, const std::string &param);

  // Internal slots
  void configChanged(const roi_viewpoint_planner::PlannerConfig &received_config);
  void planRequest(bool enable);
  void plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

  void on_randomizePlantsPushButton_clicked();
  void on_startEvaluatorPushButton_clicked();

signals:
  // Internal signals
  void configChangedSignal(const roi_viewpoint_planner::PlannerConfig &received_config);
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
  roi_viewpoint_planner::PlannerConfig current_config;
  ros::ServiceClient saveOctomapClient;
  ros::ServiceClient loadOctomapClient;
  ros::ServiceClient resetOctomapClient;
  ros::ServiceClient randomizePlantPositionsClient;
  ros::ServiceClient startEvaluatorClient;

  ros::Subscriber plannerStateSub;

  ros::ServiceServer confirmPlanExecutionServer;

  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> *configClient;

  std::unordered_map<std::string, roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr> parameter_map;
  std::unordered_map<std::string, QComboBox*> comboBox_map;
  std::unordered_map<std::string, QCheckBox*> checkBox_map;
  std::unordered_map<std::string, QSlider*> slider_map;
  std::unordered_map<std::string, QAbstractSpinBox*> spinBox_map;
  std::unordered_map<std::string, QLineEdit*> lineEdit_map;

  void initConfigGui();
  void initEnumParam(const std::string &name, const std::string &enum_description_str, const std::string &type, const boost::any &val);
  void initBoolParam(const std::string &name, bool val);
  void initIntParam(const std::string &name, int val, int min, int max);
  void initDoubleParam(const std::string &name, double val, double min, double max);
  void initStringParam(const std::string &name, const std::string &val);

  void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc);
  void configCallback(const roi_viewpoint_planner::PlannerConfig &conf);
  bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

  void intSlider_setValue(QSlider *slider, const std::string &param, int value);
  void intSpinBox_setPosition(QSpinBox *spinBox, const std::string &param, int position);
  void intValue_sendConfig(QSpinBox *spinBox, const std::string &param);

  void doubleSlider_setValue(QSlider *slider, const std::string &param, double value);
  void doubleSpinBox_setPosition(QDoubleSpinBox *spinBox, const std::string &param, int position);
  void doubleValue_sendConfig(QDoubleSpinBox *spinBox, const std::string &param);

  void sendConfig(const std::string &changed_param);

  template <typename T>
  T getValue(const std::string &param)
  {
      boost::any val;
      parameter_map[param]->getValue(current_config, val);
      return boost::any_cast<T>(val);
  }

  template <typename T>
  T getMin(const std::string &param)
  {
      boost::any val;
      parameter_map[param]->getValue(roi_viewpoint_planner::PlannerConfig::__getMin__(), val);
      return boost::any_cast<T>(val);
  }

  template <typename T>
  T getMax(const std::string &param)
  {
      boost::any val;
      parameter_map[param]->getValue(roi_viewpoint_planner::PlannerConfig::__getMax__(), val);
      return boost::any_cast<T>(val);
  }

  template <typename T>
  T getDefault(const std::string &param)
  {
      boost::any val;
      parameter_map[param]->getValue(roi_viewpoint_planner::PlannerConfig::__getDefault__(), val);
      return boost::any_cast<T>(val);
  }

  template <typename T>
  void setValue(const std::string &param, T val)
  {
      roi_viewpoint_planner::PlannerConfig::AbstractParamDescriptionConstPtr abstr_pd = parameter_map[param];
      const roi_viewpoint_planner::PlannerConfig::ParamDescription<T> *pd = reinterpret_cast<const roi_viewpoint_planner::PlannerConfig::ParamDescription<T>*>(abstr_pd.get());
      current_config.*(pd->field) = val;
  }
};

}

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
