#ifndef ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
#define ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H

#include <QWidget>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner_msgs/PlannerConfig.h>
#include <roi_viewpoint_planner_msgs/PlannerState.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <roi_viewpoint_planner_msgs/LoadOctomap.h>
#include <std_srvs/Trigger.h>
#include "ui_roi_viewpoint_planner_rqt_plugin.h"

namespace rqt_roi_viewpoint_planner
{

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
  void on_modeComboBox_activated(int index);
  void on_activateExecutionCheckBox_clicked(bool checked);
  void on_requireConfirmationCheckBox_clicked(bool checked);
  void on_minRangeSlider_sliderMoved(int position);
  void on_minRangeSlider_sliderReleased();
  void on_minRangeSpinBox_editingFinished();
  void on_maxRangeSlider_sliderMoved(int position);
  void on_maxRangeSlider_sliderReleased();
  void on_maxRangeSpinBox_editingFinished();
  void on_insertOccIfNotMovedCheckBox_clicked(bool checked);
  void on_insertRoiIfNotMovedCheckBox_clicked(bool checked);
  void on_insertOccWhileMovingCheckBox_clicked(bool checked);
  void on_insertRoiWhileMovingCheckBox_clicked(bool checked);
  void on_waitForOccScanCheckBox_clicked(bool checked);
  void on_waitForRoiScanCheckBox_clicked(bool checked);
  void on_publishPlanningStateCheckBox_clicked(bool checked);
  void on_plannerComboBox_activated(QString planner_id);
  void on_planningTimeSlider_sliderMoved(int position);
  void on_planningTimeSlider_sliderReleased();
  void on_planningTimeSpinBox_editingFinished();
  void on_useCartesianMotionCheckBox_clicked(bool checked);
  void on_computeIkWhenSamplingCheckBox_clicked(bool checked);
  void on_velocityScalingSlider_sliderMoved(int position);
  void on_velocityScalingSlider_sliderReleased();
  void on_velocityScalingSpinBox_editingFinished();
  void on_recordMapUpdatesCheckBox_clicked(bool checked);
  void on_recordViewpointsCheckBox_clicked(bool checked);
  void on_saveMapPushButton_clicked();
  void on_loadMapPushButton_clicked();

  // Internal slots
  void configChanged(const roi_viewpoint_planner::PlannerConfig &received_config);
  void planRequest(bool enable);
  void plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

signals:
  // Internal signals
  void configChangedSignal(const roi_viewpoint_planner::PlannerConfig &received_config);
  void planRequestSignal(bool enable);
  void plannerStateSignal(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

private:
  Ui::RoiViewpointPlannerRqtPlugin ui;
  QWidget* widget;
  roi_viewpoint_planner::PlannerConfig current_config;
  //ros::ServiceClient changePlannerModeClient;
  //ros::ServiceClient activatePlanExecutionClient;
  ros::ServiceClient saveOctomapClient;
  ros::ServiceClient loadOctomapClient;

  ros::Subscriber plannerStateSub;

  ros::ServiceServer confirmPlanExecutionServer;

  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> *configClient;

  void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc);
  void configCallback(const roi_viewpoint_planner::PlannerConfig &conf);
  bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

  void minRangeSlider_setValue(double value);
  void minRangeSpinBox_setPosition(int position);
  void minRange_sendConfig();
  void maxRangeSlider_setValue(double value);
  void maxRangeSpinBox_setPosition(int position);
  void maxRange_sendConfig();
  void planningTimeSlider_setValue(double value);
  void planningTimeSpinBox_setPosition(int position);
  void planningTime_sendConfig();
  void velocityScalingSlider_setValue(double value);
  void velocityScalingSpinBox_setPosition(int position);
  void velocityScaling_sendConfig();
};

}

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
