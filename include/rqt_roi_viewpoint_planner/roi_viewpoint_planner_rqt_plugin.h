#ifndef ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
#define ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H

#include <QWidget>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner/PlannerConfig.h>
#include <roi_viewpoint_planner/PlannerState.h>
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

  void configChanged();

signals:
  void configChangedSignal();

private:
  Ui::RoiViewpointPlannerRqtPlugin ui;
  QWidget* widget;
  roi_viewpoint_planner::PlannerConfig current_config;
  roi_viewpoint_planner::PlannerConfig received_config;
  //ros::ServiceClient changePlannerModeClient;
  //ros::ServiceClient activatePlanExecutionClient;

  ros::Subscriber plannerStateSub;

  ros::ServiceServer confirmPlanExecutionServer;

  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> *configClient;

  void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc);
  void configCallback(const roi_viewpoint_planner::PlannerConfig &conf);
  bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void plannerStateCallback(const roi_viewpoint_planner::PlannerStateConstPtr &state);

  void minRangeSlider_setValue(double value);
  void minRangeSpinBox_setPosition(int position);
  void minRange_sendConfig();
  void maxRangeSlider_setValue(double value);
  void maxRangeSpinBox_setPosition(int position);
  void maxRange_sendConfig();
  void planningTimeSlider_setValue(double value);
  void planningTimeSpinBox_setPosition(int position);
  void planningTime_sendConfig();
};

}

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
