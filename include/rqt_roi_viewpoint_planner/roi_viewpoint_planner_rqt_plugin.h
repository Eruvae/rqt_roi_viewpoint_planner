#ifndef ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
#define ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H

#include <QWidget>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
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
  void on_modeChangeBox_currentIndexChanged(int index);

  void on_activateExecutionCheckBox_toggled(bool checked);

private:
  Ui::RoiViewpointPlannerRqtPlugin ui_;
  QWidget* widget_;
  ros::ServiceClient changePlannerModeClient;
  ros::ServiceClient activatePlanExecutionClient;
};

}

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
