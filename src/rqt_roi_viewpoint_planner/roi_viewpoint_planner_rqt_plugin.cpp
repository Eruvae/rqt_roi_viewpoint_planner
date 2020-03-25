#include "rqt_roi_viewpoint_planner/roi_viewpoint_planner_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <roi_viewpoint_planner/ChangePlannerMode.h>
#include <std_srvs/SetBool.h>

namespace rqt_roi_viewpoint_planner
{

RoiViewpointPlannerRqtPlugin::RoiViewpointPlannerRqtPlugin() :
  rqt_gui_cpp::Plugin(),
  widget_(0)
{
  setObjectName("RoiViewpointPlannerRqtPlugin");
}

void RoiViewpointPlannerRqtPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  connect(ui_.modeChangeBox, SIGNAL(currentIndexChanged(int)), this, SLOT(on_modeChangeBox_currentIndexChanged(int)));
  connect(ui_.activateExecutionCheckBox, SIGNAL(toggled(bool)), this, SLOT(on_activateExecutionCheckBox_toggled(bool)));

  changePlannerModeClient = getNodeHandle().serviceClient<roi_viewpoint_planner::ChangePlannerMode>("/roi_viewpoint_planner/change_planner_mode");
  activatePlanExecutionClient = getNodeHandle().serviceClient<std_srvs::SetBool>("/roi_viewpoint_planner/activate_plan_execution");
}

void RoiViewpointPlannerRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  changePlannerModeClient.shutdown();
  activatePlanExecutionClient.shutdown();
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

void RoiViewpointPlannerRqtPlugin::on_modeChangeBox_currentIndexChanged(int index)
{
  roi_viewpoint_planner::ChangePlannerMode changeMode;
  changeMode.request.mode = index;
  if (changePlannerModeClient.call(changeMode) && changeMode.response.success == true)
  {
    ui_.statusTextBox->setText("Mode change successful");
  }
  else
  {
    ui_.statusTextBox->setText("Mode change failed");
  }
}

void RoiViewpointPlannerRqtPlugin::on_activateExecutionCheckBox_toggled(bool checked)
{
  std_srvs::SetBool activateExecution;
  activateExecution.request.data = checked;
  if (activatePlanExecutionClient.call(activateExecution) && activateExecution.response.success == true)
  {
    ui_.statusTextBox->setText("Activate execution successful");
  }
  else
  {
    ui_.statusTextBox->setText("Activate execution failed");
  }
}

} // namespace roi_viewpoint_planner_rqt_plugin

PLUGINLIB_EXPORT_CLASS(rqt_roi_viewpoint_planner::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(roi_viewpoint_planner_rqt_plugin, RoiViewpointPlannerRqtPlugin, roi_viewpoint_planner_rqt_plugin::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
