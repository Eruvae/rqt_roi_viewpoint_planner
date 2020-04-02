#include "rqt_roi_viewpoint_planner/roi_viewpoint_planner_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMessageBox>
#include <roi_viewpoint_planner/ChangePlannerMode.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  QMessageBox msgBox;
  msgBox.setText("Plan confirmation requested");
  msgBox.setInformativeText("Do you want to save your changes?");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::No);
  int ret = msgBox.exec();
  res.success = (ret == QMessageBox::Yes);
  return true;
}

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
  connect(ui_.requireConfirmationCheckBox, SIGNAL(toggled(bool)), this, SLOT(on_requireConfirmationCheckBox_toggled(bool)));

  changePlannerModeClient = getNodeHandle().serviceClient<roi_viewpoint_planner::ChangePlannerMode>("/roi_viewpoint_planner/change_planner_mode");
  activatePlanExecutionClient = getNodeHandle().serviceClient<std_srvs::SetBool>("/roi_viewpoint_planner/activate_plan_execution");
  confirmPlanExecutionServer = getNodeHandle().advertiseService("/roi_viewpoint_planner/request_execution_confirmation", confirmPlanExecutionCallback);

  configClient = new dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig>("/roi_viewpoint_planner");
}

void RoiViewpointPlannerRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  changePlannerModeClient.shutdown();
  activatePlanExecutionClient.shutdown();
  confirmPlanExecutionServer.shutdown();
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
  /*roi_viewpoint_planner::ChangePlannerMode changeMode;
  changeMode.request.mode = index;
  if (changePlannerModeClient.call(changeMode) && changeMode.response.success == true)
  {
    ui_.statusTextBox->setText("Mode change successful");
  }
  else
  {
    ui_.statusTextBox->setText("Mode change failed");
  }*/
  roi_viewpoint_planner::PlannerConfig config;
  if (!configClient->getCurrentConfiguration(config, ros::Duration(1)))
  {
    ui_.statusTextBox->setText("Mode change failed");
    return;
  }
  config.mode = index;
  if (!configClient->setConfiguration(config))
  {
    ui_.statusTextBox->setText("Mode change failed");
    return;
  }
  ui_.statusTextBox->setText("Mode change successful");
}

void RoiViewpointPlannerRqtPlugin::on_activateExecutionCheckBox_toggled(bool checked)
{
  /*std_srvs::SetBool activateExecution;
  activateExecution.request.data = checked;
  if (activatePlanExecutionClient.call(activateExecution) && activateExecution.response.success == true)
  {
    ui_.statusTextBox->setText("Activate execution successful");
  }
  else
  {
    ui_.statusTextBox->setText("Activate execution failed");
  }*/
  roi_viewpoint_planner::PlannerConfig config;
  if (!configClient->getCurrentConfiguration(config, ros::Duration(1)))
  {
    ui_.statusTextBox->setText("Activate execution failed");
    return;
  }
  config.activate_execution = checked;
  if (!configClient->setConfiguration(config))
  {
    ui_.statusTextBox->setText("Activate execution failed");
    return;
  }
  ui_.statusTextBox->setText("Activate execution successful");
}

void RoiViewpointPlannerRqtPlugin::on_requireConfirmationCheckBox_toggled(bool checked)
{
  roi_viewpoint_planner::PlannerConfig config;
  if (!configClient->getCurrentConfiguration(config, ros::Duration(1)))
  {
    ui_.statusTextBox->setText("Require plan change failed");
    return;
  }
  config.require_execution_confirmation = checked;
  if (!configClient->setConfiguration(config))
  {
    ui_.statusTextBox->setText("Require plan change failed");
    return;
  }
  ui_.statusTextBox->setText("Require plan change successful");
}

} // namespace roi_viewpoint_planner_rqt_plugin

PLUGINLIB_EXPORT_CLASS(rqt_roi_viewpoint_planner::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(roi_viewpoint_planner_rqt_plugin, RoiViewpointPlannerRqtPlugin, roi_viewpoint_planner_rqt_plugin::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
