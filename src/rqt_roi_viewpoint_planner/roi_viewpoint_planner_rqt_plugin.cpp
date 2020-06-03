#include "rqt_roi_viewpoint_planner/roi_viewpoint_planner_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMessageBox>
#include <QTimer>
#include <QtGlobal>
#include <roi_viewpoint_planner/ChangePlannerMode.h>

namespace rqt_roi_viewpoint_planner
{

RoiViewpointPlannerRqtPlugin::RoiViewpointPlannerRqtPlugin() :
  rqt_gui_cpp::Plugin(),
  widget(0)
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

  connect(this, SIGNAL(configChangedSignal()), this, SLOT(configChanged()));

  connect(ui.modeComboBox, SIGNAL(activated(int)), this, SLOT(on_modeComboBox_activated(int)));
  connect(ui.activateExecutionCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_activateExecutionCheckBox_clicked(bool)));
  connect(ui.requireConfirmationCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_requireConfirmationCheckBox_clicked(bool)));
  connect(ui.minRangeSlider, SIGNAL(sliderMoved(int)), this, SLOT(on_minRangeSlider_sliderMoved(int)));
  connect(ui.minRangeSlider, SIGNAL(sliderReleased()), this, SLOT(on_minRangeSlider_sliderReleased()));
  connect(ui.minRangeSpinBox, SIGNAL(editingFinished()), this, SLOT(on_minRangeSpinBox_editingFinished()));
  connect(ui.maxRangeSlider, SIGNAL(sliderMoved(int)), this, SLOT(on_maxRangeSlider_sliderMoved(int)));
  connect(ui.maxRangeSlider, SIGNAL(sliderReleased()), this, SLOT(on_maxRangeSlider_sliderReleased()));
  connect(ui.maxRangeSpinBox, SIGNAL(editingFinished()), this, SLOT(on_maxRangeSpinBox_editingFinished()));
  connect(ui.insertOccIfNotMovedCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_insertOccIfNotMovedCheckBox_clicked(bool)));
  connect(ui.insertRoiIfNotMovedCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_insertRoiIfNotMovedCheckBox_clicked(bool)));
  connect(ui.insertOccWhileMovingCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_insertOccWhileMovingCheckBox_clicked(bool)));
  connect(ui.insertRoiWhileMovingCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_insertRoiWhileMovingCheckBox_clicked(bool)));
  connect(ui.waitForOccScanCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_waitForOccScanCheckBox_clicked(bool)));
  connect(ui.waitForRoiScanCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_waitForRoiScanCheckBox_clicked(bool)));
  connect(ui.publishPlanningStateCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_publishPlanningStateCheckBox_clicked(bool)));
  connect(ui.plannerComboBox, SIGNAL(activated(QString)), this, SLOT(on_plannerComboBox_activated(QString)));
  connect(ui.planningTimeSlider, SIGNAL(sliderMoved(int)), this, SLOT(on_planningTimeSlider_sliderMoved(int)));
  connect(ui.planningTimeSlider, SIGNAL(sliderReleased()), this, SLOT(on_planningTimeSlider_sliderReleased()));
  connect(ui.planningTimeSpinBox, SIGNAL(editingFinished()), this, SLOT(on_planningTimeSpinBox_editingFinished()));
  connect(ui.useCartesianMotionCheckBox, SIGNAL(clicked(bool)), this, SLOT(on_useCartesianMotionCheckBox_clicked(bool)));

  //changePlannerModeClient = getNodeHandle().serviceClient<roi_viewpoint_planner::ChangePlannerMode>("/roi_viewpoint_planner/change_planner_mode");
  //activatePlanExecutionClient = getNodeHandle().serviceClient<std_srvs::SetBool>("/roi_viewpoint_planner/activate_plan_execution");

  plannerStateSub = getNodeHandle().subscribe("/roi_viewpoint_planner/planner_state", 10, &RoiViewpointPlannerRqtPlugin::plannerStateCallback, this);

  confirmPlanExecutionServer = getNodeHandle().advertiseService("/roi_viewpoint_planner/request_execution_confirmation", &RoiViewpointPlannerRqtPlugin::confirmPlanExecutionCallback, this);

  configClient = new dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig>("/roi_viewpoint_planner",
                 boost::bind(&RoiViewpointPlannerRqtPlugin::configCallback, this, _1));
}

void RoiViewpointPlannerRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  //changePlannerModeClient.shutdown();
  //activatePlanExecutionClient.shutdown();
  plannerStateSub.shutdown();
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

void RoiViewpointPlannerRqtPlugin::on_modeComboBox_activated(int index)
{
  current_config.mode = index;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Mode change failed");
    return;
  }
  ui.statusTextBox->setText("Mode change successful");
}

void RoiViewpointPlannerRqtPlugin::on_activateExecutionCheckBox_clicked(bool checked)
{
  current_config.activate_execution = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Activate execution failed");
    return;
  }
  ui.statusTextBox->setText("Activate execution successful");
}

void RoiViewpointPlannerRqtPlugin::on_requireConfirmationCheckBox_clicked(bool checked)
{
  current_config.require_execution_confirmation = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Require plan change failed");
    return;
  }
  ui.statusTextBox->setText("Require plan change successful");
}

void RoiViewpointPlannerRqtPlugin::minRangeSlider_setValue(double value)
{
  double minVal = ui.minRangeMin->text().toDouble();
  double maxVal = ui.minRangeMax->text().toDouble();
  int position = qBound(0, (int)((value - minVal) / (maxVal - minVal) * 100.0), 100);
  ui.minRangeSlider->setValue(position);
}

void RoiViewpointPlannerRqtPlugin::minRangeSpinBox_setPosition(int position)
{
  double minVal = ui.minRangeMin->text().toDouble();
  double maxVal = ui.minRangeMax->text().toDouble();
  ui.minRangeSpinBox->setValue(minVal + (double)position / 100.0 * (maxVal - minVal));
}

void RoiViewpointPlannerRqtPlugin::minRange_sendConfig()
{
  current_config.sensor_min_range = ui.minRangeSpinBox->value();
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Min range change failed");
    return;
  }
  ui.statusTextBox->setText("Min range change successful");
}

void RoiViewpointPlannerRqtPlugin::on_minRangeSlider_sliderMoved(int position)
{
  ROS_INFO("MinRange slider moved");
  minRangeSpinBox_setPosition(position);
}

void RoiViewpointPlannerRqtPlugin::on_minRangeSlider_sliderReleased()
{
  ROS_INFO("MinRange slider released");
  minRange_sendConfig();
}

void RoiViewpointPlannerRqtPlugin::on_minRangeSpinBox_editingFinished()
{
  ROS_INFO("MinRange edit finished");
  minRangeSlider_setValue(ui.minRangeSpinBox->value());
  minRange_sendConfig();
}

void RoiViewpointPlannerRqtPlugin::maxRangeSlider_setValue(double value)
{
  double minVal = ui.maxRangeMin->text().toDouble();
  double maxVal = ui.maxRangeMax->text().toDouble();
  int position = qBound(0, (int)((value - minVal) / (maxVal - minVal) * 100.0), 100);
  ui.maxRangeSlider->setValue(position);
}

void RoiViewpointPlannerRqtPlugin::maxRangeSpinBox_setPosition(int position)
{
  double minVal = ui.maxRangeMin->text().toDouble();
  double maxVal = ui.maxRangeMax->text().toDouble();
  ui.maxRangeSpinBox->setValue(minVal + (double)position / 100.0 * (maxVal - minVal));
}

void RoiViewpointPlannerRqtPlugin::maxRange_sendConfig()
{
  current_config.sensor_max_range = ui.maxRangeSpinBox->value();
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Max range change failed");
    return;
  }
  ui.statusTextBox->setText("Max range change successful");
}

void RoiViewpointPlannerRqtPlugin::on_maxRangeSlider_sliderMoved(int position)
{
  ROS_INFO("MaxRange slider moved");
  maxRangeSpinBox_setPosition(position);
}

void RoiViewpointPlannerRqtPlugin::on_maxRangeSlider_sliderReleased()
{
  ROS_INFO("MaxRange slider released");
  maxRange_sendConfig();
}

void RoiViewpointPlannerRqtPlugin::on_maxRangeSpinBox_editingFinished()
{
  ROS_INFO("MaxRange edit finished");
  maxRangeSlider_setValue(ui.maxRangeSpinBox->value());
  maxRange_sendConfig();
}

void RoiViewpointPlannerRqtPlugin::on_insertOccIfNotMovedCheckBox_clicked(bool checked)
{
  current_config.insert_occ_if_not_moved = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Insert occ if not moved change failed");
    return;
  }
  ui.statusTextBox->setText("Insert occ if not moved change successful");
}

void RoiViewpointPlannerRqtPlugin::on_insertRoiIfNotMovedCheckBox_clicked(bool checked)
{
  current_config.insert_roi_if_not_moved = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Insert ROI if not moved change failed");
    return;
  }
  ui.statusTextBox->setText("Insert ROI if not moved change successful");
}

void RoiViewpointPlannerRqtPlugin::on_insertOccWhileMovingCheckBox_clicked(bool checked)
{
  current_config.insert_occ_while_moving = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Insert occ while moving change failed");
    return;
  }
  ui.statusTextBox->setText("Insert occ while moving change successful");
}

void RoiViewpointPlannerRqtPlugin::on_insertRoiWhileMovingCheckBox_clicked(bool checked)
{
  current_config.insert_roi_while_moving = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Insert ROI while moving change failed");
    return;
  }
  ui.statusTextBox->setText("Insert ROI while moving change successful");
}

void RoiViewpointPlannerRqtPlugin::on_waitForOccScanCheckBox_clicked(bool checked)
{
  current_config.wait_for_occ_scan = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Wait for occ scan change failed");
    return;
  }
  ui.statusTextBox->setText("Wait for occ scan change successful");
}

void RoiViewpointPlannerRqtPlugin::on_waitForRoiScanCheckBox_clicked(bool checked)
{
  current_config.wait_for_roi_scan = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Wait for ROI scan change failed");
    return;
  }
  ui.statusTextBox->setText("Wait for ROI scan change successful");
}

void RoiViewpointPlannerRqtPlugin::on_publishPlanningStateCheckBox_clicked(bool checked)
{
  current_config.publish_planning_state = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Publish planning state change failed");
    return;
  }
  ui.statusTextBox->setText("Publish planning state change successful");
}

void RoiViewpointPlannerRqtPlugin::on_plannerComboBox_activated(QString planner_id)
{
  current_config.planner = (planner_id + "kConfigDefault").toStdString();
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Planner change failed");
    return;
  }
  ui.statusTextBox->setText("Planner change successful");
}

void RoiViewpointPlannerRqtPlugin::planningTimeSlider_setValue(double value)
{
  double minVal = ui.planningTimeMin->text().toDouble();
  double maxVal = ui.planningTimeMax->text().toDouble();
  int position = qBound(0, (int)((value - minVal) / (maxVal - minVal) * 100.0), 100);
  ui.planningTimeSlider->setValue(position);
}

void RoiViewpointPlannerRqtPlugin::planningTimeSpinBox_setPosition(int position)
{
  double minVal = ui.planningTimeMin->text().toDouble();
  double maxVal = ui.planningTimeMax->text().toDouble();
  ui.planningTimeSpinBox->setValue(minVal + (double)position / 100.0 * (maxVal - minVal));
}

void RoiViewpointPlannerRqtPlugin::planningTime_sendConfig()
{
  current_config.planning_time = ui.planningTimeSpinBox->value();
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Planning time change failed");
    return;
  }
  ui.statusTextBox->setText("Planning time change successful");
}

void RoiViewpointPlannerRqtPlugin::on_planningTimeSlider_sliderMoved(int position)
{
  planningTimeSpinBox_setPosition(position);
}

void RoiViewpointPlannerRqtPlugin::on_planningTimeSlider_sliderReleased()
{
  planningTime_sendConfig();
}

void RoiViewpointPlannerRqtPlugin::on_planningTimeSpinBox_editingFinished()
{
  planningTimeSlider_setValue(ui.planningTimeSpinBox->value());
  planningTime_sendConfig();
}

void RoiViewpointPlannerRqtPlugin::on_useCartesianMotionCheckBox_clicked(bool checked)
{
  current_config.use_cartesian_motion = checked;
  if (!configClient->setConfiguration(current_config))
  {
    ui.statusTextBox->setText("Use cartesian motion change failed");
    return;
  }
  ui.statusTextBox->setText("Use cartesian motion change successful");
}

void RoiViewpointPlannerRqtPlugin::configChanged()
{
  if (received_config.mode != current_config.mode)
  {
    ui.modeComboBox->setCurrentIndex(received_config.mode);
  }
  if (received_config.activate_execution != current_config.activate_execution)
  {
    ui.activateExecutionCheckBox->setChecked(received_config.activate_execution);
  }
  if (received_config.require_execution_confirmation != current_config.require_execution_confirmation)
  {
    ui.requireConfirmationCheckBox->setChecked(received_config.require_execution_confirmation);
  }
  if (received_config.sensor_min_range != current_config.sensor_min_range)
  {
    minRangeSlider_setValue(received_config.sensor_min_range);
    ui.minRangeSpinBox->setValue(received_config.sensor_min_range);
  }
  if (received_config.sensor_max_range != current_config.sensor_max_range)
  {
    maxRangeSlider_setValue(received_config.sensor_max_range);
    ui.maxRangeSpinBox->setValue(received_config.sensor_max_range);
  }
  if (received_config.insert_occ_if_not_moved != current_config.insert_occ_if_not_moved)
  {
    ui.insertOccIfNotMovedCheckBox->setChecked(received_config.insert_occ_if_not_moved);
  }
  if (received_config.insert_roi_if_not_moved != current_config.insert_roi_if_not_moved)
  {
    ui.insertRoiIfNotMovedCheckBox->setChecked(received_config.insert_roi_if_not_moved);
  }
  if (received_config.insert_occ_while_moving != current_config.insert_occ_while_moving)
  {
    ui.insertOccWhileMovingCheckBox->setChecked(received_config.insert_occ_while_moving);
  }
  if (received_config.insert_roi_while_moving != current_config.insert_roi_while_moving)
  {
    ui.insertRoiWhileMovingCheckBox->setChecked(received_config.insert_roi_while_moving);
  }
  if (received_config.wait_for_occ_scan != current_config.wait_for_occ_scan)
  {
    ui.waitForOccScanCheckBox->setChecked(received_config.wait_for_occ_scan);
  }
  if (received_config.wait_for_roi_scan != current_config.wait_for_roi_scan)
  {
    ui.waitForRoiScanCheckBox->setChecked(received_config.wait_for_roi_scan);
  }
  if (received_config.publish_planning_state != current_config.publish_planning_state)
  {
    ui.publishPlanningStateCheckBox->setChecked(received_config.publish_planning_state);
  }
  if (received_config.planner != current_config.planner)
  {
    ui.plannerComboBox->setCurrentText(QString::fromStdString(received_config.planner.substr(0, received_config.planner.size() - 14)));
  }
  if (received_config.planning_time != current_config.planning_time)
  {
    planningTimeSlider_setValue(received_config.planning_time);
    ui.planningTimeSpinBox->setValue(received_config.planning_time);
  }
  if (received_config.use_cartesian_motion != current_config.use_cartesian_motion)
  {
    ui.useCartesianMotionCheckBox->setChecked(received_config.use_cartesian_motion);
  }

  current_config = received_config;
}

void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc)
{
  ROS_INFO_STREAM("Description callback called");
}

void RoiViewpointPlannerRqtPlugin::configCallback(const roi_viewpoint_planner::PlannerConfig &conf)
{
  ROS_INFO_STREAM("Config callback called");
  static bool first_config_received = false;
  received_config = conf;
  //if (!first_config_received) // extra wait on first config to wait for initalization
  //{
    ROS_INFO_STREAM("First callback; wait for 50 ms");
    first_config_received = true;
    QTimer::singleShot(50, this, &RoiViewpointPlannerRqtPlugin::configChanged);
  /*}
  else
  {
    emit configChangedSignal();
  }*/
}

bool RoiViewpointPlannerRqtPlugin::confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  QMessageBox msgBox;
  msgBox.setText("Plan confirmation requested");
  msgBox.setInformativeText("Do you want to execute the plan (see rviz)?");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::No);
  int ret = msgBox.exec();
  res.success = (ret == QMessageBox::Yes);
  return true;
}

void RoiViewpointPlannerRqtPlugin::plannerStateCallback(const roi_viewpoint_planner::PlannerStateConstPtr &state)
{
  ui.movingLed->setState(state->robot_is_moving);
  ui.occScanLed->setState(state->occupancy_scanned);
  ui.roiScanLed->setState(state->roi_scanned);
}

} // namespace roi_viewpoint_planner_rqt_plugin

PLUGINLIB_EXPORT_CLASS(rqt_roi_viewpoint_planner::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(roi_viewpoint_planner_rqt_plugin, RoiViewpointPlannerRqtPlugin, roi_viewpoint_planner_rqt_plugin::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
