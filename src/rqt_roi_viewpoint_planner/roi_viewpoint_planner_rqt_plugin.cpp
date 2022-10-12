#include "rqt_roi_viewpoint_planner/roi_viewpoint_planner_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMessageBox>
#include <QTimer>
#include <QtGlobal>
#include <QFileDialog>

#include <stdio.h>
#include <stdlib.h>


Q_DECLARE_METATYPE(roi_viewpoint_planner_msgs::PlannerStateConstPtr)

namespace rqt_roi_viewpoint_planner
{

RoiViewpointPlannerRqtPlugin::RoiViewpointPlannerRqtPlugin() :
  rqt_gui_cpp::Plugin(),
  widget(nullptr)
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

  qRegisterMetaType<roi_viewpoint_planner_msgs::PlannerStateConstPtr>();

  connect(this, SIGNAL(rvpConfigChangedSignal(const roi_viewpoint_planner::PlannerConfig&)), this, SLOT(rvpConfigChanged(const roi_viewpoint_planner::PlannerConfig&)));
  connect(this, SIGNAL(planRequestSignal(bool)), this, SLOT(planRequest(bool)));
  connect(this, SIGNAL(plannerStateSignal(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &)), this, SLOT(plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &)));

  connect(ui.saveMapPushButton, SIGNAL(clicked()), this, SLOT(on_saveMapPushButton_clicked()));
  connect(ui.loadMapPushButton, SIGNAL(clicked()), this, SLOT(on_loadMapPushButton_clicked()));
  connect(ui.resetMapPushButton, SIGNAL(clicked()), this, SLOT(on_resetMapPushButton_clicked()));
  connect(ui.moveArmPushButton, SIGNAL(clicked()), this, SLOT(on_moveArmPushButton_clicked()));
  connect(ui.randomizePlantsPushButton, SIGNAL(clicked()), this, SLOT(on_randomizePlantsPushButton_clicked()));
  connect(ui.startEvaluatorPushButton, SIGNAL(clicked()), this, SLOT(on_startEvaluatorPushButton_clicked()));
  connect(ui.saveRobotPosePushButton, SIGNAL(clicked()), this, SLOT(on_saveRobotPosePushButton_clicked()));

  connect(ui.loadConfigPushButton, SIGNAL(clicked()), this, SLOT(on_loadConfigPushButton_clicked()));
  connect(ui.saveConfigPushButton, SIGNAL(clicked()), this, SLOT(on_saveConfigPushButton_clicked()));



  moveToStateThread.reset(new MoveToStateThread(getNodeHandle()));
  connect(moveToStateThread.get(), SIGNAL(success(QString)), ui.statusTextBox, SLOT(setText(QString)));
  connect(moveToStateThread.get(), SIGNAL(error(QString)), ui.statusTextBox, SLOT(setText(QString)));

  saveOctomapClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::SaveOctomap>("/roi_viewpoint_planner/save_octomap");
  loadOctomapClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::LoadOctomap>("/roi_viewpoint_planner/load_octomap");
  resetOctomapClient = getNodeHandle().serviceClient<std_srvs::Empty>("/roi_viewpoint_planner/reset_octomap");
  randomizePlantPositionsClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::RandomizePlantPositions>("/roi_viewpoint_planner/randomize_plant_positions");
  startEvaluatorClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::StartEvaluator>("/roi_viewpoint_planner/start_evaluator");
  saveCurrentRobotStateClient = getNodeHandle().serviceClient<roi_viewpoint_planner_msgs::SaveCurrentRobotState>("/roi_viewpoint_planner/save_robot_state");

  plannerStateSub = getNodeHandle().subscribe("/roi_viewpoint_planner/planner_state", 10, &RoiViewpointPlannerRqtPlugin::plannerStateCallback, this);

  confirmPlanExecutionServer = getNodeHandle().advertiseService("/roi_viewpoint_planner/request_execution_confirmation", &RoiViewpointPlannerRqtPlugin::confirmPlanExecutionCallback, this);

  rvpConfigClient = new ReconfigureClient<roi_viewpoint_planner::PlannerConfig>("/roi_viewpoint_planner", ui.configTabWidget, ui.statusTextBox);
  vmpConfigClient = new ReconfigureClient<view_motion_planner::VmpConfig>("/view_motion_planner", ui.configTabWidget, ui.statusTextBox);

  //ROS_INFO_STREAM("Init is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
}

void RoiViewpointPlannerRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  plannerStateSub.shutdown();
  confirmPlanExecutionServer.shutdown();
  saveOctomapClient.shutdown();
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

void RoiViewpointPlannerRqtPlugin::planRequest(bool enable)
{
  ROS_INFO_STREAM("Plan request slot is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  ui.planRequestLed->setState(enable);
  ui.planAcceptPushButton->setEnabled(enable);
  ui.planDeclinePushButton->setEnabled(enable);
  if (enable)
    ui.statusTextBox->setText("Plan requested; check rviz for plan");
  else
    ui.statusTextBox->setText("Plan request answered");
}

void RoiViewpointPlannerRqtPlugin::plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state)
{
  ROS_INFO_STREAM("Planner state slot is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  ui.planningLed->setState(false);
  ui.movingLed->setState(state->robot_is_moving);
  ui.scanLed->setState(state->scan_inserted);
}

void descriptionCallback(const dynamic_reconfigure::ConfigDescription& desc)
{
  ROS_INFO_STREAM("Description callback called");
}

// DO NOT DIRECTLY UPDATE UI ELEMENTS HERE
bool RoiViewpointPlannerRqtPlugin::confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO_STREAM("Confirm plan callback is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  emit planRequestSignal(true);

  QEventLoop loop;
  bool accepted = false;
  connect( ui.planAcceptPushButton, &QPushButton::clicked, [&]() { accepted = true; loop.quit(); });
  connect( ui.planDeclinePushButton, &QPushButton::clicked, [&]() { accepted = false; loop.quit(); });
  ROS_INFO_STREAM("Loop starting");
  loop.exec();
  ROS_INFO_STREAM("Loop left");
  emit planRequestSignal(false);
  ROS_INFO_STREAM("Plan request signal emitted starting");
  if (accepted)
    res.success = true;
  else
    res.success = false;

  return true;
}

// DO NOT DIRECTLY UPDATE UI ELEMENTS HERE
void RoiViewpointPlannerRqtPlugin::plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state)
{
  //ROS_INFO_STREAM("Planner state callback is GUI thread: " << (QThread::currentThread() == QCoreApplication::instance()->thread()));
  emit plannerStateSignal(state);
}

void RoiViewpointPlannerRqtPlugin::on_saveMapPushButton_clicked()
{
  roi_viewpoint_planner_msgs::SaveOctomap srv;
  if (saveOctomapClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Map saved successfully");
    else
      ui.statusTextBox->setText("Map could not be saved");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call save map service");
  }
}

void RoiViewpointPlannerRqtPlugin::setQuaternion(geometry_msgs::Quaternion &q, double roll, double pitch, double yaw)
{
  double halfYaw = yaw * 0.5 * M_PI / 180.0;
  double halfPitch = pitch * 0.5 * M_PI / 180.0;
  double halfRoll = roll * 0.5 * M_PI / 180.0;
  double cosYaw = std::cos(halfYaw);
  double sinYaw = std::sin(halfYaw);
  double cosPitch = std::cos(halfPitch);
  double sinPitch = std::sin(halfPitch);
  double cosRoll = std::cos(halfRoll);
  double sinRoll = std::sin(halfRoll);
  q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
}

void RoiViewpointPlannerRqtPlugin::on_loadMapPushButton_clicked()
{
  QString file_path = QFileDialog::getOpenFileName(widget, QString(), QString(), "Octree (*.ot *.bt)");
  if (file_path.isEmpty())
    return;

  roi_viewpoint_planner_msgs::LoadOctomap srv;
  srv.request.filename = file_path.toStdString();
  srv.request.offset.translation.x = ui.mapLoadOffsetXSpinBox->value();
  srv.request.offset.translation.y = ui.mapLoadOffsetYSpinBox->value();
  srv.request.offset.translation.z = ui.mapLoadOffsetZSpinBox->value();
  setQuaternion(srv.request.offset.rotation, ui.mapLoadOffsetRollSpinBox->value(), ui.mapLoadOffsetPitchSpinBox->value(), ui.mapLoadOffsetYawSpinBox->value());

  if (loadOctomapClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Map loaded successfully");
    else
      ui.statusTextBox->setText("Error loading map: " + QString::fromStdString(srv.response.error_message));
  }
  else
  {
    ui.statusTextBox->setText("Failed to call load map service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_resetMapPushButton_clicked()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(widget, "Confirm reset", "Do you really want to delete the current map?", QMessageBox::Yes|QMessageBox::No);
  if (reply == QMessageBox::No)
    return;

  std_srvs::Empty srv;
  if (resetOctomapClient.call(srv))
  {
    ui.statusTextBox->setText("Map reset successfully");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call load map service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_moveArmPushButton_clicked()
{
  if (moveToStateThread->isRunning())
  {
    ui.statusTextBox->setText("Arm is already moving");
    return;
  }
  int index = ui.moveArmComboBox->currentIndex();
  if (index < 0 || index >= MOVE_CONFIGS.size())
  {
    ui.statusTextBox->setText("Invalid move config");
    return;
  }
  if (!moveToStateThread->setGoalState(MOVE_CONFIGS[index], ui.moveArmComboBox->currentText()))
  {
    ui.statusTextBox->setText("Couldn't set move config");
    return;
  }
  moveToStateThread->start();
}

void RoiViewpointPlannerRqtPlugin::on_randomizePlantsPushButton_clicked()
{
  roi_viewpoint_planner_msgs::RandomizePlantPositions srv;
  srv.request.min_point.x = ui.randMinXSpinBox->value();
  srv.request.min_point.y = ui.randMinYSpinBox->value();
  srv.request.min_point.z = ui.randMinZSpinBox->value();
  srv.request.max_point.x = ui.randMaxXSpinBox->value();
  srv.request.max_point.y = ui.randMaxYSpinBox->value();
  srv.request.max_point.z = ui.randMaxZSpinBox->value();
  srv.request.min_dist = ui.randMinDistSpinBox->value();
  if (randomizePlantPositionsClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Plant positions randomized");
    else
      ui.statusTextBox->setText("Couldn't randomize plant positions");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call start randomize plant positions service");
  }
}

void RoiViewpointPlannerRqtPlugin::on_startEvaluatorPushButton_clicked()
{
  roi_viewpoint_planner_msgs::StartEvaluator srv;
  srv.request.num_evals = ui.evalTrialsSpinBox->value();
  srv.request.episode_end_param = static_cast<uint8_t>(ui.evalEndParamComboBox->currentIndex());
  srv.request.episode_duration = ui.evalDurationSpinBox->value();
  srv.request.starting_index = ui.startingIndexSpinBox->value();
  srv.request.randomize_plants = ui.evalRandomizeCheckBox->isChecked();
  srv.request.min_point.x = ui.randMinXSpinBox->value();
  srv.request.min_point.y = ui.randMinYSpinBox->value();
  srv.request.min_point.z = ui.randMinZSpinBox->value();
  srv.request.max_point.x = ui.randMaxXSpinBox->value();
  srv.request.max_point.y = ui.randMaxYSpinBox->value();
  srv.request.max_point.z = ui.randMaxZSpinBox->value();
  srv.request.min_dist = ui.randMinDistSpinBox->value();
  if (startEvaluatorClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Evaluator started");
    else
      ui.statusTextBox->setText("Couldn't start evaluator");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call start evaluator service");
  }
}

void RoiViewpointPlannerRqtPlugin:: on_saveRobotPosePushButton_clicked()
{
  std::string pose_name = ui.poseNameLineEdit->text().toStdString();
  if(pose_name.empty())
  {
    ROS_WARN("Pose name empty");
    ui.statusTextBox->setText("Pose Name Invalid");
    return;
  }
  roi_viewpoint_planner_msgs::SaveCurrentRobotState srv;
  srv.request.pose_name = pose_name;

  if(saveCurrentRobotStateClient.call(srv))
  {
    if (srv.response.success)
      ui.statusTextBox->setText("Current Robot Pose Saved Successfully");
    else
      ui.statusTextBox->setText("Failed to save current robot pose");
  }
  else
  {
    ui.statusTextBox->setText("Failed to call save current robot state service");
  }

}

void RoiViewpointPlannerRqtPlugin::on_loadConfigPushButton_clicked()
{
  QString file_path = QFileDialog::getOpenFileName(widget, QString(), QString(), "YAML (*.yaml *.yml)");
  if (file_path.isEmpty())
  {
    ui.statusTextBox->setText("YAML File name empty");
    return;      
  }
  std::string load_command = "rosrun dynamic_reconfigure dynparam load /view_motion_planner ";
  std::string yaml_filepath = file_path.toStdString();
  try
  {
    system((load_command + yaml_filepath).c_str());
    ui.statusTextBox->setText("YAML File loaded successfully");

  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ui.statusTextBox->setText("Failed to load Config from YAML File");
  }
}

void RoiViewpointPlannerRqtPlugin::on_saveConfigPushButton_clicked()
{
  QString file_path = QFileDialog::getSaveFileName(widget, tr("Save YAML File"),
				"untitled.yaml", tr("YAML (*.yaml)"));

  if (file_path.isEmpty())
  {
    ui.statusTextBox->setText("YAML File name empty");
    return;      
  }

  std::string yaml_filepath = file_path.toStdString();

  std::string dump_command = "rosrun dynamic_reconfigure dynparam dump /view_motion_planner ";

  try
  {
    system((dump_command + yaml_filepath).c_str());
    ui.statusTextBox->setText("YAML File saved successfully");

  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ui.statusTextBox->setText("Failed to save YAML File");
  }
    
}


constexpr std::array<std::array<double, 6>, 5> RoiViewpointPlannerRqtPlugin::MOVE_CONFIGS;

} // namespace roi_viewpoint_planner_rqt_plugin

PLUGINLIB_EXPORT_CLASS(rqt_roi_viewpoint_planner::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(roi_viewpoint_planner_rqt_plugin, RoiViewpointPlannerRqtPlugin, roi_viewpoint_planner_rqt_plugin::RoiViewpointPlannerRqtPlugin, rqt_gui_cpp::Plugin)
