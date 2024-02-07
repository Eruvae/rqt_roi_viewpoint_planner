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
#include <roi_viewpoint_planner_msgs/EvaluatorConfig.h>
#include <roi_viewpoint_planner_msgs/PlannerState.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <roi_viewpoint_planner_msgs/LoadOctomap.h>
#include <roi_viewpoint_planner_msgs/MoveToState.h>
#include <roi_viewpoint_planner_msgs/RandomizePlantPositions.h>
#include <roi_viewpoint_planner_msgs/StartEvaluator.h>
#include <roi_viewpoint_planner_msgs/SaveCurrentRobotState.h>

#include <trolley_simulation/trolley_remote.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include "ui_roi_viewpoint_planner_rqt_plugin.h"
#include "reconfigure_client_gui.h"

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
  void on_saveRobotPosePushButton_clicked();

  void on_loadConfigPushButton_clicked();
  void on_saveConfigPushButton_clicked();

  // Internal slots
  void planRequest(bool enable);
  void plannerStateChanged(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);

  void on_randomizePlantsPushButton_clicked();
  void on_startEvaluatorPushButton_clicked();

  void updateTrolleyPosition();
  void on_trolleyMovePushButton_clicked();
  void on_trolleyLiftPushButton_clicked();

  void on_flipWssrPushButton_clicked();
  void on_updateWssrMarkerPushButton_clicked();

signals:
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
  ros::ServiceClient saveOctomapClient;
  ros::ServiceClient loadOctomapClient;
  ros::ServiceClient resetOctomapClient;
  ros::ServiceClient randomizePlantPositionsClient;
  ros::ServiceClient startEvaluatorClient;
  ros::ServiceClient saveCurrentRobotStateClient;
  ros::ServiceClient flipWssrClient;
  ros::ServiceClient updateWssrMarkerClient;

  std::unique_ptr<trolley_remote::TrolleyRemote> trolley_remote;
  QTimer *trolley_update_timer;

  void setQuaternion(geometry_msgs::Quaternion &q, double roll, double pitch, double yaw);

  ros::Subscriber plannerStateSub;

  ros::ServiceServer confirmPlanExecutionServer;

  std::array<std::unique_ptr<AbstractReconfigureClient>, 3> configClients;

  bool confirmPlanExecutionCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void plannerStateCallback(const roi_viewpoint_planner_msgs::PlannerStateConstPtr &state);
};

} // namespace rqt_roi_viewpoint_planner

#endif // ROI_VIEWPOINT_PLANNER_RQT_PLUGIN_H
