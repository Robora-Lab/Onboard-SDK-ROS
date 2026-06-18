/** @file dji_sdk_node.h
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  A ROS wrapper to interact with DJI onboard SDK
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#ifndef DJI_SDK_NODE_MAIN_H
#define DJI_SDK_NODE_MAIN_H

//! ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

//! ROS standard msgs
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nmea_msgs/msg/sentence.hpp>

//! msgs
#include <dji_sdk/msg/gimbal.hpp>
#include <dji_sdk/msg/mobile_data.hpp>
#include <dji_sdk/msg/payload_data.hpp>
#include <dji_sdk/msg/flight_anomaly.hpp>
#include <dji_sdk/msg/vo_position.hpp>
#include <dji_sdk/msg/fc_time_in_utc.hpp>
#include <dji_sdk/msg/gpsutc.hpp>

//! mission service
// missionManager
#include <dji_sdk/srv/mission_status.hpp>
// waypoint
#include <dji_sdk/srv/mission_wp_action.hpp>
#include <dji_sdk/srv/mission_wp_get_info.hpp>
#include <dji_sdk/srv/mission_wp_get_speed.hpp>
#include <dji_sdk/srv/mission_wp_set_speed.hpp>
#include <dji_sdk/srv/mission_wp_upload.hpp>
// hotpoint
#include <dji_sdk/srv/mission_hp_action.hpp>
#include <dji_sdk/srv/mission_hp_get_info.hpp>
#include <dji_sdk/srv/mission_hp_reset_yaw.hpp>
#include <dji_sdk/srv/mission_hp_update_radius.hpp>
#include <dji_sdk/srv/mission_hp_update_yaw_rate.hpp>
#include <dji_sdk/srv/mission_hp_upload.hpp>
// hardsync
#include <dji_sdk/srv/set_hard_sync.hpp>

//! service headers
#include <dji_sdk/srv/activation.hpp>
#include <dji_sdk/srv/camera_action.hpp>
#include <dji_sdk/srv/drone_arm_control.hpp>
#include <dji_sdk/srv/drone_task_control.hpp>
#include <dji_sdk/srv/mfio_config.hpp>
#include <dji_sdk/srv/mfio_set_value.hpp>
#include <dji_sdk/srv/sdk_control_authority.hpp>
#include <dji_sdk/srv/set_local_pos_ref.hpp>
#include <dji_sdk/srv/send_mobile_data.hpp>
#include <dji_sdk/srv/send_payload_data.hpp>
#include <dji_sdk/srv/query_drone_version.hpp>
#ifdef ADVANCED_SENSING
#include <dji_sdk/srv/stereo240p_subscription.hpp>
#include <dji_sdk/srv/stereo_depth_subscription.hpp>
#include <dji_sdk/srv/stereo_vga_subscription.hpp>
#include <dji_sdk/srv/setup_camera_stream.hpp>
#endif

//! SDK library
#include <dji_vehicle.hpp>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

using namespace DJI::OSDK;

class DJISDKNode : public rclcpp::Node
{
public:
  DJISDKNode();
  ~DJISDKNode();

  enum TELEMETRY_TYPE
  {
    USE_BROADCAST = 0,
    USE_SUBSCRIBE = 1
  };

  enum
  {
    PACKAGE_ID_5HZ   = 0,
    PACKAGE_ID_50HZ  = 1,
    PACKAGE_ID_100HZ = 2,
    PACKAGE_ID_400HZ = 3
  };

private:
  bool initVehicle();
  bool initServices();
  bool initFlightControl();
  bool initSubscriber();
  bool initPublisher();
  bool initActions();
  bool initDataSubscribeFromFC();
  void cleanUpSubscribeFromFC();
  bool validateSerialDevice(LinuxSerialDevice* serialDevice);
  bool isM100();

  /*!
   * @note this function exists here instead of inside the callback function
   *        due to the usages, i.e. we not only provide service call but also
   *        call it for the user when this node was instantiated
   *        we cannot call a service without serviceClient, which is in another
   * node
   */
  ACK::ErrorCode activate(int l_app_id, std::string l_enc_key);

  //! flight control subscriber callbacks
  void flightControlSetpointCallback(
    const sensor_msgs::msg::Joy::SharedPtr pMsg);

  void flightControlPxPyPzYawCallback(
    const sensor_msgs::msg::Joy::SharedPtr pMsg);

  void flightControlVxVyVzYawrateCallback(
    const sensor_msgs::msg::Joy::SharedPtr pMsg);

  void flightControlRollPitchPzYawrateCallback(
    const sensor_msgs::msg::Joy::SharedPtr pMsg);

  //! general subscriber callbacks
  void gimbalAngleCtrlCallback(const dji_sdk::msg::Gimbal msg);
  void gimbalSpeedCtrlCallback(
    const geometry_msgs::msg::Vector3Stamped msg);

  //! general service callbacks
  void droneActivationCallback(const dji_sdk::srv::Activation::Request::SharedPtr  request,
                               dji_sdk::srv::Activation::Response::SharedPtr response);
  void sdkCtrlAuthorityCallback(const 
    dji_sdk::srv::SDKControlAuthority::Request::SharedPtr  request,
    dji_sdk::srv::SDKControlAuthority::Response::SharedPtr response);

  void setLocalPosRefCallback(const 
      dji_sdk::srv::SetLocalPosRef::Request::SharedPtr  request,
      dji_sdk::srv::SetLocalPosRef::Response::SharedPtr response);
  //! control service callbacks
  void droneArmCallback(const dji_sdk::srv::DroneArmControl::Request::SharedPtr  request,
                        dji_sdk::srv::DroneArmControl::Response::SharedPtr response);
  void droneTaskCallback(const dji_sdk::srv::DroneTaskControl::Request::SharedPtr  request,
                         dji_sdk::srv::DroneTaskControl::Response::SharedPtr response);

  //! Mobile Data Service
  void sendToMobileCallback(const dji_sdk::srv::SendMobileData::Request::SharedPtr  request,
                            dji_sdk::srv::SendMobileData::Response::SharedPtr response);
  //! Payload Data Service
  void sendToPayloadCallback(const dji_sdk::srv::SendPayloadData::Request::SharedPtr request,
                             dji_sdk::srv::SendPayloadData::Response::SharedPtr response);
  //! Query Drone FW version
  void queryVersionCallback(const dji_sdk::srv::QueryDroneVersion::Request::SharedPtr request,
                            dji_sdk::srv::QueryDroneVersion::Response::SharedPtr response);

  void cameraActionCallback(const dji_sdk::srv::CameraAction::Request::SharedPtr  request,
                            dji_sdk::srv::CameraAction::Response::SharedPtr response);
  //! mfio service callbacks
  void MFIOConfigCallback(const dji_sdk::srv::MFIOConfig::Request::SharedPtr  request,
                          dji_sdk::srv::MFIOConfig::Response::SharedPtr response);
  void MFIOSetValueCallback(const dji_sdk::srv::MFIOSetValue::Request::SharedPtr  request,
                            dji_sdk::srv::MFIOSetValue::Response::SharedPtr response);
  //! mission service callbacks
  // mission manager
  void missionStatusCallback(const dji_sdk::srv::MissionStatus::Request::SharedPtr  request,
                             dji_sdk::srv::MissionStatus::Response::SharedPtr response);
  // waypoint mission
  void missionWpUploadCallback(const dji_sdk::srv::MissionWpUpload::Request::SharedPtr  request,
                               dji_sdk::srv::MissionWpUpload::Response::SharedPtr response);
  void missionWpActionCallback(const dji_sdk::srv::MissionWpAction::Request::SharedPtr  request,
                               dji_sdk::srv::MissionWpAction::Response::SharedPtr response);
  void missionWpGetInfoCallback(const dji_sdk::srv::MissionWpGetInfo::Request::SharedPtr  request,
                                dji_sdk::srv::MissionWpGetInfo::Response::SharedPtr response);
  void missionWpGetSpeedCallback(const 
    dji_sdk::srv::MissionWpGetSpeed::Request::SharedPtr  request,
    dji_sdk::srv::MissionWpGetSpeed::Response::SharedPtr response);
  void missionWpSetSpeedCallback(const 
    dji_sdk::srv::MissionWpSetSpeed::Request::SharedPtr  request,
    dji_sdk::srv::MissionWpSetSpeed::Response::SharedPtr response);
  // hotpoint mission
  void missionHpUploadCallback(const dji_sdk::srv::MissionHpUpload::Request::SharedPtr  request,
                               dji_sdk::srv::MissionHpUpload::Response::SharedPtr response);
  void missionHpActionCallback(const dji_sdk::srv::MissionHpAction::Request::SharedPtr  request,
                               dji_sdk::srv::MissionHpAction::Response::SharedPtr response);
  void missionHpGetInfoCallback(const dji_sdk::srv::MissionHpGetInfo::Request::SharedPtr  request,
                                dji_sdk::srv::MissionHpGetInfo::Response::SharedPtr response);
  void missionHpUpdateYawRateCallback(const 
    dji_sdk::srv::MissionHpUpdateYawRate::Request::SharedPtr  request,
    dji_sdk::srv::MissionHpUpdateYawRate::Response::SharedPtr response);
  void missionHpResetYawCallback(const 
    dji_sdk::srv::MissionHpResetYaw::Request::SharedPtr  request,
    dji_sdk::srv::MissionHpResetYaw::Response::SharedPtr response);
  void missionHpUpdateRadiusCallback(const 
    dji_sdk::srv::MissionHpUpdateRadius::Request::SharedPtr  request,
    dji_sdk::srv::MissionHpUpdateRadius::Response::SharedPtr response);
  //! hard sync service callback
  void setHardsyncCallback(const dji_sdk::srv::SetHardSync::Request::SharedPtr  request,
                           dji_sdk::srv::SetHardSync::Response::SharedPtr response);

#ifdef ADVANCED_SENSING
  //! stereo image service callback
  void stereo240pSubscriptionCallback(const dji_sdk::srv::Stereo240pSubscription::Request::SharedPtr  request,
                                      dji_sdk::srv::Stereo240pSubscription::Response::SharedPtr response);
  void stereoDepthSubscriptionCallback(const dji_sdk::srv::StereoDepthSubscription::Request::SharedPtr  request,
                                       dji_sdk::srv::StereoDepthSubscription::Response::SharedPtr response);
  void stereoVGASubscriptionCallback(const dji_sdk::srv::StereoVGASubscription::Request::SharedPtr  request,
                                     dji_sdk::srv::StereoVGASubscription::Response::SharedPtr response);
  void setupCameraStreamCallback(const dji_sdk::srv::SetupCameraStream::Request::SharedPtr  request,
                                 dji_sdk::srv::SetupCameraStream::Response::SharedPtr response);
#endif

  //! data broadcast callback
  void dataBroadcastCallback();
  void fromMobileDataCallback(RecvContainer recvFrame);

  void fromPayloadDataCallback(RecvContainer recvFrame);

  static void NMEACallback(Vehicle* vehiclePtr,
                           RecvContainer recvFrame,
                           UserData userData);

  static void GPSUTCTimeCallback(Vehicle *vehiclePtr,
                                 RecvContainer recvFrame,
                                 UserData userData);


  static void FCTimeInUTCCallback(Vehicle* vehiclePtr,
                                  RecvContainer recvFrame,
                                  UserData userData);

  static void PPSSourceCallback(Vehicle* vehiclePtr,
                                RecvContainer recvFrame,
                                UserData userData);

  static void SDKfromMobileDataCallback(Vehicle*            vehicle,
                                        RecvContainer       recvFrame,
                                        DJI::OSDK::UserData userData);

  static void SDKfromPayloadDataCallback(Vehicle *vehicle,
                                        RecvContainer recvFrame,
                                        DJI::OSDK::UserData userData);

  static void SDKBroadcastCallback(Vehicle*            vehicle,
                                   RecvContainer       recvFrame,
                                   DJI::OSDK::UserData userData);

  static void publish5HzData(Vehicle*            vehicle,
                              RecvContainer       recvFrame,
                              DJI::OSDK::UserData userData);

  static void publish50HzData(Vehicle*            vehicle,
                              RecvContainer       recvFrame,
                              DJI::OSDK::UserData userData);

  static void publish100HzData(Vehicle*            vehicle,
                               RecvContainer       recvFrame,
                               DJI::OSDK::UserData userData);

  static void publish400HzData(Vehicle*            vehicle,
                               RecvContainer       recvFrame,
                               DJI::OSDK::UserData userData);

#ifdef ADVANCED_SENSING
  static void publish240pStereoImage(Vehicle*            vehicle,
                                     RecvContainer       recvFrame,
                                     DJI::OSDK::UserData userData);

  static void publishVGAStereoImage(Vehicle*            vehicle,
                                    RecvContainer       recvFrame,
                                    DJI::OSDK::UserData userData);

  static void publishMainCameraImage(CameraRGBImage img, void* userData);

  static void publishFPVCameraImage(CameraRGBImage img, void* userData);
#endif

private:
  //! OSDK core
  Vehicle* vehicle;
  //! general service servers
  rclcpp::Service<dji_sdk::srv::Activation>::SharedPtr drone_activation_server;
  rclcpp::Service<dji_sdk::srv::SDKControlAuthority>::SharedPtr sdk_ctrlAuthority_server;
  rclcpp::Service<dji_sdk::srv::CameraAction>::SharedPtr camera_action_server;
  //! flight control service servers
  rclcpp::Service<dji_sdk::srv::DroneArmControl>::SharedPtr drone_arm_server;
  rclcpp::Service<dji_sdk::srv::DroneTaskControl>::SharedPtr drone_task_server;
  //! mfio service servers
  rclcpp::Service<dji_sdk::srv::MFIOConfig>::SharedPtr mfio_config_server;
  rclcpp::Service<dji_sdk::srv::MFIOSetValue>::SharedPtr mfio_set_value_server;
  //! mission service servers
  // mission manager
  rclcpp::Service<dji_sdk::srv::MissionStatus>::SharedPtr mission_status_server;
  // waypoint mission
  rclcpp::Service<dji_sdk::srv::MissionWpUpload>::SharedPtr waypoint_upload_server;
  rclcpp::Service<dji_sdk::srv::MissionWpAction>::SharedPtr waypoint_action_server;
  rclcpp::Service<dji_sdk::srv::MissionWpGetInfo>::SharedPtr waypoint_getInfo_server;
  rclcpp::Service<dji_sdk::srv::MissionWpGetSpeed>::SharedPtr waypoint_getSpeed_server;
  rclcpp::Service<dji_sdk::srv::MissionWpSetSpeed>::SharedPtr waypoint_setSpeed_server;
  // hotpoint mission
  rclcpp::Service<dji_sdk::srv::MissionHpUpload>::SharedPtr hotpoint_upload_server;
  rclcpp::Service<dji_sdk::srv::MissionHpAction>::SharedPtr hotpoint_action_server;
  rclcpp::Service<dji_sdk::srv::MissionHpGetInfo>::SharedPtr hotpoint_getInfo_server;
  rclcpp::Service<dji_sdk::srv::MissionHpUpdateYawRate>::SharedPtr hotpoint_setSpeed_server;
  rclcpp::Service<dji_sdk::srv::MissionHpResetYaw>::SharedPtr hotpoint_resetYaw_server;
  rclcpp::Service<dji_sdk::srv::MissionHpUpdateRadius>::SharedPtr hotpoint_setRadius_server;
  // send data to mobile device
  rclcpp::Service<dji_sdk::srv::SendMobileData>::SharedPtr send_to_mobile_server;
  // send data to payload device
  rclcpp::Service<dji_sdk::srv::SendPayloadData>::SharedPtr send_to_payload_server;
  //! hardsync service
  rclcpp::Service<dji_sdk::srv::SetHardSync>::SharedPtr set_hardsync_server;
  //! Query FW version of FC
  rclcpp::Service<dji_sdk::srv::QueryDroneVersion>::SharedPtr query_version_server;
  //! Set Local position reference
  rclcpp::Service<dji_sdk::srv::SetLocalPosRef>::SharedPtr local_pos_ref_server;

#ifdef ADVANCED_SENSING
  //! stereo image service
  rclcpp::Service<dji_sdk::srv::Stereo240pSubscription>::SharedPtr subscribe_stereo_240p_server;
  rclcpp::Service<dji_sdk::srv::StereoDepthSubscription>::SharedPtr subscribe_stereo_depth_server;
  rclcpp::Service<dji_sdk::srv::StereoVGASubscription>::SharedPtr subscribe_stereo_vga_server;
  rclcpp::Service<dji_sdk::srv::SetupCameraStream>::SharedPtr camera_stream_server;
#endif

  //! flight control subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_sub;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_position_yaw_sub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_velocity_yawrate_sub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_rollpitch_yawrate_vertpos_sub;

  //! general subscribers
  rclcpp::Subscription<dji_sdk::msg::Gimbal>::SharedPtr gimbal_angle_cmd_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_speed_cmd_subscriber;
  //! telemetry data publisher
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angularRate_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acceleration_publisher;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr trigger_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr flight_status_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gps_health_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_position_publisher;
  rclcpp::Publisher<dji_sdk::msg::VOPosition>::SharedPtr vo_position_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_publisher;
  rclcpp::Publisher<dji_sdk::msg::MobileData>::SharedPtr from_mobile_data_publisher;
  rclcpp::Publisher<dji_sdk::msg::PayloadData>::SharedPtr from_payload_data_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_angle_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr displaymode_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr rc_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rc_connection_status_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_position_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rtk_velocity_publisher;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rtk_yaw_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rtk_position_info_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rtk_yaw_info_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rtk_connection_status_publisher;
  rclcpp::Publisher<dji_sdk::msg::FlightAnomaly>::SharedPtr flight_anomaly_publisher;
  //! Local Position Publisher (Publishes local position in ENU frame)
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_position_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr local_frame_ref_publisher;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr time_sync_nmea_publisher;
  rclcpp::Publisher<dji_sdk::msg::GPSUTC>::SharedPtr time_sync_gps_utc_publisher;
  rclcpp::Publisher<dji_sdk::msg::FCTimeInUTC>::SharedPtr time_sync_fc_utc_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr time_sync_pps_source_publisher;

#ifdef ADVANCED_SENSING
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_front_left_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_front_right_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_down_front_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_down_back_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_front_depth_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_vga_front_left_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_vga_front_right_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr main_camera_stream_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fpv_camera_stream_publisher;
#endif
  //! constant
  const int WAIT_TIMEOUT           = 10;
  const int MAX_SUBSCRIBE_PACKAGES = 5;
  const int INVALID_VERSION        = 0;

  //! configurations
  int         app_id;
  std::string enc_key;
  std::string drone_version;
  std::string serial_device;
  std::string acm_device;
  int         baud_rate;
  int         app_version;
  std::string app_bundle_id; // reserved
  int         uart_or_usb;
  double      gravity_const;

  //! use broadcast or subscription to get telemetry data
  TELEMETRY_TYPE telemetry_from_fc;
  bool stereo_subscription_success;
  bool stereo_vga_subscription_success;
  bool user_select_broadcast;
  const tf2::Matrix3x3 R_FLU2FRD;
  const tf2::Matrix3x3 R_ENU2NED;

  void flightControl(uint8_t flag, float32_t xSP, float32_t ySP, float32_t zSP, float32_t yawSP);

  enum AlignState
  {
    UNALIGNED,
    ALIGNING,
    ALIGNED
  };

  AlignState curr_align_state;

  static int constexpr STABLE_ALIGNMENT_COUNT = 400;
  static double constexpr TIME_DIFF_CHECK = 0.008;
  static double constexpr TIME_DIFF_ALERT = 0.020;

  ros::Time base_time;

  bool align_time_with_FC;

  bool local_pos_ref_set;

  void alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick);
  void setUpM100DefaultFreq(uint8_t freq[16]);
  void setUpA3N3DefaultFreq(uint8_t freq[16]);
  void gpsConvertENU(double &ENU_x, double &ENU_y,
                     double gps_t_lon, double gps_t_lat,
                     double gps_r_lon, double gps_r_lat);

  double local_pos_ref_latitude, local_pos_ref_longitude, local_pos_ref_altitude;
  double current_gps_latitude, current_gps_longitude, current_gps_altitude;
  int current_gps_health;
  bool rtkSupport;
};

#endif // DJI_SDK_NODE_MAIN_H
