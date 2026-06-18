/** @file dji_sdk_node.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the initialization functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

using namespace DJI::OSDK;
using namespace std::placeholders;

DJISDKNode::DJISDKNode()
  : Node("dji_sdk"),
    telemetry_from_fc(USE_BROADCAST),
    R_FLU2FRD(tf::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
    R_ENU2NED(tf::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1)),
    curr_align_state(UNALIGNED)
{
  this->declare_parameter("acm_name", std::string("/dev/ttyACM0"));
  acm_device = this->get_parameter("my_parameter").as_string();
  this->declare_parameter("serial_name", std::string("/dev/ttyUSB0"));
  serial_device = this->get_parameter("serial_name").as_string();
  this->declare_parameter("baud_rate", 921600);
  baud_rate = this->get_parameter("baud_rate").as_int();
  this->declare_parameter("app_id", 123456);
  app_id = this->get_parameter("app_id").as_int();
  this->declare_parameter("app_version", 1);
  app_version = this->get_parameter("app_version").as_int();
  this->declare_parameter("enc_key", std::string("abcd1234"));
  enc_key = this->get_parameter("enc_key").as_string();
  this->declare_parameter("drone_version", std::string("M100")); // choose M100 as default
  drone_version = this->get_parameter("drone_version").as_string();
  this->declare_parameter("gravity_const", 9.801);
  gravity_const = this->get_parameter("gravity_const").as_double();
  this->declare_parameter("align_time", false);
  align_time_with_FC = this->get_parameter("align_time").as_bool();
  this->declare_parameter("use_broadcast", false);
  user_select_broadcast = this->get_parameter("use_broadcast").as_bool();

  //! Default values for local Position
  local_pos_ref_latitude  = 0;
  local_pos_ref_longitude = 0;
  local_pos_ref_altitude  = 0;
  local_pos_ref_set       = false;

  //! RTK support check
  rtkSupport = false;

  // @todo need some error handling for init functions
  //! @note parsing launch file to get environment parameters
  if (!initVehicle())
  {
    RCLCPP_ERROR(this->get_logger(), "Vehicle initialization failed");
  }

  else
  {
    if (!initServices())
    {
      RCLCPP_ERROR(this->get_logger(), "initServices failed");
    }

    if (!initFlightControl())
    {
      RCLCPP_ERROR(this->get_logger(), "initFlightControl failed");
    }

    if (!initSubscriber())
    {
      RCLCPP_ERROR(this->get_logger(), "initSubscriber failed");
    }

    if (!initPublisher())
    {
      RCLCPP_ERROR(this->get_logger(), "initPublisher failed");
    }
  }
}

DJISDKNode::~DJISDKNode()
{
  if(!isM100())
  {
    cleanUpSubscribeFromFC();
  }
  if (vehicle)
  {
    delete vehicle;
  }
}

bool
DJISDKNode::initVehicle()
{
  bool threadSupport = true;
  bool enable_advanced_sensing = false;

#ifdef ADVANCED_SENSING
  enable_advanced_sensing = true;
  RCLCPP_INFO(this->get_logger(), "Advanced Sensing is Enabled on M210.");
#endif

  //! @note currently does not work without thread support
  vehicle = new Vehicle(serial_device.c_str(), baud_rate, threadSupport, enable_advanced_sensing);

  /*!
   * @note activate the drone for the user at the beginning
   *        user can also call it as a service
   *        this has been tested by giving wrong appID in launch file
   */
  if (ACK::getError(this->activate(this->app_id, this->enc_key)))
  {
    RCLCPP_ERROR(this->get_logger(), "drone activation error");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "drone activated");

  // This version of ROS Node works for:
  //    1. A3/N3/M600 with latest FW
  //    2. M100 with FW version M100_31
  if(vehicle->getFwVersion() > INVALID_VERSION
      && vehicle->getFwVersion() < mandatoryVersionBase
      && (!isM100()))
  {
    return false;
  }


  if (NULL != vehicle->subscribe && (!user_select_broadcast))
  {
    telemetry_from_fc = USE_SUBSCRIBE;
  }

#ifdef ADVANCED_SENSING
  vehicle->advancedSensing->setAcmDevicePath(acm_device.c_str());
#endif

  return true;
}

// clang-format off
bool DJISDKNode::initServices() {
  // Common to A3/N3 and M100
  drone_activation_server   = this->create_service<dji_sdk::srv::Activation>("dji_sdk/activation",                     std::bind(&DJISDKNode::droneActivationCallback, this, _1, _2));
  drone_arm_server          = this->create_service<dji_sdk::srv::DroneArmControl>("dji_sdk/drone_arm_control",              std::bind(&DJISDKNode::droneArmCallback, this, _1, _2));
  drone_task_server         = this->create_service<dji_sdk::srv::DroneTaskControl>("dji_sdk/drone_task_control",             std::bind(&DJISDKNode::droneTaskCallback, this, _1, _2));
  sdk_ctrlAuthority_server  = this->create_service<dji_sdk::srv::SDKControlAuthority>("dji_sdk/sdk_control_authority",          std::bind(&DJISDKNode::sdkCtrlAuthorityCallback, this, _1, _2));
  camera_action_server      = this->create_service<dji_sdk::srv::CameraAction>("dji_sdk/camera_action",                  std::bind(&DJISDKNode::cameraActionCallback, this, _1, _2));
  waypoint_upload_server    = this->create_service<dji_sdk::srv::MissionWpUpload>("dji_sdk/mission_waypoint_upload",        std::bind(&DJISDKNode::missionWpUploadCallback, this, _1, _2));
  waypoint_action_server    = this->create_service<dji_sdk::srv::MissionWpAction>("dji_sdk/mission_waypoint_action",        std::bind(&DJISDKNode::missionWpActionCallback, this, _1, _2));
  waypoint_getInfo_server   = this->create_service<dji_sdk::srv::MissionWpGetInfo>("dji_sdk/mission_waypoint_getInfo",       std::bind(&DJISDKNode::missionWpGetInfoCallback, this, _1, _2));
  waypoint_getSpeed_server  = this->create_service<dji_sdk::srv::MissionWpGetSpeed>("dji_sdk/mission_waypoint_getSpeed",      std::bind(&DJISDKNode::missionWpGetSpeedCallback, this, _1, _2));
  waypoint_setSpeed_server  = this->create_service<dji_sdk::srv::MissionWpSetSpeed>("dji_sdk/mission_waypoint_setSpeed",      std::bind(&DJISDKNode::missionWpSetSpeedCallback, this, _1, _2));
  hotpoint_upload_server    = this->create_service<dji_sdk::srv::MissionHpUpload>("dji_sdk/mission_hotpoint_upload",        std::bind(&DJISDKNode::missionHpUploadCallback, this, _1, _2));
  hotpoint_action_server    = this->create_service<dji_sdk::srv::MissionHpAction>("dji_sdk/mission_hotpoint_action",        std::bind(&DJISDKNode::missionHpActionCallback, this, _1, _2));
  hotpoint_getInfo_server   = this->create_service<dji_sdk::srv::MissionHpGetInfo>("dji_sdk/mission_hotpoint_getInfo",       std::bind(&DJISDKNode::missionHpGetInfoCallback, this, _1, _2));
  hotpoint_setSpeed_server  = this->create_service<dji_sdk::srv::MissionHpUpdateYawRate>("dji_sdk/mission_hotpoint_updateYawRate", std::bind(&DJISDKNode::missionHpUpdateYawRateCallback, this, _1, _2));
  hotpoint_resetYaw_server  = this->create_service<dji_sdk::srv::MissionHpResetYaw>("dji_sdk/mission_hotpoint_resetYaw",      std::bind(&DJISDKNode::missionHpResetYawCallback, this, _1, _2));
  hotpoint_setRadius_server = this->create_service<dji_sdk::srv::MissionHpUpdateRadius>("dji_sdk/mission_hotpoint_updateRadius",  std::bind(&DJISDKNode::missionHpUpdateRadiusCallback, this, _1, _2));
  mission_status_server     = this->create_service<dji_sdk::srv::MissionStatus>("dji_sdk/mission_status",                 std::bind(&DJISDKNode::missionStatusCallback, this, _1, _2));
  send_to_mobile_server     = this->create_service<dji_sdk::srv::SendMobileData>("dji_sdk/send_data_to_mobile",            std::bind(&DJISDKNode::sendToMobileCallback, this, _1, _2));
  send_to_payload_server    = this->create_service<dji_sdk::srv::SendPayloadData>("dji_sdk/send_data_to_payload",           std::bind(&DJISDKNode::sendToPayloadCallback, this, _1, _2));
  query_version_server      = this->create_service<dji_sdk::srv::QueryDroneVersion>("dji_sdk/query_drone_version",            std::bind(&DJISDKNode::queryVersionCallback, this, _1, _2));
  local_pos_ref_server      = this->create_service<dji_sdk::srv::SetLocalPosRef>("dji_sdk/set_local_pos_ref",              std::bind(&DJISDKNode::setLocalPosRefCallback, this, _1, _2));
#ifdef ADVANCED_SENSING
  subscribe_stereo_240p_server  = this->create_service<dji_sdk::srv::Stereo240pSubscription>("dji_sdk/stereo_240p_subscription",   std::bind(&DJISDKNode::stereo240pSubscriptionCallback, this, _1, _2));
  subscribe_stereo_depth_server = this->create_service<dji_sdk::srv::StereoDepthSubscription>("dji_sdk/stereo_depth_subscription",  std::bind(&DJISDKNode::stereoDepthSubscriptionCallback, this, _1, _2));
  camera_stream_server          = this->create_service<dji_sdk::srv::SetupCameraStream>("dji_sdk/setup_camera_stream",        std::bind(&DJISDKNode::setupCameraStreamCallback, this, _1, _2));
#endif

  // A3/N3 only
  if(!isM100())
  {
    set_hardsync_server   = this->create_service<dji_sdk::srv::SetHardSync>("dji_sdk/set_hardsyc", std::bind(&DJISDKNode::setHardsyncCallback, this, _1, _2));
    mfio_config_server    = this->create_service<dji_sdk::srv::MFIOConfig>("dji_sdk/mfio_config", std::bind(&DJISDKNode::MFIOConfigCallback, this, _1, _2));
    mfio_set_value_server = this->create_service<dji_sdk::srv::MFIOSetValue>("dji_sdk/mfio_set_value", std::bind(&DJISDKNode::MFIOSetValueCallback, this, _1, _2));
  }
  return true;
}
// clang-format on

bool
DJISDKNode::initFlightControl()
{
  flight_control_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "dji_sdk/flight_control_setpoint_generic", 10, 
    std::bind(&DJISDKNode::flightControlSetpointCallback, this, _1));

  flight_control_position_yaw_sub =
    this->create_subscription<sensor_msgs::msg::Joy>(
      "dji_sdk/flight_control_setpoint_ENUposition_yaw", 10,
      std::bind(&DJISDKNode::flightControlPxPyPzYawCallback, this, _1));

  flight_control_velocity_yawrate_sub =
    this->create_subscription<sensor_msgs::msg::Joy>(
      "dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10,
      std::bind(&DJISDKNode::flightControlVxVyVzYawrateCallback, this, _1));

  flight_control_rollpitch_yawrate_vertpos_sub =
    this->create_subscription<sensor_msgs::msg::Joy>(
      "dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10,
      std::bind(&DJISDKNode::flightControlRollPitchPzYawrateCallback, this, _1));

  return true;
}

bool DJISDKNode::isM100()
{
  return(vehicle->isM100());
}


ACK::ErrorCode
DJISDKNode::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char                  app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  RCLCPP_DEBUG(this->get_logger(), "called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool
DJISDKNode::initSubscriber()
{
  gimbal_angle_cmd_subscriber = this->create_subscription<dji_sdk::msg::Gimbal>(
    "dji_sdk/gimbal_angle_cmd", 10, std::bind(&DJISDKNode::gimbalAngleCtrlCallback, this, _1));
  gimbal_speed_cmd_subscriber = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "dji_sdk/gimbal_speed_cmd", 10, std::bind(&DJISDKNode::gimbalSpeedCtrlCallback, this, _1));
  return true;
}

bool
DJISDKNode::initPublisher()
{
  rc_publisher = this->create_publisher<sensor_msgs::msg::Joy>("dji_sdk/rc", 10);

  attitude_publisher =
    this->create_publisher<geometry_msgs::msg::QuaternionStamped>("dji_sdk/attitude", 10);

  battery_state_publisher =
    this->create_publisher<sensor_msgs::msg::BatteryState>("dji_sdk/battery_state",10);

  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s^2)
   */
  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("dji_sdk/imu", 10);

  // Refer to dji_sdk.h for different enums for M100 and A3/N3
  flight_status_publisher =
    this->create_publisher<std_msgs::msg::UInt8>("dji_sdk/flight_status", 10);

  /*!
   * gps_health needs to be greater than 3 for gps_position and velocity topics
   * to be trusted
   */
  gps_health_publisher =
    this->create_publisher<std_msgs::msg::UInt8>("dji_sdk/gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher =
    this->create_publisher<sensor_msgs::msg::NavSatFix>("dji_sdk/gps_position", 10);

  /*!
   *   x [m]. Positive along navigation frame x axis
   *   y [m]. Positive along navigation frame y axis
   *   z [m]. Positive is down
   *   For details about navigation frame, please see telemetry documentation in API reference
  */
  vo_position_publisher =
          this->create_publisher<dji_sdk::msg::VOPosition>("dji_sdk/vo_position", 10);
  /*!
   * Height above home altitude. It is valid only after drone
   * is armed.
   */
  height_publisher =
    this->create_publisher<std_msgs::msg::Float32>("dji_sdk/height_above_takeoff", 10);

  velocity_publisher =
    this->create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/velocity", 10);

  from_mobile_data_publisher =
    this->create_publisher<dji_sdk::msg::MobileData>("dji_sdk/from_mobile_data", 10);

  from_payload_data_publisher =
    this->create_publisher<dji_sdk::msg::PayloadData>("dji_sdk/from_payload_data", 10);

  // TODO: documentation and proper frame id
  gimbal_angle_publisher =
    this->create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/gimbal_angle", 10);

  local_position_publisher =
      this->create_publisher<geometry_msgs::msg::PointStamped>("dji_sdk/local_position", 10);

  local_frame_ref_publisher =
      this->create_publisher<sensor_msgs::msg::NavSatFix>("dji_sdk/local_frame_ref", rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .transient_local());

  time_sync_nmea_publisher =
      this->create_publisher<nmea_msgs::msg::Sentence>("dji_sdk/time_sync_nmea_msg", 10);

  time_sync_gps_utc_publisher =
      this->create_publisher<dji_sdk::msg::GPSUTC>("dji_sdk/time_sync_gps_utc", 10);

  time_sync_fc_utc_publisher =
      this->create_publisher<dji_sdk::msg::FCTimeInUTC>("dji_sdk/time_sync_fc_time_utc", 10);

  time_sync_pps_source_publisher =
      this->create_publisher<std_msgs::msg::String>("dji_sdk/time_sync_pps_source", 10);

#ifdef ADVANCED_SENSING
  stereo_240p_front_left_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_front_left_images", 10);

  stereo_240p_front_right_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_front_right_images", 10);

  stereo_240p_down_front_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_down_front_images", 10);

  stereo_240p_down_back_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_down_back_images", 10);

  stereo_240p_front_depth_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_front_depth_images", 10);

  stereo_vga_front_left_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_vga_front_left_images", 10);

  stereo_vga_front_right_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_vga_front_right_images", 10);

  main_camera_stream_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/main_camera_images", 10);

  fpv_camera_stream_publisher =
    this->create_publisher<sensor_msgs::msg::Image>("dji_sdk/fpv_camera_images", 10);
#endif



  if (telemetry_from_fc == USE_BROADCAST)
  {
    ACK::ErrorCode broadcast_set_freq_ack;
    RCLCPP_INFO(this->get_logger(), "Use legacy data broadcast to get telemetry data!");

    uint8_t defaultFreq[16];

    if(isM100())
    {
      setUpM100DefaultFreq(defaultFreq);
    }
    else
    {
      setUpA3N3DefaultFreq(defaultFreq);
    }

    broadcast_set_freq_ack =
      vehicle->broadcast->setBroadcastFreq(defaultFreq, WAIT_TIMEOUT);
//      vehicle->broadcast->setBroadcastFreqDefaults(WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack))
    {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    vehicle->broadcast->setUserBroadcastCallback(
      &DJISDKNode::SDKBroadcastCallback, this);
  }
  else if (telemetry_from_fc == USE_SUBSCRIBE)
  {
    RCLCPP_INFO(this->get_logger(), "Use data subscription to get telemetry data!");
    if(!align_time_with_FC)
    {
      RCLCPP_INFO(this->get_logger(), "align_time_with_FC set to false. We will use ros time to time stamp messages!");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "align_time_with_FC set to true. We will time stamp messages based on flight controller time!");
    }

    // Extra topics that is only available from subscription

    // Details can be found in DisplayMode enum in dji_sdk.h
    displaymode_publisher =
      this->create_publisher<std_msgs::msg::UInt8>("dji_sdk/display_mode", 10);

    angularRate_publisher =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/angular_velocity_fused", 10);

    acceleration_publisher =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/acceleration_ground_fused", 10);

    trigger_publisher = this->create_publisher<sensor_msgs::msg::TimeReference>("dji_sdk/trigger_time", 10);

    if (!initDataSubscribeFromFC())
    {
      return false;
    }
  }
  vehicle->moc->setFromMSDKCallback(&DJISDKNode::SDKfromMobileDataCallback,
                                    this);
  if (vehicle->payloadDevice)
  {
    vehicle->payloadDevice->setFromPSDKCallback(&DJISDKNode::SDKfromPayloadDataCallback, this);
  }

  if (vehicle->hardSync)
  {
    vehicle->hardSync->subscribeNMEAMsgs(NMEACallback, this);
    vehicle->hardSync->subscribeUTCTime(GPSUTCTimeCallback, this);
    vehicle->hardSync->subscribeFCTimeInUTCRef(FCTimeInUTCCallback, this);
    vehicle->hardSync->subscribePPSSource(PPSSourceCallback, this);
  }
  return true;
}

bool
DJISDKNode::initDataSubscribeFromFC()
{
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  std::vector<Telemetry::TopicName> topicList100Hz;
  topicList100Hz.push_back(Telemetry::TOPIC_QUATERNION);
  topicList100Hz.push_back(Telemetry::TOPIC_ACCELERATION_GROUND);
  topicList100Hz.push_back(Telemetry::TOPIC_ANGULAR_RATE_FUSIONED);

  int nTopic100Hz    = topicList100Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_100HZ, nTopic100Hz,
                                                   topicList100Hz.data(), 1, 100))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR(this->get_logger(), "Failed to start 100Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_100HZ, publish100HzData, this);
    }
  }

  std::vector<Telemetry::TopicName> topicList50Hz;
  // 50 Hz package from FC
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
  topicList50Hz.push_back(Telemetry::TOPIC_ALTITUDE_FUSIONED);
  topicList50Hz.push_back(Telemetry::TOPIC_HEIGHT_FUSION);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_DISPLAYMODE);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_ANGLES);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_STATUS);
  topicList50Hz.push_back(Telemetry::TOPIC_RC);
  topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_CONTROL_LEVEL);

  if(vehicle->getFwVersion() > versionBase33)
  {
    topicList50Hz.push_back(Telemetry::TOPIC_POSITION_VO);
    topicList50Hz.push_back(Telemetry::TOPIC_RC_WITH_FLAG_DATA);
    topicList50Hz.push_back(Telemetry::TOPIC_FLIGHT_ANOMALY);

    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(vehicle->getHwVersion());
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
    {
      topicList50Hz.push_back(Telemetry::TOPIC_RC_FULL_RAW_DATA);      
    }

    // Advertise rc connection status only if this topic is supported by FW
    rc_connection_status_publisher =
            this->create_publisher<std_msgs::msg::UInt8>("dji_sdk/rc_connection_status", 10);

    flight_anomaly_publisher =
            this->create_publisher<dji_sdk::msg::FlightAnomaly>("dji_sdk/flight_anomaly", 10);
  }

  int nTopic50Hz    = topicList50Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_50HZ, nTopic50Hz,
                                                   topicList50Hz.data(), 1, 50))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR(this->get_logger(), "Failed to start 50Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_50HZ, publish50HzData, (UserData) this);
    }
  }

  //! Check if RTK is supported in the FC
  Telemetry::TopicName topicRTKSupport[] =
  {
    Telemetry::TOPIC_RTK_POSITION
  };

  int nTopicRTKSupport    = sizeof(topicRTKSupport)/sizeof(topicRTKSupport[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_5HZ, nTopicRTKSupport,
                                                   topicRTKSupport, 1, 5))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    if (ack.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      rtkSupport = false;
      RCLCPP_INFO(this->get_logger(), "Flight Controller does not support RTK");
    }
    else
    {
      rtkSupport = true;
      vehicle->subscribe->removePackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    }
  }

  std::vector<Telemetry::TopicName> topicList5hz;
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DATE);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_TIME);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_POSITION);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_VELOCITY);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DETAILS);
  topicList5hz.push_back(Telemetry::TOPIC_BATTERY_INFO);

  if(rtkSupport)
  {
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_VELOCITY);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW_INFO);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION_INFO);

    // Advertise rtk data only when rtk is supported
    rtk_position_publisher =
            this->create_publisher<sensor_msgs::msg::NavSatFix>("dji_sdk/rtk_position", 10);

    rtk_velocity_publisher =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/rtk_velocity", 10);

    rtk_yaw_publisher =
            this->create_publisher<std_msgs::msg::Int16>("dji_sdk/rtk_yaw", 10);

    rtk_position_info_publisher =
            this->create_publisher<std_msgs::msg::UInt8>("dji_sdk/rtk_info_position", 10);

    rtk_yaw_info_publisher =
            this->create_publisher<std_msgs::msg::UInt8>("dji_sdk/rtk_info_yaw", 10);

    if(vehicle->getFwVersion() > versionBase33)
    {
      topicList5hz.push_back(Telemetry::TOPIC_RTK_CONNECT_STATUS);

      // Advertise rtk connection only when rtk is supported
      rtk_connection_status_publisher =
              this->create_publisher<std_msgs::msg::UInt8>("dji_sdk/rtk_connection_status", 10);
    }
  }

  int nTopic5hz    = topicList5hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_5HZ, nTopic5hz,
                                                   topicList5hz.data(), 1, 5))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR(this->get_logger(), "Failed to start 5hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_5HZ, publish5HzData, (UserData) this);
    }
  }

  // 400 Hz data from FC
  std::vector<Telemetry::TopicName> topicList400Hz;
  topicList400Hz.push_back(Telemetry::TOPIC_HARD_SYNC);

  int nTopic400Hz = topicList400Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_400HZ, nTopic400Hz,
                                                   topicList400Hz.data(), 1, 400))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
    if(ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR(this->get_logger(), "Failed to start 400Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(PACKAGE_ID_400HZ, publish400HzData, this);
    }
  }

  ros::Duration(1).sleep();
  return true;
}

void
DJISDKNode::cleanUpSubscribeFromFC()
{
  vehicle->subscribe->removePackage(0, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(1, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(2, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(3, WAIT_TIMEOUT);
  if (vehicle->hardSync)
  {
    vehicle->hardSync->unsubscribeNMEAMsgs();
    vehicle->hardSync->unsubscribeUTCTime();
    vehicle->hardSync->unsubscribeFCTimeInUTCRef();
    vehicle->hardSync->unsubscribePPSSource();
  }
}

bool DJISDKNode::validateSerialDevice(LinuxSerialDevice* serialDevice)
{
  static const int BUFFER_SIZE = 2048;
  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  if (!serialDevice->setSerialPureTimedRead())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set up port for timed read.\n");
    return (false);
  };
  usleep(100000);
  if(serialDevice->serialRead(buf, BUFFER_SIZE))
  {
    RCLCPP_INFO(this->get_logger(), "Succeeded to read from serial device");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to read from serial device. The Onboard SDK is not communicating with your drone.");
    return (false);
  }

  // All the tests passed and the serial device is properly set up
  serialDevice->unsetSerialPureTimedRead();
  return (true);
}

void
DJISDKNode::setUpM100DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_10HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_50HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_10HZ;
  freq[11] = DataBroadcast::FREQ_10HZ;
}

void
DJISDKNode::setUpA3N3DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_50HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_10HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_50HZ;
  freq[11] = DataBroadcast::FREQ_50HZ;
  freq[12] = DataBroadcast::FREQ_10HZ;
  freq[13] = DataBroadcast::FREQ_10HZ;
}

void DJISDKNode::gpsConvertENU(double &ENU_x, double &ENU_y,
                                 double gps_t_lon, double gps_t_lat,
                                 double gps_r_lon, double gps_r_lat)
{
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  ENU_y = DEG2RAD(d_lat) * C_EARTH;
  ENU_x = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
};
