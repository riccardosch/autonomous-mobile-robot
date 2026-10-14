// Wheel firmware for the ESP32 DevKit, micro-ROS build (env microros).
//
// One node, ROS_NODE_NAME, over the USB serial at MICROROS_BAUD:
//   out  TOPIC_WHEEL_STATES  sensor_msgs/JointState, position (rad) and velocity (rad/s)
//   out  TOPIC_IMU           sensor_msgs/Imu, angular velocity and linear acceleration,
//                            orientation not provided (orientation_covariance[0] = -1)
//   in   TOPIC_WHEEL_CMD     sensor_msgs/JointState, velocity targets (rad/s)
// Both outputs at 1 / PUBLISH_PERIOD_S, best effort. The control task, motors,
// encoders and IMU are the same as in the debug build. Nothing else may be
// written to Serial: it carries the micro-ROS protocol.
//
// The node waits for the micro-ROS agent, creates its entities, and if the agent
// stops answering destroys them and waits again. Without commands the
// watchdog stops the wheels.

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

#include <cstring>

#include "config.h"
#include "control.h"
#include "encoders.h"
#include "imu.h"
#include "motors.h"

namespace {

enum class Agent { WAITING, AVAILABLE, CONNECTED, LOST };

constexpr uint32_t PUBLISH_PERIOD_MS = static_cast<uint32_t>(PUBLISH_PERIOD_S * 1000.0f + 0.5f);

Agent agent = Agent::WAITING;
uint32_t last_ping_ms = 0;
uint32_t last_sync_ms = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t wheel_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t cmd_sub;
rcl_timer_t timer;
rclc_executor_t executor;

// Messages with static storage for their sequences and strings.
sensor_msgs__msg__JointState wheel_msg;
double wheel_position[WHEEL_COUNT];
double wheel_velocity[WHEEL_COUNT];
rosidl_runtime_c__String wheel_names[WHEEL_COUNT];

sensor_msgs__msg__Imu imu_msg;

sensor_msgs__msg__JointState cmd_msg;
char cmd_name_buf[WHEEL_COUNT][JOINT_NAME_MAX];
rosidl_runtime_c__String cmd_names[WHEEL_COUNT];
double cmd_position[WHEEL_COUNT];
double cmd_velocity[WHEEL_COUNT];
double cmd_effort[WHEEL_COUNT];

// For return codes that cannot be acted on: a best-effort publish that fails
// is simply lost, and teardown runs after the agent is already gone.
void ignore(rcl_ret_t) {}

void set_string(rosidl_runtime_c__String &s, const char *text) {
  s.data = const_cast<char *>(text);
  s.size = strlen(text);
  s.capacity = s.size + 1;
}

template <typename Seq, typename T>
void set_sequence(Seq &seq, T *data, size_t size, size_t capacity) {
  seq.data = data;
  seq.size = size;
  seq.capacity = capacity;
}

void init_messages() {
  memset(&wheel_msg, 0, sizeof(wheel_msg));
  for (int w = 0; w < WHEEL_COUNT; w++) set_string(wheel_names[w], JOINT_NAMES[w]);
  set_sequence(wheel_msg.name, wheel_names, WHEEL_COUNT, WHEEL_COUNT);
  set_sequence(wheel_msg.position, wheel_position, WHEEL_COUNT, WHEEL_COUNT);
  set_sequence(wheel_msg.velocity, wheel_velocity, WHEEL_COUNT, WHEEL_COUNT);

  memset(&imu_msg, 0, sizeof(imu_msg));
  set_string(imu_msg.header.frame_id, IMU_FRAME_ID);
  imu_msg.orientation_covariance[0] = -1.0;  // orientation not provided
  for (int i = 0; i < 3; i++) {
    imu_msg.angular_velocity_covariance[4 * i] = sq(IMU_GYRO_STD_RAD_S);
    imu_msg.linear_acceleration_covariance[4 * i] = sq(IMU_ACCEL_STD_M_S2);
  }

  // Received messages are written into this preallocated storage.
  memset(&cmd_msg, 0, sizeof(cmd_msg));
  for (int w = 0; w < WHEEL_COUNT; w++) {
    cmd_names[w] = {cmd_name_buf[w], 0, JOINT_NAME_MAX};
  }
  set_sequence(cmd_msg.name, cmd_names, 0, WHEEL_COUNT);
  set_sequence(cmd_msg.position, cmd_position, 0, WHEEL_COUNT);
  set_sequence(cmd_msg.velocity, cmd_velocity, 0, WHEEL_COUNT);
  set_sequence(cmd_msg.effort, cmd_effort, 0, WHEEL_COUNT);
}

void stamp(builtin_interfaces__msg__Time &t) {
  const int64_t ns = rmw_uros_epoch_nanos();
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

void publish_cb(rcl_timer_t *, int64_t) {
  WheelState st[WHEEL_COUNT];
  control_get_state(st);
  stamp(wheel_msg.header.stamp);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    wheel_position[w] = st[w].position;
    wheel_velocity[w] = st[w].velocity;
  }
  ignore(rcl_publish(&wheel_pub, &wheel_msg, nullptr));

  ImuState imu;
  control_get_imu(imu);
  if (!imu.ok || imu.calibration != ImuCal::DONE) return;
  imu_msg.header.stamp = wheel_msg.header.stamp;
  imu_msg.angular_velocity.x = imu.sample.gyro[0];
  imu_msg.angular_velocity.y = imu.sample.gyro[1];
  imu_msg.angular_velocity.z = imu.sample.gyro[2];
  imu_msg.linear_acceleration.x = imu.sample.accel[0];
  imu_msg.linear_acceleration.y = imu.sample.accel[1];
  imu_msg.linear_acceleration.z = imu.sample.accel[2];
  ignore(rcl_publish(&imu_pub, &imu_msg, nullptr));
}

// Velocity targets by joint name; with no names, in the order left, right.
void cmd_cb(const void *msg_in) {
  const auto *msg = static_cast<const sensor_msgs__msg__JointState *>(msg_in);
  if (msg->velocity.size < WHEEL_COUNT) return;
  float target[WHEEL_COUNT];
  for (int w = 0; w < WHEEL_COUNT; w++) {
    size_t index = w;
    if (msg->name.size == msg->velocity.size) {
      index = msg->velocity.size;  // not found
      for (size_t i = 0; i < msg->name.size; i++) {
        if (strcmp(msg->name.data[i].data, JOINT_NAMES[w]) == 0) index = i;
      }
      if (index == msg->velocity.size) return;
    }
    target[w] = static_cast<float>(msg->velocity.data[index]);
  }
  control_set_target(target[WHEEL_LEFT], target[WHEEL_RIGHT]);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  bool ok = rclc_node_init_default(&node, ROS_NODE_NAME, "", &support) == RCL_RET_OK &&
            rclc_publisher_init_best_effort(&wheel_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), TOPIC_WHEEL_STATES) == RCL_RET_OK &&
            rclc_publisher_init_best_effort(&imu_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), TOPIC_IMU) == RCL_RET_OK &&
            rclc_subscription_init_best_effort(&cmd_sub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), TOPIC_WHEEL_CMD) == RCL_RET_OK &&
            rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), publish_cb, true) == RCL_RET_OK;
  executor = rclc_executor_get_zero_initialized_executor();
  ok = ok && rclc_executor_init(&executor, &support.context, 2, &allocator) == RCL_RET_OK &&
       rclc_executor_add_timer(&executor, &timer) == RCL_RET_OK &&
       rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, cmd_cb, ON_NEW_DATA) == RCL_RET_OK;
  rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
  last_sync_ms = millis();
  return ok;
}

void destroy_entities() {
  rmw_context_t *context = rcl_context_get_rmw_context(&support.context);
  ignore(rmw_uros_set_context_entity_destroy_session_timeout(context, 0));
  ignore(rcl_publisher_fini(&wheel_pub, &node));
  ignore(rcl_publisher_fini(&imu_pub, &node));
  ignore(rcl_subscription_fini(&cmd_sub, &node));
  ignore(rcl_timer_fini(&timer));
  ignore(rclc_executor_fini(&executor));
  ignore(rcl_node_fini(&node));
  ignore(rclc_support_fini(&support));
}

bool ping_due() {
  if (millis() - last_ping_ms < AGENT_PING_PERIOD_MS) return false;
  last_ping_ms = millis();
  return true;
}

bool agent_answers() { return rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK; }

}  // namespace

void setup() {
  motors_begin();  // first, so the motors are off as soon as possible
  Serial.setRxBufferSize(MICROROS_SERIAL_BUFFER_BYTES);
  Serial.setTxBufferSize(MICROROS_SERIAL_BUFFER_BYTES);
  Serial.begin(MICROROS_BAUD);
  set_microros_serial_transports(Serial);
  encoders_begin();
  imu_begin();  // starts the gyro calibration: keep the robot still at power-on
  control_begin();
  init_messages();
}

void loop() {
  switch (agent) {
    case Agent::WAITING:
      if (ping_due() && agent_answers()) agent = Agent::AVAILABLE;
      break;
    case Agent::AVAILABLE:
      if (create_entities()) {
        agent = Agent::CONNECTED;
      } else {
        destroy_entities();
        agent = Agent::WAITING;
      }
      break;
    case Agent::CONNECTED:
      if (ping_due() && !agent_answers()) {
        agent = Agent::LOST;
        break;
      }
      if (millis() - last_sync_ms > TIME_SYNC_PERIOD_MS) {
        rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        last_sync_ms = millis();
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_MS));
      break;
    case Agent::LOST:
      destroy_entities();
      control_set_target(0.0f, 0.0f);  // do not wait for the watchdog
      agent = Agent::WAITING;
      break;
  }
}
