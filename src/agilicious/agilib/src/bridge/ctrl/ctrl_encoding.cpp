#include "agilib/bridge/ctrl/ctrl_encoding.hpp"

#include <cstring>

#include "agilib/bridge/ctrl/ctrl_msgs_defs.hpp"
#include "agilib/serial/cobs.hpp"

namespace agi {

CtrlEncoding::CtrlEncoding(const Quadrotor& quad, const ThrustMap& thrust_map,
                           const ctrl::CTRLMODE mode,
                           bool command_throttle_direct,
                           const Scalar thrust_coeff)
  : quad_(quad),
    thrust_map_(thrust_map),
    mode_(mode),
    command_throttle_direct_(command_throttle_direct),
    thrust_coeff_(thrust_coeff),
    voltage_(NAN) {}

bool CtrlEncoding::encode(const Command& command, const bool armed,
                          char* const buffer, int* const length) const {
  if (buffer == nullptr || *length <= (int)ctrl::SERIAL_CONTAINER_MAX_LENGTH)
    return false;

  if (time_ns_init_ == 0) time_ns_init_ = (uint64_t)(1e9 * command.t);
  const uint64_t time_ns = (uint64_t)(1e9 * command.t) - time_ns_init_;

  ctrl::serial_container_t container;
  container.num_messages = 1u;
  ctrl::serial_message_t& msg = container.messages[0];
  msg.time = time_ns;

  // If it should not be armed, send off message.
  if (!armed) {
    msg.mode = ctrl::CTRLMODE::OFF;
  } else {
    // well do it without mutex locking
    const Feedback current_feedback = feedback_;

    // We should be armed, so lets see if we are armed.
    if (current_feedback.control_mode >= Feedback::CTRLMODE::ARM) {
      // Ok, we are armed... so lets send the command.

      if (command_throttle_direct_) {
        // use the command thrusts directly as the throttle values - i.e. for
        // thrust mapping
        msg.mode = ctrl::CTRLMODE::ROTOR_THROTTLE;
        ctrl::rotor_throttle_payload_t throttle_payload;

        throttle_payload.throttle[0] = (int16_t)command.thrusts(0);
        throttle_payload.throttle[1] = (int16_t)command.thrusts(1);
        throttle_payload.throttle[2] = (int16_t)command.thrusts(2);
        throttle_payload.throttle[3] = (int16_t)command.thrusts(3);

        memcpy(msg.payload, &throttle_payload,
               getPayloadLength(ctrl::CTRLMODE::ROTOR_THROTTLE));
      } else {
        // use the command thrusts as force

        switch (mode_) {
          case ctrl::CTRLMODE::ROTOR_THRUST: {
            // comand in thrusts
            msg.mode = ctrl::CTRLMODE::ROTOR_THRUST;
            const Vector<4> thrusts = command.thrusts;
            ctrl::rotor_thrust_payload_t thrust_payload;
            thrust_payload.thrust[0] =
              (int16_t)(thrusts(0) *
                        (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::FORCE_RANGE));
            thrust_payload.thrust[1] =
              (int16_t)(thrusts(1) *
                        (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::FORCE_RANGE));
            thrust_payload.thrust[2] =
              (int16_t)(thrusts(2) *
                        (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::FORCE_RANGE));
            thrust_payload.thrust[3] =
              (int16_t)(thrusts(3) *
                        (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::FORCE_RANGE));
            memcpy(msg.payload, &thrust_payload,
                   getPayloadLength(ctrl::CTRLMODE::ROTOR_THRUST));
            break;
          }
          case ctrl::CTRLMODE::ROTOR_THROTTLE: {
            // command in throttle
            msg.mode = ctrl::CTRLMODE::ROTOR_THROTTLE;
            ctrl::rotor_throttle_payload_t throttle_payload;
            const Vector<4> thrusts = quad_.clampThrust(command.thrusts);
            throttle_payload.throttle[0] =
              (int16_t)thrust_map_.map(thrusts(0), current_feedback.voltage);
            throttle_payload.throttle[1] =
              (int16_t)thrust_map_.map(thrusts(1), current_feedback.voltage);
            throttle_payload.throttle[2] =
              (int16_t)thrust_map_.map(thrusts(2), current_feedback.voltage);
            throttle_payload.throttle[3] =
              (int16_t)thrust_map_.map(thrusts(3), current_feedback.voltage);

            memcpy(msg.payload, &throttle_payload,
                   getPayloadLength(ctrl::CTRLMODE::ROTOR_THROTTLE));
            break;
          }
          case ctrl::CTRLMODE::ROTOR_SPEED: {
            // command in rotor speed
            msg.mode = ctrl::CTRLMODE::ROTOR_SPEED;
            ctrl::rotor_speed_payload_t rotor_speed_payload;
            const Vector<4> thrusts = quad_.clampThrust(command.thrusts);
            rotor_speed_payload.speed[0] = (int16_t)(
              sqrt(thrusts[0] / thrust_coeff_) *
              (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::ROTOR_SPEED_RANGE));
            rotor_speed_payload.speed[1] = (int16_t)(
              sqrt(thrusts[1] / thrust_coeff_) *
              (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::ROTOR_SPEED_RANGE));
            rotor_speed_payload.speed[2] = (int16_t)(
              sqrt(thrusts[2] / thrust_coeff_) *
              (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::ROTOR_SPEED_RANGE));
            rotor_speed_payload.speed[3] = (int16_t)(
              sqrt(thrusts[3] / thrust_coeff_) *
              (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::ROTOR_SPEED_RANGE));

            memcpy(msg.payload, &rotor_speed_payload,
                   getPayloadLength(ctrl::CTRLMODE::ROTOR_SPEED));
            break;
          }
          case ctrl::CTRLMODE::BODY_RATE: {
            // command in body rates
            msg.mode = ctrl::CTRLMODE::BODY_RATE;
            ctrl::body_rate_payload_t rate_payload;
            rate_payload.body_rate[0] =
              (int16_t)(command.omega(0) * (agi::ctrl::FREQUENCY_RANGE /
                                            agi::ctrl::ANGULAR_VELOCITY_RANGE));
            rate_payload.body_rate[1] =
              (int16_t)(command.omega(1) * (agi::ctrl::FREQUENCY_RANGE /
                                            agi::ctrl::ANGULAR_VELOCITY_RANGE));
            rate_payload.body_rate[2] =
              (int16_t)(command.omega(2) * (agi::ctrl::FREQUENCY_RANGE /
                                            agi::ctrl::ANGULAR_VELOCITY_RANGE));

            // thrust-map and thrust limits are for single rotor but
            // collective_thrust is for 4
            const Scalar single_thrust =
              quad_.clampThrust(command.collective_thrust / 4.0);
            rate_payload.thrust = (int16_t)(
              thrust_map_.map(single_thrust * quad_.m_,
                              current_feedback.voltage) *
              4.0 * (agi::ctrl::FREQUENCY_RANGE / agi::ctrl::FORCE_RANGE));

            memcpy(msg.payload, &rate_payload,
                   getPayloadLength(ctrl::CTRLMODE::BODY_RATE));
            break;
          }
          default:
            logger_.error("Command mode %u is not implemented", mode_);
            return false;
        }
      }

    } else {
      // Not yet armed, lets arm then...
      msg.mode = ctrl::CTRLMODE::ARM;
    }
  }

  if (!encodeContainer(container, &buffer[1], length)) {
    logger_.error("Encoding error!");
    return false;
  }

  buffer[0] = DELIMITER;
  (*length)++;
  /* append at least 1 delimiter and pad to a multiple of 8 bytes in length */
  do {
    buffer[(*length)++] = DELIMITER;
  } while (*length % 16 != 0);
  return true;
}


bool CtrlEncoding::decodeFeedback(const char* const buffer, const int length,
                                  Feedback* const feedback) {
  ctrl::serial_container_t container;

  if (!decodeContainer(buffer, length, &container)) {
    logger_.error("Container decoding error!");
    return false;
  }

  int n_successful_decoded = 0;
  Feedback new_feedback;
  for (int i = 0; i < (int)container.num_messages; ++i) {
    if (container.messages[i].mode != ctrl::CTRLMODE::FEEDBACK) {
      logger_.error("Received unexpected message!");
      continue;
    }

    const ctrl::upstream_payload_t data =
      *((ctrl::upstream_payload_t*)container.messages[i].payload);
    const uint64_t time_ns = container.messages[i].time;


    const Scalar t = 1e-9 * (time_ns_init_ + time_ns);
    if (t < feedback_.t ||
        (std::isfinite(new_feedback.t) && t < new_feedback.t)) {
      logger_.error("Received old feedback message! %0.2f %0.2f", t,
                    feedback_.t);
      continue;
    }

    ++n_successful_decoded;

    new_feedback.t = t;
    new_feedback.imu.acc.x() = data.linear_accel_x;
    new_feedback.imu.acc.y() = data.linear_accel_y;
    new_feedback.imu.acc.z() = data.linear_accel_z;
    new_feedback.imu.omega.x() = data.angular_vel_x;
    new_feedback.imu.omega.y() = data.angular_vel_y;
    new_feedback.imu.omega.z() = data.angular_vel_z;
    new_feedback.imu.t = t;

    new_feedback.attitude = Quaternion(data.quaternion_w, data.quaternion_x,
                                       data.quaternion_y, data.quaternion_z);

    new_feedback.voltage = data.battery_voltage;
    new_feedback.current = data.battery_current;

    new_feedback.control_mode =
      static_cast<Feedback::CTRLMODE>(data.control_mode);
    new_feedback.armed = new_feedback.control_mode > Feedback::CTRLMODE::ARM;
    new_feedback.rotor_feedback_type =
      static_cast<Feedback::ROTORFEEDBACKTYPE>(data.feedback_type);

    switch (new_feedback.rotor_feedback_type) {
      case Feedback::ROTORFEEDBACKTYPE::SPEED:
        new_feedback.rotor_speed_rads(0) = data.feedback_0;
        new_feedback.rotor_speed_rads(1) = data.feedback_1;
        new_feedback.rotor_speed_rads(2) = data.feedback_2;
        new_feedback.rotor_speed_rads(3) = data.feedback_3;
        new_feedback.rotor_thrust_newton.setConstant(NAN);
        new_feedback.rotor_value.setConstant(NAN);
        break;
      case Feedback::ROTORFEEDBACKTYPE::THROTTLE:
        new_feedback.rotor_speed_rads.setConstant(NAN);
        new_feedback.rotor_thrust_newton.setConstant(NAN);
        new_feedback.rotor_value(0) = data.feedback_0;
        new_feedback.rotor_value(1) = data.feedback_1;
        new_feedback.rotor_value(2) = data.feedback_2;
        new_feedback.rotor_value(3) = data.feedback_3;
        break;
      case Feedback::ROTORFEEDBACKTYPE::THRUST:
        new_feedback.rotor_speed_rads.setConstant(NAN);
        new_feedback.rotor_thrust_newton(0) = data.feedback_0;
        new_feedback.rotor_thrust_newton(1) = data.feedback_1;
        new_feedback.rotor_thrust_newton(2) = data.feedback_2;
        new_feedback.rotor_thrust_newton(3) = data.feedback_3;
        new_feedback.rotor_value.setConstant(NAN);
        break;
      default:
        new_feedback.rotor_speed_rads.setConstant(NAN);
        new_feedback.rotor_thrust_newton.setConstant(NAN);
        new_feedback.rotor_value.setConstant(NAN);
    }
  }

  if (feedback != nullptr) *feedback = new_feedback;

  std::lock_guard<std::mutex> lock(feedback_mtx_);
  feedback_ = new_feedback;

  return n_successful_decoded > 0;
}

bool CtrlEncoding::encodeMessage(const ctrl::serial_message_t& message,
                                 char* const buffer, int* const length) const {
  if (buffer == nullptr) return false;
  if (length == nullptr) return false;

  const int msg_length = getMsgLength(message.mode);
  if (msg_length > ctrl::SERIAL_MESSAGE_MAX_LENGTH) return false;
  if (msg_length > *length) return false;
  *length = msg_length;

  char* ptr = buffer;
  *ptr++ = *length;
  *ptr++ = message.mode;
  *ptr++ = message.flags;
  memcpy(ptr, &message.time, sizeof(uint64_t));
  ptr += sizeof(uint64_t);
  memcpy(ptr, &message.payload, getPayloadLength(message.mode));
  return true;
}

bool CtrlEncoding::decodeMessage(
  const char* const buffer, const int length,
  ctrl::serial_message_t* const serial_message) const {
  if (buffer == nullptr) return false;
  if (length < ctrl::SERIAL_MESSAGE_MIN_LENGTH) return false;
  if (serial_message == nullptr) return false;

  const char* ptr = buffer;

  const char msg_length = *ptr++;

  if (msg_length < ctrl::SERIAL_MESSAGE_MIN_LENGTH) return false;

  serial_message->mode = *ptr++;
  serial_message->flags = *ptr++;

  memcpy(&serial_message->time, ptr, sizeof(uint64_t));
  ptr += sizeof(uint64_t);

  const int payload_length = getPayloadLength(serial_message->mode);
  const int msg_length_expected = getMsgLength(serial_message->mode);

  if (length < msg_length_expected) return false;
  if (msg_length < msg_length_expected) return false;

  memcpy(&serial_message->payload, ptr, payload_length);

  return true;
}

bool CtrlEncoding::encodeContainer(
  const ctrl::serial_container_t& serial_container, char* const buffer,
  int* const length) const {
  if (buffer == nullptr) return false;
  if (length == NULL) return false;

  if (serial_container.num_messages < 1) return false;
  if (serial_container.num_messages > ctrl::SERIAL_MESSAGE_MAX_COUNT)
    return false;

  char container_buffer[ctrl::SERIAL_CONTAINER_MAX_LENGTH];

  int pos = 1;

  container_buffer[pos++] = serial_container.num_messages;

  for (int i = 0; i < serial_container.num_messages; ++i) {
    int len = *length - pos;
    if (!encodeMessage(serial_container.messages[i], &container_buffer[pos],
                       &len))
      return false;

    pos += len;
  }

  container_buffer[0] = pos;

  const uint16_t checksum = crc_.compute(container_buffer, pos);
  container_buffer[pos++] = checksum;
  container_buffer[pos++] = checksum >> 8;

  if (!COBS<DELIMITER>::stuff(container_buffer, pos, buffer, length))
    return false;

  return true;
}

bool CtrlEncoding::decodeContainer(
  const char* const buffer, const int length,
  ctrl::serial_container_t* const serial_container) const {
  if (buffer == nullptr) return false;
  if (serial_container == nullptr) return false;

  if (length == 0) return false;

  int unstuffed_length = ctrl::SERIAL_CONTAINER_MAX_LENGTH;
  char container_buffer[ctrl::SERIAL_CONTAINER_MAX_LENGTH];

  if (!COBS<DELIMITER>::unstuff(buffer, length, container_buffer,
                                &unstuffed_length))
    return false;

  if (unstuffed_length - 2 != container_buffer[0]) return false;

  // Compute checksum
  const uint16_t checksum = crc_.compute(container_buffer, unstuffed_length);

  // Compare checksum
  if (checksum != 0) return false;

  // Read all messags
  serial_container->num_messages = container_buffer[1];

  if (serial_container->num_messages < 1) return false;
  if (serial_container->num_messages > ctrl::SERIAL_MESSAGE_MAX_COUNT)
    return false;

  int pos = 2u;

  for (int i = 0; i < serial_container->num_messages; ++i) {
    int msg_length = container_buffer[pos];
    int len = length - pos;
    if (!decodeMessage(&container_buffer[pos], len,
                       &serial_container->messages[i]))
      return false;

    pos += msg_length;
  }

  return true;
}

int CtrlEncoding::getPayloadLength(const ctrl::CTRLMODE mode) {
  switch (mode) {
    case ctrl::CTRLMODE::OFF:
    case ctrl::CTRLMODE::ARM:
    case ctrl::CTRLMODE::EMERGENCY:
      return 0;
    case ctrl::CTRLMODE::ROTOR_THROTTLE:
      return (int)sizeof(ctrl::rotor_throttle_payload_t);
    case ctrl::CTRLMODE::ROTOR_SPEED:
      return (int)sizeof(ctrl::rotor_speed_payload_t);
    case ctrl::CTRLMODE::ROTOR_THRUST:
      return (int)sizeof(ctrl::rotor_thrust_payload_t);
    case ctrl::CTRLMODE::BODY_RATE:
      return (int)sizeof(ctrl::body_rate_payload_t);
    case ctrl::CTRLMODE::ATTITUDE:
      return (int)sizeof(ctrl::attitude_payload_t);
    case ctrl::CTRLMODE::VELOCITY:
      return (int)sizeof(ctrl::velocity_payload_t);
    case ctrl::CTRLMODE::SET_PARAM:
      return (int)sizeof(ctrl::parameter_payload_t);
    case ctrl::CTRLMODE::FEEDBACK:
      return (int)sizeof(ctrl::upstream_payload_t);
    default:
      return 0;
  }
}

int CtrlEncoding::getPayloadLength(const uint8_t mode) {
  return getPayloadLength((ctrl::CTRLMODE)mode);
}

int CtrlEncoding::getMsgLength(const ctrl::CTRLMODE mode) {
  return 1 + ctrl::SERIAL_MESSAGE_HEADER_LENGTH + getPayloadLength(mode);
}

int CtrlEncoding::getMsgLength(const uint8_t mode) {
  return getMsgLength((ctrl::CTRLMODE)mode);
}
}  // namespace agi
