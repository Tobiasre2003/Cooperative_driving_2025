// Auto-generated. Do not edit!

// (in-package roswifibot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.battery_level = null;
      this.current = null;
      this.ADC1 = null;
      this.ADC2 = null;
      this.ADC3 = null;
      this.ADC4 = null;
      this.speed_front_left = null;
      this.speed_front_right = null;
      this.odometry_left = null;
      this.odometry_right = null;
      this.version = null;
      this.relay1 = null;
      this.relay2 = null;
      this.relay3 = null;
    }
    else {
      if (initObj.hasOwnProperty('battery_level')) {
        this.battery_level = initObj.battery_level
      }
      else {
        this.battery_level = 0.0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('ADC1')) {
        this.ADC1 = initObj.ADC1
      }
      else {
        this.ADC1 = 0;
      }
      if (initObj.hasOwnProperty('ADC2')) {
        this.ADC2 = initObj.ADC2
      }
      else {
        this.ADC2 = 0;
      }
      if (initObj.hasOwnProperty('ADC3')) {
        this.ADC3 = initObj.ADC3
      }
      else {
        this.ADC3 = 0;
      }
      if (initObj.hasOwnProperty('ADC4')) {
        this.ADC4 = initObj.ADC4
      }
      else {
        this.ADC4 = 0;
      }
      if (initObj.hasOwnProperty('speed_front_left')) {
        this.speed_front_left = initObj.speed_front_left
      }
      else {
        this.speed_front_left = 0.0;
      }
      if (initObj.hasOwnProperty('speed_front_right')) {
        this.speed_front_right = initObj.speed_front_right
      }
      else {
        this.speed_front_right = 0.0;
      }
      if (initObj.hasOwnProperty('odometry_left')) {
        this.odometry_left = initObj.odometry_left
      }
      else {
        this.odometry_left = 0.0;
      }
      if (initObj.hasOwnProperty('odometry_right')) {
        this.odometry_right = initObj.odometry_right
      }
      else {
        this.odometry_right = 0.0;
      }
      if (initObj.hasOwnProperty('version')) {
        this.version = initObj.version
      }
      else {
        this.version = 0;
      }
      if (initObj.hasOwnProperty('relay1')) {
        this.relay1 = initObj.relay1
      }
      else {
        this.relay1 = 0;
      }
      if (initObj.hasOwnProperty('relay2')) {
        this.relay2 = initObj.relay2
      }
      else {
        this.relay2 = 0;
      }
      if (initObj.hasOwnProperty('relay3')) {
        this.relay3 = initObj.relay3
      }
      else {
        this.relay3 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Status
    // Serialize message field [battery_level]
    bufferOffset = _serializer.float64(obj.battery_level, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float64(obj.current, buffer, bufferOffset);
    // Serialize message field [ADC1]
    bufferOffset = _serializer.int32(obj.ADC1, buffer, bufferOffset);
    // Serialize message field [ADC2]
    bufferOffset = _serializer.int32(obj.ADC2, buffer, bufferOffset);
    // Serialize message field [ADC3]
    bufferOffset = _serializer.int32(obj.ADC3, buffer, bufferOffset);
    // Serialize message field [ADC4]
    bufferOffset = _serializer.int32(obj.ADC4, buffer, bufferOffset);
    // Serialize message field [speed_front_left]
    bufferOffset = _serializer.float64(obj.speed_front_left, buffer, bufferOffset);
    // Serialize message field [speed_front_right]
    bufferOffset = _serializer.float64(obj.speed_front_right, buffer, bufferOffset);
    // Serialize message field [odometry_left]
    bufferOffset = _serializer.float64(obj.odometry_left, buffer, bufferOffset);
    // Serialize message field [odometry_right]
    bufferOffset = _serializer.float64(obj.odometry_right, buffer, bufferOffset);
    // Serialize message field [version]
    bufferOffset = _serializer.int32(obj.version, buffer, bufferOffset);
    // Serialize message field [relay1]
    bufferOffset = _serializer.int8(obj.relay1, buffer, bufferOffset);
    // Serialize message field [relay2]
    bufferOffset = _serializer.int8(obj.relay2, buffer, bufferOffset);
    // Serialize message field [relay3]
    bufferOffset = _serializer.int8(obj.relay3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Status
    let len;
    let data = new Status(null);
    // Deserialize message field [battery_level]
    data.battery_level = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ADC1]
    data.ADC1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ADC2]
    data.ADC2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ADC3]
    data.ADC3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ADC4]
    data.ADC4 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [speed_front_left]
    data.speed_front_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speed_front_right]
    data.speed_front_right = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [odometry_left]
    data.odometry_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [odometry_right]
    data.odometry_right = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [version]
    data.version = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [relay1]
    data.relay1 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [relay2]
    data.relay2 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [relay3]
    data.relay3 = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 71;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roswifibot/Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd55c85d21aaf2691ac6422ea4770a7d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 battery_level
    float64 current
    int32 ADC1
    int32 ADC2
    int32 ADC3
    int32 ADC4
    float64 speed_front_left
    float64 speed_front_right
    float64 odometry_left
    float64 odometry_right
    int32 version
    int8 relay1
    int8 relay2
    int8 relay3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Status(null);
    if (msg.battery_level !== undefined) {
      resolved.battery_level = msg.battery_level;
    }
    else {
      resolved.battery_level = 0.0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.ADC1 !== undefined) {
      resolved.ADC1 = msg.ADC1;
    }
    else {
      resolved.ADC1 = 0
    }

    if (msg.ADC2 !== undefined) {
      resolved.ADC2 = msg.ADC2;
    }
    else {
      resolved.ADC2 = 0
    }

    if (msg.ADC3 !== undefined) {
      resolved.ADC3 = msg.ADC3;
    }
    else {
      resolved.ADC3 = 0
    }

    if (msg.ADC4 !== undefined) {
      resolved.ADC4 = msg.ADC4;
    }
    else {
      resolved.ADC4 = 0
    }

    if (msg.speed_front_left !== undefined) {
      resolved.speed_front_left = msg.speed_front_left;
    }
    else {
      resolved.speed_front_left = 0.0
    }

    if (msg.speed_front_right !== undefined) {
      resolved.speed_front_right = msg.speed_front_right;
    }
    else {
      resolved.speed_front_right = 0.0
    }

    if (msg.odometry_left !== undefined) {
      resolved.odometry_left = msg.odometry_left;
    }
    else {
      resolved.odometry_left = 0.0
    }

    if (msg.odometry_right !== undefined) {
      resolved.odometry_right = msg.odometry_right;
    }
    else {
      resolved.odometry_right = 0.0
    }

    if (msg.version !== undefined) {
      resolved.version = msg.version;
    }
    else {
      resolved.version = 0
    }

    if (msg.relay1 !== undefined) {
      resolved.relay1 = msg.relay1;
    }
    else {
      resolved.relay1 = 0
    }

    if (msg.relay2 !== undefined) {
      resolved.relay2 = msg.relay2;
    }
    else {
      resolved.relay2 = 0
    }

    if (msg.relay3 !== undefined) {
      resolved.relay3 = msg.relay3;
    }
    else {
      resolved.relay3 = 0
    }

    return resolved;
    }
};

module.exports = Status;
