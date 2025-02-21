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

class IR {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.IR_front_left = null;
      this.IR_back_left = null;
      this.IR_front_right = null;
      this.IR_back_right = null;
    }
    else {
      if (initObj.hasOwnProperty('IR_front_left')) {
        this.IR_front_left = initObj.IR_front_left
      }
      else {
        this.IR_front_left = 0.0;
      }
      if (initObj.hasOwnProperty('IR_back_left')) {
        this.IR_back_left = initObj.IR_back_left
      }
      else {
        this.IR_back_left = 0.0;
      }
      if (initObj.hasOwnProperty('IR_front_right')) {
        this.IR_front_right = initObj.IR_front_right
      }
      else {
        this.IR_front_right = 0.0;
      }
      if (initObj.hasOwnProperty('IR_back_right')) {
        this.IR_back_right = initObj.IR_back_right
      }
      else {
        this.IR_back_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IR
    // Serialize message field [IR_front_left]
    bufferOffset = _serializer.float64(obj.IR_front_left, buffer, bufferOffset);
    // Serialize message field [IR_back_left]
    bufferOffset = _serializer.float64(obj.IR_back_left, buffer, bufferOffset);
    // Serialize message field [IR_front_right]
    bufferOffset = _serializer.float64(obj.IR_front_right, buffer, bufferOffset);
    // Serialize message field [IR_back_right]
    bufferOffset = _serializer.float64(obj.IR_back_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IR
    let len;
    let data = new IR(null);
    // Deserialize message field [IR_front_left]
    data.IR_front_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR_back_left]
    data.IR_back_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR_front_right]
    data.IR_front_right = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR_back_right]
    data.IR_back_right = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roswifibot/IR';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08989c603acc510242caf5149106a2a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 IR_front_left
    float64 IR_back_left
    float64 IR_front_right
    float64 IR_back_right
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IR(null);
    if (msg.IR_front_left !== undefined) {
      resolved.IR_front_left = msg.IR_front_left;
    }
    else {
      resolved.IR_front_left = 0.0
    }

    if (msg.IR_back_left !== undefined) {
      resolved.IR_back_left = msg.IR_back_left;
    }
    else {
      resolved.IR_back_left = 0.0
    }

    if (msg.IR_front_right !== undefined) {
      resolved.IR_front_right = msg.IR_front_right;
    }
    else {
      resolved.IR_front_right = 0.0
    }

    if (msg.IR_back_right !== undefined) {
      resolved.IR_back_right = msg.IR_back_right;
    }
    else {
      resolved.IR_back_right = 0.0
    }

    return resolved;
    }
};

module.exports = IR;
