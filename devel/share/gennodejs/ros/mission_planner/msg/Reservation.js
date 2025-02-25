// Auto-generated. Do not edit!

// (in-package mission_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Reservation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
      this.end = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = '';
      }
      if (initObj.hasOwnProperty('end')) {
        this.end = initObj.end
      }
      else {
        this.end = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Reservation
    // Serialize message field [start]
    bufferOffset = _serializer.string(obj.start, buffer, bufferOffset);
    // Serialize message field [end]
    bufferOffset = _serializer.string(obj.end, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Reservation
    let len;
    let data = new Reservation(null);
    // Deserialize message field [start]
    data.start = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [end]
    data.end = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.start);
    length += _getByteLength(object.end);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mission_planner/Reservation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fd9b411e8e594457e56809ca789fc69e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string start
    string end
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Reservation(null);
    if (msg.start !== undefined) {
      resolved.start = msg.start;
    }
    else {
      resolved.start = ''
    }

    if (msg.end !== undefined) {
      resolved.end = msg.end;
    }
    else {
      resolved.end = ''
    }

    return resolved;
    }
};

module.exports = Reservation;
