// Auto-generated. Do not edit!

// (in-package mapdata.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Position = require('./Position.js');

//-----------------------------------------------------------

class RoadSection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left = null;
      this.right = null;
      this.length = null;
      this.stopline_offset = null;
      this.priority_sign = null;
      this.name = null;
    }
    else {
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = new Position();
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = new Position();
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0;
      }
      if (initObj.hasOwnProperty('stopline_offset')) {
        this.stopline_offset = initObj.stopline_offset
      }
      else {
        this.stopline_offset = 0;
      }
      if (initObj.hasOwnProperty('priority_sign')) {
        this.priority_sign = initObj.priority_sign
      }
      else {
        this.priority_sign = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RoadSection
    // Serialize message field [left]
    bufferOffset = Position.serialize(obj.left, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = Position.serialize(obj.right, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.int32(obj.length, buffer, bufferOffset);
    // Serialize message field [stopline_offset]
    bufferOffset = _serializer.int32(obj.stopline_offset, buffer, bufferOffset);
    // Serialize message field [priority_sign]
    bufferOffset = _serializer.uint8(obj.priority_sign, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RoadSection
    let len;
    let data = new RoadSection(null);
    // Deserialize message field [left]
    data.left = Position.deserialize(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = Position.deserialize(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [stopline_offset]
    data.stopline_offset = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [priority_sign]
    data.priority_sign = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mapdata/RoadSection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e564107726b89210b4492752f37581d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # If you're standing on the road looking towards the intersection, the left
    # position is the leftmost edge of the rectangle closest to the intersection.
    #    |       |
    #    |       |
    #  right    left
    # ---+       +----
    #
    Position left
    Position right
    
    # How far the road section extends from the intersection
    int32 length
    
    # How far from the intersection the cars should stop
    int32 stopline_offset
    
    # Enumeration (just constants) of priority signs
    uint8 PRIORITY_ROAD=0
    uint8 GIVE_WAY=1
    uint8 STOP_SIGN=2
    uint8 TRAFFIC_LIGHT=3
    uint8 BOOKING=4
    
    uint8 priority_sign
    
    # A bit redundant but nice for pretty printing
    string name
    
    ================================================================================
    MSG: mapdata/Position
    int32 x
    int32 y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RoadSection(null);
    if (msg.left !== undefined) {
      resolved.left = Position.Resolve(msg.left)
    }
    else {
      resolved.left = new Position()
    }

    if (msg.right !== undefined) {
      resolved.right = Position.Resolve(msg.right)
    }
    else {
      resolved.right = new Position()
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0
    }

    if (msg.stopline_offset !== undefined) {
      resolved.stopline_offset = msg.stopline_offset;
    }
    else {
      resolved.stopline_offset = 0
    }

    if (msg.priority_sign !== undefined) {
      resolved.priority_sign = msg.priority_sign;
    }
    else {
      resolved.priority_sign = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    return resolved;
    }
};

// Constants for message
RoadSection.Constants = {
  PRIORITY_ROAD: 0,
  GIVE_WAY: 1,
  STOP_SIGN: 2,
  TRAFFIC_LIGHT: 3,
  BOOKING: 4,
}

module.exports = RoadSection;
