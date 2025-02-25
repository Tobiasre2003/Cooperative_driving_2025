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
let CDMConnectionZone = require('./CDMConnectionZone.js');

//-----------------------------------------------------------

class CDMSection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.pos = null;
      this.type = null;
      this.width = null;
      this.height = null;
      this.orientation = null;
      this.connection_zones = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Position();
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = 0;
      }
      if (initObj.hasOwnProperty('connection_zones')) {
        this.connection_zones = initObj.connection_zones
      }
      else {
        this.connection_zones = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CDMSection
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [pos]
    bufferOffset = Position.serialize(obj.pos, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.int32(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.int32(obj.height, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = _serializer.int32(obj.orientation, buffer, bufferOffset);
    // Serialize message field [connection_zones]
    // Serialize the length for message field [connection_zones]
    bufferOffset = _serializer.uint32(obj.connection_zones.length, buffer, bufferOffset);
    obj.connection_zones.forEach((val) => {
      bufferOffset = CDMConnectionZone.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CDMSection
    let len;
    let data = new CDMSection(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = Position.deserialize(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [connection_zones]
    // Deserialize array length for message field [connection_zones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.connection_zones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.connection_zones[i] = CDMConnectionZone.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.connection_zones.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mapdata/CDMSection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '84a37099866d8390b55fd98c37e4e716';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Type enumeration
    uint8 THREE_WAY_INTERSECTION=0
    uint8 FOUR_WAY_INTERSECTION=1
    uint8 THREE_WAY_ROUNDABOUT=2
    
    # Orientation enumeration
    uint8 NORTH=0
    uint8 SOUTH=1
    uint8 EAST=2
    uint8 WEST=3
    
    
    int32 id
    Position pos
    int32 type
    int32 width
    int32 height
    int32 orientation
    CDMConnectionZone[] connection_zones
    
    ================================================================================
    MSG: mapdata/Position
    int32 x
    int32 y
    
    ================================================================================
    MSG: mapdata/CDMConnectionZone
    Position pos
    uint32 width
    uint32 height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CDMSection(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.pos !== undefined) {
      resolved.pos = Position.Resolve(msg.pos)
    }
    else {
      resolved.pos = new Position()
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = msg.orientation;
    }
    else {
      resolved.orientation = 0
    }

    if (msg.connection_zones !== undefined) {
      resolved.connection_zones = new Array(msg.connection_zones.length);
      for (let i = 0; i < resolved.connection_zones.length; ++i) {
        resolved.connection_zones[i] = CDMConnectionZone.Resolve(msg.connection_zones[i]);
      }
    }
    else {
      resolved.connection_zones = []
    }

    return resolved;
    }
};

// Constants for message
CDMSection.Constants = {
  THREE_WAY_INTERSECTION: 0,
  FOUR_WAY_INTERSECTION: 1,
  THREE_WAY_ROUNDABOUT: 2,
  NORTH: 0,
  SOUTH: 1,
  EAST: 2,
  WEST: 3,
}

module.exports = CDMSection;
