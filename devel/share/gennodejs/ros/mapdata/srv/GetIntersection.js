// Auto-generated. Do not edit!

// (in-package mapdata.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let RoadSection = require('../msg/RoadSection.js');

//-----------------------------------------------------------

class GetIntersectionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIntersectionRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIntersectionRequest
    let len;
    let data = new GetIntersectionRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mapdata/GetIntersectionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetIntersectionRequest(null);
    return resolved;
    }
};

class GetIntersectionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.north = null;
      this.west = null;
      this.south = null;
      this.east = null;
    }
    else {
      if (initObj.hasOwnProperty('north')) {
        this.north = initObj.north
      }
      else {
        this.north = new RoadSection();
      }
      if (initObj.hasOwnProperty('west')) {
        this.west = initObj.west
      }
      else {
        this.west = new RoadSection();
      }
      if (initObj.hasOwnProperty('south')) {
        this.south = initObj.south
      }
      else {
        this.south = new RoadSection();
      }
      if (initObj.hasOwnProperty('east')) {
        this.east = initObj.east
      }
      else {
        this.east = new RoadSection();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIntersectionResponse
    // Serialize message field [north]
    bufferOffset = RoadSection.serialize(obj.north, buffer, bufferOffset);
    // Serialize message field [west]
    bufferOffset = RoadSection.serialize(obj.west, buffer, bufferOffset);
    // Serialize message field [south]
    bufferOffset = RoadSection.serialize(obj.south, buffer, bufferOffset);
    // Serialize message field [east]
    bufferOffset = RoadSection.serialize(obj.east, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIntersectionResponse
    let len;
    let data = new GetIntersectionResponse(null);
    // Deserialize message field [north]
    data.north = RoadSection.deserialize(buffer, bufferOffset);
    // Deserialize message field [west]
    data.west = RoadSection.deserialize(buffer, bufferOffset);
    // Deserialize message field [south]
    data.south = RoadSection.deserialize(buffer, bufferOffset);
    // Deserialize message field [east]
    data.east = RoadSection.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += RoadSection.getMessageSize(object.north);
    length += RoadSection.getMessageSize(object.west);
    length += RoadSection.getMessageSize(object.south);
    length += RoadSection.getMessageSize(object.east);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mapdata/GetIntersectionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05e0de0c4f7d86e84748f13d311cd03e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # For our purposes a 4-way intersection is enough
    RoadSection north
    RoadSection west
    RoadSection south
    RoadSection east
    
    ================================================================================
    MSG: mapdata/RoadSection
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
    const resolved = new GetIntersectionResponse(null);
    if (msg.north !== undefined) {
      resolved.north = RoadSection.Resolve(msg.north)
    }
    else {
      resolved.north = new RoadSection()
    }

    if (msg.west !== undefined) {
      resolved.west = RoadSection.Resolve(msg.west)
    }
    else {
      resolved.west = new RoadSection()
    }

    if (msg.south !== undefined) {
      resolved.south = RoadSection.Resolve(msg.south)
    }
    else {
      resolved.south = new RoadSection()
    }

    if (msg.east !== undefined) {
      resolved.east = RoadSection.Resolve(msg.east)
    }
    else {
      resolved.east = new RoadSection()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetIntersectionRequest,
  Response: GetIntersectionResponse,
  md5sum() { return '05e0de0c4f7d86e84748f13d311cd03e'; },
  datatype() { return 'mapdata/GetIntersection'; }
};
