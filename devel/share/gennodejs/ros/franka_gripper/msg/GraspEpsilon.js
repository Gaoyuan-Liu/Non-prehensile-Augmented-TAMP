// Auto-generated. Do not edit!

// (in-package franka_gripper.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GraspEpsilon {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inner = null;
      this.outer = null;
    }
    else {
      if (initObj.hasOwnProperty('inner')) {
        this.inner = initObj.inner
      }
      else {
        this.inner = 0.0;
      }
      if (initObj.hasOwnProperty('outer')) {
        this.outer = initObj.outer
      }
      else {
        this.outer = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspEpsilon
    // Serialize message field [inner]
    bufferOffset = _serializer.float64(obj.inner, buffer, bufferOffset);
    // Serialize message field [outer]
    bufferOffset = _serializer.float64(obj.outer, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspEpsilon
    let len;
    let data = new GraspEpsilon(null);
    // Deserialize message field [inner]
    data.inner = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [outer]
    data.outer = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_gripper/GraspEpsilon';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95b2c5464a6f679bd1dacaf86414f095';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 inner # [m]
    float64 outer # [m]
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspEpsilon(null);
    if (msg.inner !== undefined) {
      resolved.inner = msg.inner;
    }
    else {
      resolved.inner = 0.0
    }

    if (msg.outer !== undefined) {
      resolved.outer = msg.outer;
    }
    else {
      resolved.outer = 0.0
    }

    return resolved;
    }
};

module.exports = GraspEpsilon;
