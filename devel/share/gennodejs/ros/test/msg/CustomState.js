// Auto-generated. Do not edit!

// (in-package test.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CustomState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta1 = null;
      this.theta1_dot = null;
      this.theta2 = null;
      this.theta2_dot = null;
    }
    else {
      if (initObj.hasOwnProperty('theta1')) {
        this.theta1 = initObj.theta1
      }
      else {
        this.theta1 = 0.0;
      }
      if (initObj.hasOwnProperty('theta1_dot')) {
        this.theta1_dot = initObj.theta1_dot
      }
      else {
        this.theta1_dot = 0.0;
      }
      if (initObj.hasOwnProperty('theta2')) {
        this.theta2 = initObj.theta2
      }
      else {
        this.theta2 = 0.0;
      }
      if (initObj.hasOwnProperty('theta2_dot')) {
        this.theta2_dot = initObj.theta2_dot
      }
      else {
        this.theta2_dot = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CustomState
    // Serialize message field [theta1]
    bufferOffset = _serializer.float64(obj.theta1, buffer, bufferOffset);
    // Serialize message field [theta1_dot]
    bufferOffset = _serializer.float64(obj.theta1_dot, buffer, bufferOffset);
    // Serialize message field [theta2]
    bufferOffset = _serializer.float64(obj.theta2, buffer, bufferOffset);
    // Serialize message field [theta2_dot]
    bufferOffset = _serializer.float64(obj.theta2_dot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CustomState
    let len;
    let data = new CustomState(null);
    // Deserialize message field [theta1]
    data.theta1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta1_dot]
    data.theta1_dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta2]
    data.theta2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta2_dot]
    data.theta2_dot = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'test/CustomState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8d2dcedf46285bfb75a4965f00f0338';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 theta1
    float64 theta1_dot
    float64 theta2
    float64 theta2_dot
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CustomState(null);
    if (msg.theta1 !== undefined) {
      resolved.theta1 = msg.theta1;
    }
    else {
      resolved.theta1 = 0.0
    }

    if (msg.theta1_dot !== undefined) {
      resolved.theta1_dot = msg.theta1_dot;
    }
    else {
      resolved.theta1_dot = 0.0
    }

    if (msg.theta2 !== undefined) {
      resolved.theta2 = msg.theta2;
    }
    else {
      resolved.theta2 = 0.0
    }

    if (msg.theta2_dot !== undefined) {
      resolved.theta2_dot = msg.theta2_dot;
    }
    else {
      resolved.theta2_dot = 0.0
    }

    return resolved;
    }
};

module.exports = CustomState;
