// Auto-generated. Do not edit!

// (in-package seed_r7_ros_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class HandControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.script = null;
      this.current = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0;
      }
      if (initObj.hasOwnProperty('script')) {
        this.script = initObj.script
      }
      else {
        this.script = '';
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HandControlRequest
    // Serialize message field [position]
    bufferOffset = _serializer.uint8(obj.position, buffer, bufferOffset);
    // Serialize message field [script]
    bufferOffset = _serializer.string(obj.script, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.uint8(obj.current, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HandControlRequest
    let len;
    let data = new HandControlRequest(null);
    // Deserialize message field [position]
    data.position = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [script]
    data.script = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.script.length;
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'seed_r7_ros_controller/HandControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b8557444e21ab5689c6d4cf096e0f7e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 position
    uint8 POSITION_RIGHT = 0
    uint8 POSITION_LEFT = 1
    
    string script
    string SCRIPT_GRASP = grasp
    string SCRIPT_RELEASE = release
    string SCRIPT_CANCEL = cancel
    
    uint8  current
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HandControlRequest(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0
    }

    if (msg.script !== undefined) {
      resolved.script = msg.script;
    }
    else {
      resolved.script = ''
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0
    }

    return resolved;
    }
};

// Constants for message
HandControlRequest.Constants = {
  POSITION_RIGHT: 0,
  POSITION_LEFT: 1,
  SCRIPT_GRASP: 'grasp',
  SCRIPT_RELEASE: 'release',
  SCRIPT_CANCEL: 'cancel',
}

class HandControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HandControlResponse
    // Serialize message field [result]
    bufferOffset = _serializer.string(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HandControlResponse
    let len;
    let data = new HandControlResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.result.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'seed_r7_ros_controller/HandControlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c22f2a1ed8654a0b365f1bb3f7ff2c0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HandControlResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: HandControlRequest,
  Response: HandControlResponse,
  md5sum() { return 'beb2871e68a142be80f3b23a0a3a73da'; },
  datatype() { return 'seed_r7_ros_controller/HandControl'; }
};
