// Auto-generated. Do not edit!

// (in-package ICET.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Num {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.errors = null;
      this.ch1 = null;
      this.ch2 = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('errors')) {
        this.errors = initObj.errors
      }
      else {
        this.errors = false;
      }
      if (initObj.hasOwnProperty('ch1')) {
        this.ch1 = initObj.ch1
      }
      else {
        this.ch1 = 0.0;
      }
      if (initObj.hasOwnProperty('ch2')) {
        this.ch2 = initObj.ch2
      }
      else {
        this.ch2 = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Num
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [errors]
    bufferOffset = _serializer.bool(obj.errors, buffer, bufferOffset);
    // Serialize message field [ch1]
    bufferOffset = _serializer.float32(obj.ch1, buffer, bufferOffset);
    // Serialize message field [ch2]
    bufferOffset = _serializer.float32(obj.ch2, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Num
    let len;
    let data = new Num(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [errors]
    data.errors = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ch1]
    data.ch1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ch2]
    data.ch2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.status);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ICET/Num';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c756fa6b1c2901279edb34f2bd952f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time timestamp
    bool errors
    float32 ch1
    float32 ch2
    string status
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Num(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = {secs: 0, nsecs: 0}
    }

    if (msg.errors !== undefined) {
      resolved.errors = msg.errors;
    }
    else {
      resolved.errors = false
    }

    if (msg.ch1 !== undefined) {
      resolved.ch1 = msg.ch1;
    }
    else {
      resolved.ch1 = 0.0
    }

    if (msg.ch2 !== undefined) {
      resolved.ch2 = msg.ch2;
    }
    else {
      resolved.ch2 = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    return resolved;
    }
};

module.exports = Num;
