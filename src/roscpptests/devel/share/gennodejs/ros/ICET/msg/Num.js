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
      this.restart = null;
      this.frame = null;
      this.status = null;
      this.true_transform = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('restart')) {
        this.restart = initObj.restart
      }
      else {
        this.restart = false;
      }
      if (initObj.hasOwnProperty('frame')) {
        this.frame = initObj.frame
      }
      else {
        this.frame = 0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
      if (initObj.hasOwnProperty('true_transform')) {
        this.true_transform = initObj.true_transform
      }
      else {
        this.true_transform = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Num
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [restart]
    bufferOffset = _serializer.bool(obj.restart, buffer, bufferOffset);
    // Serialize message field [frame]
    bufferOffset = _serializer.int32(obj.frame, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    // Serialize message field [true_transform]
    bufferOffset = _arraySerializer.float32(obj.true_transform, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Num
    let len;
    let data = new Num(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [restart]
    data.restart = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [frame]
    data.frame = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [true_transform]
    data.true_transform = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.status);
    length += 4 * object.true_transform.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ICET/Num';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93042447ed01f85739c5e6e8683f8ec7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time timestamp
    bool restart
    int32 frame
    string status
    float32[] true_transform
    
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

    if (msg.restart !== undefined) {
      resolved.restart = msg.restart;
    }
    else {
      resolved.restart = false
    }

    if (msg.frame !== undefined) {
      resolved.frame = msg.frame;
    }
    else {
      resolved.frame = 0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    if (msg.true_transform !== undefined) {
      resolved.true_transform = msg.true_transform;
    }
    else {
      resolved.true_transform = []
    }

    return resolved;
    }
};

module.exports = Num;
