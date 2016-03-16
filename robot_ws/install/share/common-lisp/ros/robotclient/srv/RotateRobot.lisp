; Auto-generated. Do not edit!


(cl:in-package robotclient-srv)


;//! \htmlinclude RotateRobot-request.msg.html

(cl:defclass <RotateRobot-request> (roslisp-msg-protocol:ros-message)
  ((deg
    :reader deg
    :initarg :deg
    :type cl:float
    :initform 0.0))
)

(cl:defclass RotateRobot-request (<RotateRobot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotateRobot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotateRobot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotclient-srv:<RotateRobot-request> is deprecated: use robotclient-srv:RotateRobot-request instead.")))

(cl:ensure-generic-function 'deg-val :lambda-list '(m))
(cl:defmethod deg-val ((m <RotateRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotclient-srv:deg-val is deprecated.  Use robotclient-srv:deg instead.")
  (deg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotateRobot-request>) ostream)
  "Serializes a message object of type '<RotateRobot-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotateRobot-request>) istream)
  "Deserializes a message object of type '<RotateRobot-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'deg) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotateRobot-request>)))
  "Returns string type for a service object of type '<RotateRobot-request>"
  "robotclient/RotateRobotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateRobot-request)))
  "Returns string type for a service object of type 'RotateRobot-request"
  "robotclient/RotateRobotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotateRobot-request>)))
  "Returns md5sum for a message object of type '<RotateRobot-request>"
  "95b88eb44c4cda7e8bd208bebdd2aee8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotateRobot-request)))
  "Returns md5sum for a message object of type 'RotateRobot-request"
  "95b88eb44c4cda7e8bd208bebdd2aee8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotateRobot-request>)))
  "Returns full string definition for message of type '<RotateRobot-request>"
  (cl:format cl:nil "float64 deg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotateRobot-request)))
  "Returns full string definition for message of type 'RotateRobot-request"
  (cl:format cl:nil "float64 deg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotateRobot-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotateRobot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RotateRobot-request
    (cl:cons ':deg (deg msg))
))
;//! \htmlinclude RotateRobot-response.msg.html

(cl:defclass <RotateRobot-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RotateRobot-response (<RotateRobot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotateRobot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotateRobot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotclient-srv:<RotateRobot-response> is deprecated: use robotclient-srv:RotateRobot-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <RotateRobot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotclient-srv:ack-val is deprecated.  Use robotclient-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotateRobot-response>) ostream)
  "Serializes a message object of type '<RotateRobot-response>"
  (cl:let* ((signed (cl:slot-value msg 'ack)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotateRobot-response>) istream)
  "Deserializes a message object of type '<RotateRobot-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ack) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotateRobot-response>)))
  "Returns string type for a service object of type '<RotateRobot-response>"
  "robotclient/RotateRobotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateRobot-response)))
  "Returns string type for a service object of type 'RotateRobot-response"
  "robotclient/RotateRobotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotateRobot-response>)))
  "Returns md5sum for a message object of type '<RotateRobot-response>"
  "95b88eb44c4cda7e8bd208bebdd2aee8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotateRobot-response)))
  "Returns md5sum for a message object of type 'RotateRobot-response"
  "95b88eb44c4cda7e8bd208bebdd2aee8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotateRobot-response>)))
  "Returns full string definition for message of type '<RotateRobot-response>"
  (cl:format cl:nil "int8 ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotateRobot-response)))
  "Returns full string definition for message of type 'RotateRobot-response"
  (cl:format cl:nil "int8 ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotateRobot-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotateRobot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RotateRobot-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RotateRobot)))
  'RotateRobot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RotateRobot)))
  'RotateRobot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateRobot)))
  "Returns string type for a service object of type '<RotateRobot>"
  "robotclient/RotateRobot")