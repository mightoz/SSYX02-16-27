; Auto-generated. Do not edit!


(cl:in-package robotclient-srv)


;//! \htmlinclude MoveRobot-request.msg.html

(cl:defclass <MoveRobot-request> (roslisp-msg-protocol:ros-message)
  ((length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveRobot-request (<MoveRobot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveRobot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveRobot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotclient-srv:<MoveRobot-request> is deprecated: use robotclient-srv:MoveRobot-request instead.")))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <MoveRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotclient-srv:length-val is deprecated.  Use robotclient-srv:length instead.")
  (length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveRobot-request>) ostream)
  "Serializes a message object of type '<MoveRobot-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveRobot-request>) istream)
  "Deserializes a message object of type '<MoveRobot-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveRobot-request>)))
  "Returns string type for a service object of type '<MoveRobot-request>"
  "robotclient/MoveRobotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRobot-request)))
  "Returns string type for a service object of type 'MoveRobot-request"
  "robotclient/MoveRobotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveRobot-request>)))
  "Returns md5sum for a message object of type '<MoveRobot-request>"
  "73d983cb94f147797fb40bd0af7e75a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveRobot-request)))
  "Returns md5sum for a message object of type 'MoveRobot-request"
  "73d983cb94f147797fb40bd0af7e75a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveRobot-request>)))
  "Returns full string definition for message of type '<MoveRobot-request>"
  (cl:format cl:nil "float64 length~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveRobot-request)))
  "Returns full string definition for message of type 'MoveRobot-request"
  (cl:format cl:nil "float64 length~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveRobot-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveRobot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveRobot-request
    (cl:cons ':length (length msg))
))
;//! \htmlinclude MoveRobot-response.msg.html

(cl:defclass <MoveRobot-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MoveRobot-response (<MoveRobot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveRobot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveRobot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotclient-srv:<MoveRobot-response> is deprecated: use robotclient-srv:MoveRobot-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <MoveRobot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotclient-srv:ack-val is deprecated.  Use robotclient-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveRobot-response>) ostream)
  "Serializes a message object of type '<MoveRobot-response>"
  (cl:let* ((signed (cl:slot-value msg 'ack)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveRobot-response>) istream)
  "Deserializes a message object of type '<MoveRobot-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ack) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveRobot-response>)))
  "Returns string type for a service object of type '<MoveRobot-response>"
  "robotclient/MoveRobotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRobot-response)))
  "Returns string type for a service object of type 'MoveRobot-response"
  "robotclient/MoveRobotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveRobot-response>)))
  "Returns md5sum for a message object of type '<MoveRobot-response>"
  "73d983cb94f147797fb40bd0af7e75a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveRobot-response)))
  "Returns md5sum for a message object of type 'MoveRobot-response"
  "73d983cb94f147797fb40bd0af7e75a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveRobot-response>)))
  "Returns full string definition for message of type '<MoveRobot-response>"
  (cl:format cl:nil "int8 ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveRobot-response)))
  "Returns full string definition for message of type 'MoveRobot-response"
  (cl:format cl:nil "int8 ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveRobot-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveRobot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveRobot-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveRobot)))
  'MoveRobot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveRobot)))
  'MoveRobot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRobot)))
  "Returns string type for a service object of type '<MoveRobot>"
  "robotclient/MoveRobot")