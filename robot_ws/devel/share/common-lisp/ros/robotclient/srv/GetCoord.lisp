; Auto-generated. Do not edit!


(cl:in-package robotclient-srv)


;//! \htmlinclude GetCoord-request.msg.html

(cl:defclass <GetCoord-request> (roslisp-msg-protocol:ros-message)
  ((in
    :reader in
    :initarg :in
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCoord-request (<GetCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotclient-srv:<GetCoord-request> is deprecated: use robotclient-srv:GetCoord-request instead.")))

(cl:ensure-generic-function 'in-val :lambda-list '(m))
(cl:defmethod in-val ((m <GetCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotclient-srv:in-val is deprecated.  Use robotclient-srv:in instead.")
  (in m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCoord-request>) ostream)
  "Serializes a message object of type '<GetCoord-request>"
  (cl:let* ((signed (cl:slot-value msg 'in)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCoord-request>) istream)
  "Deserializes a message object of type '<GetCoord-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'in) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCoord-request>)))
  "Returns string type for a service object of type '<GetCoord-request>"
  "robotclient/GetCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoord-request)))
  "Returns string type for a service object of type 'GetCoord-request"
  "robotclient/GetCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCoord-request>)))
  "Returns md5sum for a message object of type '<GetCoord-request>"
  "ec7fb4cadb363741bad99758dda47526")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCoord-request)))
  "Returns md5sum for a message object of type 'GetCoord-request"
  "ec7fb4cadb363741bad99758dda47526")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCoord-request>)))
  "Returns full string definition for message of type '<GetCoord-request>"
  (cl:format cl:nil "int8 in~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCoord-request)))
  "Returns full string definition for message of type 'GetCoord-request"
  (cl:format cl:nil "int8 in~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCoord-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCoord-request
    (cl:cons ':in (in msg))
))
;//! \htmlinclude GetCoord-response.msg.html

(cl:defclass <GetCoord-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type robotclient-msg:Floats
    :initform (cl:make-instance 'robotclient-msg:Floats)))
)

(cl:defclass GetCoord-response (<GetCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotclient-srv:<GetCoord-response> is deprecated: use robotclient-srv:GetCoord-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotclient-srv:data-val is deprecated.  Use robotclient-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCoord-response>) ostream)
  "Serializes a message object of type '<GetCoord-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCoord-response>) istream)
  "Deserializes a message object of type '<GetCoord-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCoord-response>)))
  "Returns string type for a service object of type '<GetCoord-response>"
  "robotclient/GetCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoord-response)))
  "Returns string type for a service object of type 'GetCoord-response"
  "robotclient/GetCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCoord-response>)))
  "Returns md5sum for a message object of type '<GetCoord-response>"
  "ec7fb4cadb363741bad99758dda47526")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCoord-response)))
  "Returns md5sum for a message object of type 'GetCoord-response"
  "ec7fb4cadb363741bad99758dda47526")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCoord-response>)))
  "Returns full string definition for message of type '<GetCoord-response>"
  (cl:format cl:nil "Floats data~%~%~%================================================================================~%MSG: robotclient/Floats~%float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCoord-response)))
  "Returns full string definition for message of type 'GetCoord-response"
  (cl:format cl:nil "Floats data~%~%~%================================================================================~%MSG: robotclient/Floats~%float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCoord-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCoord-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCoord)))
  'GetCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCoord)))
  'GetCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoord)))
  "Returns string type for a service object of type '<GetCoord>"
  "robotclient/GetCoord")