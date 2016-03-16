; Auto-generated. Do not edit!


(cl:in-package robotclient-msg)


;//! \htmlinclude Floats.msg.html

(cl:defclass <Floats> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Floats (<Floats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Floats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Floats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotclient-msg:<Floats> is deprecated: use robotclient-msg:Floats instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Floats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotclient-msg:data-val is deprecated.  Use robotclient-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Floats>) ostream)
  "Serializes a message object of type '<Floats>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Floats>) istream)
  "Deserializes a message object of type '<Floats>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Floats>)))
  "Returns string type for a message object of type '<Floats>"
  "robotclient/Floats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Floats)))
  "Returns string type for a message object of type 'Floats"
  "robotclient/Floats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Floats>)))
  "Returns md5sum for a message object of type '<Floats>"
  "3a423cd43be5532c6a3def568afe4aec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Floats)))
  "Returns md5sum for a message object of type 'Floats"
  "3a423cd43be5532c6a3def568afe4aec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Floats>)))
  "Returns full string definition for message of type '<Floats>"
  (cl:format cl:nil "float32[2] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Floats)))
  "Returns full string definition for message of type 'Floats"
  (cl:format cl:nil "float32[2] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Floats>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Floats>))
  "Converts a ROS message object to a list"
  (cl:list 'Floats
    (cl:cons ':data (data msg))
))
