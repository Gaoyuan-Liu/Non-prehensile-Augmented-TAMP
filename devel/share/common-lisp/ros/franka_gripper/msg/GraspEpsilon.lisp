; Auto-generated. Do not edit!


(cl:in-package franka_gripper-msg)


;//! \htmlinclude GraspEpsilon.msg.html

(cl:defclass <GraspEpsilon> (roslisp-msg-protocol:ros-message)
  ((inner
    :reader inner
    :initarg :inner
    :type cl:float
    :initform 0.0)
   (outer
    :reader outer
    :initarg :outer
    :type cl:float
    :initform 0.0))
)

(cl:defclass GraspEpsilon (<GraspEpsilon>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspEpsilon>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspEpsilon)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_gripper-msg:<GraspEpsilon> is deprecated: use franka_gripper-msg:GraspEpsilon instead.")))

(cl:ensure-generic-function 'inner-val :lambda-list '(m))
(cl:defmethod inner-val ((m <GraspEpsilon>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_gripper-msg:inner-val is deprecated.  Use franka_gripper-msg:inner instead.")
  (inner m))

(cl:ensure-generic-function 'outer-val :lambda-list '(m))
(cl:defmethod outer-val ((m <GraspEpsilon>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_gripper-msg:outer-val is deprecated.  Use franka_gripper-msg:outer instead.")
  (outer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspEpsilon>) ostream)
  "Serializes a message object of type '<GraspEpsilon>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'inner))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'outer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspEpsilon>) istream)
  "Deserializes a message object of type '<GraspEpsilon>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inner) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'outer) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspEpsilon>)))
  "Returns string type for a message object of type '<GraspEpsilon>"
  "franka_gripper/GraspEpsilon")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspEpsilon)))
  "Returns string type for a message object of type 'GraspEpsilon"
  "franka_gripper/GraspEpsilon")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspEpsilon>)))
  "Returns md5sum for a message object of type '<GraspEpsilon>"
  "95b2c5464a6f679bd1dacaf86414f095")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspEpsilon)))
  "Returns md5sum for a message object of type 'GraspEpsilon"
  "95b2c5464a6f679bd1dacaf86414f095")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspEpsilon>)))
  "Returns full string definition for message of type '<GraspEpsilon>"
  (cl:format cl:nil "float64 inner # [m]~%float64 outer # [m]~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspEpsilon)))
  "Returns full string definition for message of type 'GraspEpsilon"
  (cl:format cl:nil "float64 inner # [m]~%float64 outer # [m]~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspEpsilon>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspEpsilon>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspEpsilon
    (cl:cons ':inner (inner msg))
    (cl:cons ':outer (outer msg))
))
