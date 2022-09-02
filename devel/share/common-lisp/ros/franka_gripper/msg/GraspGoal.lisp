; Auto-generated. Do not edit!


(cl:in-package franka_gripper-msg)


;//! \htmlinclude GraspGoal.msg.html

(cl:defclass <GraspGoal> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (epsilon
    :reader epsilon
    :initarg :epsilon
    :type franka_gripper-msg:GraspEpsilon
    :initform (cl:make-instance 'franka_gripper-msg:GraspEpsilon))
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (force
    :reader force
    :initarg :force
    :type cl:float
    :initform 0.0))
)

(cl:defclass GraspGoal (<GraspGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_gripper-msg:<GraspGoal> is deprecated: use franka_gripper-msg:GraspGoal instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GraspGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_gripper-msg:width-val is deprecated.  Use franka_gripper-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'epsilon-val :lambda-list '(m))
(cl:defmethod epsilon-val ((m <GraspGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_gripper-msg:epsilon-val is deprecated.  Use franka_gripper-msg:epsilon instead.")
  (epsilon m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <GraspGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_gripper-msg:speed-val is deprecated.  Use franka_gripper-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <GraspGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_gripper-msg:force-val is deprecated.  Use franka_gripper-msg:force instead.")
  (force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspGoal>) ostream)
  "Serializes a message object of type '<GraspGoal>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'epsilon) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspGoal>) istream)
  "Deserializes a message object of type '<GraspGoal>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'epsilon) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspGoal>)))
  "Returns string type for a message object of type '<GraspGoal>"
  "franka_gripper/GraspGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspGoal)))
  "Returns string type for a message object of type 'GraspGoal"
  "franka_gripper/GraspGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspGoal>)))
  "Returns md5sum for a message object of type '<GraspGoal>"
  "627a0f0b10ad0c919fbd62b0b3427e63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspGoal)))
  "Returns md5sum for a message object of type 'GraspGoal"
  "627a0f0b10ad0c919fbd62b0b3427e63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspGoal>)))
  "Returns full string definition for message of type '<GraspGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64 width # [m]~%GraspEpsilon epsilon~%float64 speed # [m/s]~%float64 force # [N]~%~%================================================================================~%MSG: franka_gripper/GraspEpsilon~%float64 inner # [m]~%float64 outer # [m]~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspGoal)))
  "Returns full string definition for message of type 'GraspGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64 width # [m]~%GraspEpsilon epsilon~%float64 speed # [m/s]~%float64 force # [N]~%~%================================================================================~%MSG: franka_gripper/GraspEpsilon~%float64 inner # [m]~%float64 outer # [m]~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspGoal>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'epsilon))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspGoal
    (cl:cons ':width (width msg))
    (cl:cons ':epsilon (epsilon msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':force (force msg))
))
