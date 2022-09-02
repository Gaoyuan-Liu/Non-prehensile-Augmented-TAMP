;; Auto-generated. Do not edit!


(when (boundp 'franka_gripper::GraspEpsilon)
  (if (not (find-package "FRANKA_GRIPPER"))
    (make-package "FRANKA_GRIPPER"))
  (shadow 'GraspEpsilon (find-package "FRANKA_GRIPPER")))
(unless (find-package "FRANKA_GRIPPER::GRASPEPSILON")
  (make-package "FRANKA_GRIPPER::GRASPEPSILON"))

(in-package "ROS")
;;//! \htmlinclude GraspEpsilon.msg.html


(defclass franka_gripper::GraspEpsilon
  :super ros::object
  :slots (_inner _outer ))

(defmethod franka_gripper::GraspEpsilon
  (:init
   (&key
    ((:inner __inner) 0.0)
    ((:outer __outer) 0.0)
    )
   (send-super :init)
   (setq _inner (float __inner))
   (setq _outer (float __outer))
   self)
  (:inner
   (&optional __inner)
   (if __inner (setq _inner __inner)) _inner)
  (:outer
   (&optional __outer)
   (if __outer (setq _outer __outer)) _outer)
  (:serialization-length
   ()
   (+
    ;; float64 _inner
    8
    ;; float64 _outer
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _inner
       (sys::poke _inner (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _outer
       (sys::poke _outer (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _inner
     (setq _inner (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _outer
     (setq _outer (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get franka_gripper::GraspEpsilon :md5sum-) "95b2c5464a6f679bd1dacaf86414f095")
(setf (get franka_gripper::GraspEpsilon :datatype-) "franka_gripper/GraspEpsilon")
(setf (get franka_gripper::GraspEpsilon :definition-)
      "float64 inner # [m]
float64 outer # [m]

")



(provide :franka_gripper/GraspEpsilon "95b2c5464a6f679bd1dacaf86414f095")


