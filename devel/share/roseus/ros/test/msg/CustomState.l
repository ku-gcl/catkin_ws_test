;; Auto-generated. Do not edit!


(when (boundp 'test::CustomState)
  (if (not (find-package "TEST"))
    (make-package "TEST"))
  (shadow 'CustomState (find-package "TEST")))
(unless (find-package "TEST::CUSTOMSTATE")
  (make-package "TEST::CUSTOMSTATE"))

(in-package "ROS")
;;//! \htmlinclude CustomState.msg.html


(defclass test::CustomState
  :super ros::object
  :slots (_theta1 _theta1_dot _theta2 _theta2_dot ))

(defmethod test::CustomState
  (:init
   (&key
    ((:theta1 __theta1) 0.0)
    ((:theta1_dot __theta1_dot) 0.0)
    ((:theta2 __theta2) 0.0)
    ((:theta2_dot __theta2_dot) 0.0)
    )
   (send-super :init)
   (setq _theta1 (float __theta1))
   (setq _theta1_dot (float __theta1_dot))
   (setq _theta2 (float __theta2))
   (setq _theta2_dot (float __theta2_dot))
   self)
  (:theta1
   (&optional __theta1)
   (if __theta1 (setq _theta1 __theta1)) _theta1)
  (:theta1_dot
   (&optional __theta1_dot)
   (if __theta1_dot (setq _theta1_dot __theta1_dot)) _theta1_dot)
  (:theta2
   (&optional __theta2)
   (if __theta2 (setq _theta2 __theta2)) _theta2)
  (:theta2_dot
   (&optional __theta2_dot)
   (if __theta2_dot (setq _theta2_dot __theta2_dot)) _theta2_dot)
  (:serialization-length
   ()
   (+
    ;; float64 _theta1
    8
    ;; float64 _theta1_dot
    8
    ;; float64 _theta2
    8
    ;; float64 _theta2_dot
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _theta1
       (sys::poke _theta1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta1_dot
       (sys::poke _theta1_dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta2
       (sys::poke _theta2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta2_dot
       (sys::poke _theta2_dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _theta1
     (setq _theta1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta1_dot
     (setq _theta1_dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta2
     (setq _theta2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta2_dot
     (setq _theta2_dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get test::CustomState :md5sum-) "a8d2dcedf46285bfb75a4965f00f0338")
(setf (get test::CustomState :datatype-) "test/CustomState")
(setf (get test::CustomState :definition-)
      "float64 theta1
float64 theta1_dot
float64 theta2
float64 theta2_dot
")



(provide :test/CustomState "a8d2dcedf46285bfb75a4965f00f0338")


