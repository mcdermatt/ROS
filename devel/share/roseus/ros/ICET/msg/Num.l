;; Auto-generated. Do not edit!


(when (boundp 'ICET::Num)
  (if (not (find-package "ICET"))
    (make-package "ICET"))
  (shadow 'Num (find-package "ICET")))
(unless (find-package "ICET::NUM")
  (make-package "ICET::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass ICET::Num
  :super ros::object
  :slots (_timestamp _errors _ch1 _ch2 _status ))

(defmethod ICET::Num
  (:init
   (&key
    ((:timestamp __timestamp) (instance ros::time :init))
    ((:errors __errors) nil)
    ((:ch1 __ch1) 0.0)
    ((:ch2 __ch2) 0.0)
    ((:status __status) "")
    )
   (send-super :init)
   (setq _timestamp __timestamp)
   (setq _errors __errors)
   (setq _ch1 (float __ch1))
   (setq _ch2 (float __ch2))
   (setq _status (string __status))
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:errors
   (&optional (__errors :null))
   (if (not (eq __errors :null)) (setq _errors __errors)) _errors)
  (:ch1
   (&optional __ch1)
   (if __ch1 (setq _ch1 __ch1)) _ch1)
  (:ch2
   (&optional __ch2)
   (if __ch2 (setq _ch2 __ch2)) _ch2)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; time _timestamp
    8
    ;; bool _errors
    1
    ;; float32 _ch1
    4
    ;; float32 _ch2
    4
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;; bool _errors
       (if _errors (write-byte -1 s) (write-byte 0 s))
     ;; float32 _ch1
       (sys::poke _ch1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ch2
       (sys::poke _ch2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _errors
     (setq _errors (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _ch1
     (setq _ch1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ch2
     (setq _ch2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ICET::Num :md5sum-) "5c756fa6b1c2901279edb34f2bd952f7")
(setf (get ICET::Num :datatype-) "ICET/Num")
(setf (get ICET::Num :definition-)
      "time timestamp
bool errors
float32 ch1
float32 ch2
string status
")



(provide :ICET/Num "5c756fa6b1c2901279edb34f2bd952f7")


