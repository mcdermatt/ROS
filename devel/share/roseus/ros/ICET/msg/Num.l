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
  :slots (_timestamp _restart _frame _status _true_transform ))

(defmethod ICET::Num
  (:init
   (&key
    ((:timestamp __timestamp) (instance ros::time :init))
    ((:restart __restart) nil)
    ((:frame __frame) 0)
    ((:status __status) "")
    ((:true_transform __true_transform) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _timestamp __timestamp)
   (setq _restart __restart)
   (setq _frame (round __frame))
   (setq _status (string __status))
   (setq _true_transform __true_transform)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:restart
   (&optional (__restart :null))
   (if (not (eq __restart :null)) (setq _restart __restart)) _restart)
  (:frame
   (&optional __frame)
   (if __frame (setq _frame __frame)) _frame)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:true_transform
   (&optional __true_transform)
   (if __true_transform (setq _true_transform __true_transform)) _true_transform)
  (:serialization-length
   ()
   (+
    ;; time _timestamp
    8
    ;; bool _restart
    1
    ;; int32 _frame
    4
    ;; string _status
    4 (length _status)
    ;; float32[] _true_transform
    (* 4    (length _true_transform)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;; bool _restart
       (if _restart (write-byte -1 s) (write-byte 0 s))
     ;; int32 _frame
       (write-long _frame s)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; float32[] _true_transform
     (write-long (length _true_transform) s)
     (dotimes (i (length _true_transform))
       (sys::poke (elt _true_transform i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _restart
     (setq _restart (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _frame
     (setq _frame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32[] _true_transform
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _true_transform (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _true_transform i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get ICET::Num :md5sum-) "93042447ed01f85739c5e6e8683f8ec7")
(setf (get ICET::Num :datatype-) "ICET/Num")
(setf (get ICET::Num :definition-)
      "time timestamp
bool restart
int32 frame
string status
float32[] true_transform

")



(provide :ICET/Num "93042447ed01f85739c5e6e8683f8ec7")


