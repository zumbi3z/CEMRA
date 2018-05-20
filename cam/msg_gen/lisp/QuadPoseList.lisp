; Auto-generated. Do not edit!


(cl:in-package cam-msg)


;//! \htmlinclude QuadPoseList.msg.html

(cl:defclass <QuadPoseList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector cam-msg:QuadPose)
   :initform (cl:make-array 0 :element-type 'cam-msg:QuadPose :initial-element (cl:make-instance 'cam-msg:QuadPose))))
)

(cl:defclass QuadPoseList (<QuadPoseList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuadPoseList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuadPoseList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cam-msg:<QuadPoseList> is deprecated: use cam-msg:QuadPoseList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <QuadPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cam-msg:header-val is deprecated.  Use cam-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <QuadPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cam-msg:poses-val is deprecated.  Use cam-msg:poses instead.")
  (poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuadPoseList>) ostream)
  "Serializes a message object of type '<QuadPoseList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuadPoseList>) istream)
  "Deserializes a message object of type '<QuadPoseList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cam-msg:QuadPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuadPoseList>)))
  "Returns string type for a message object of type '<QuadPoseList>"
  "cam/QuadPoseList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuadPoseList)))
  "Returns string type for a message object of type 'QuadPoseList"
  "cam/QuadPoseList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuadPoseList>)))
  "Returns md5sum for a message object of type '<QuadPoseList>"
  "e14cd22ab2349681fc355becb7328751")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuadPoseList)))
  "Returns md5sum for a message object of type 'QuadPoseList"
  "e14cd22ab2349681fc355becb7328751")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuadPoseList>)))
  "Returns full string definition for message of type '<QuadPoseList>"
  (cl:format cl:nil "Header header~%QuadPose[] poses~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cam/QuadPose~%string name~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%int8 pose_updated~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuadPoseList)))
  "Returns full string definition for message of type 'QuadPoseList"
  (cl:format cl:nil "Header header~%QuadPose[] poses~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cam/QuadPose~%string name~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%int8 pose_updated~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuadPoseList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuadPoseList>))
  "Converts a ROS message object to a list"
  (cl:list 'QuadPoseList
    (cl:cons ':header (header msg))
    (cl:cons ':poses (poses msg))
))
