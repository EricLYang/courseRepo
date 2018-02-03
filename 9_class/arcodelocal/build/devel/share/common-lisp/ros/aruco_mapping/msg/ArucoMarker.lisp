; Auto-generated. Do not edit!


(cl:in-package aruco_mapping-msg)


;//! \htmlinclude ArucoMarker.msg.html

(cl:defclass <ArucoMarker> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (marker_visibile
    :reader marker_visibile
    :initarg :marker_visibile
    :type cl:boolean
    :initform cl:nil)
   (num_of_visible_markers
    :reader num_of_visible_markers
    :initarg :num_of_visible_markers
    :type cl:integer
    :initform 0)
   (global_camera_pose
    :reader global_camera_pose
    :initarg :global_camera_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (marker_ids
    :reader marker_ids
    :initarg :marker_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (global_marker_poses
    :reader global_marker_poses
    :initarg :global_marker_poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass ArucoMarker (<ArucoMarker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArucoMarker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArucoMarker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aruco_mapping-msg:<ArucoMarker> is deprecated: use aruco_mapping-msg:ArucoMarker instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArucoMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_mapping-msg:header-val is deprecated.  Use aruco_mapping-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'marker_visibile-val :lambda-list '(m))
(cl:defmethod marker_visibile-val ((m <ArucoMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_mapping-msg:marker_visibile-val is deprecated.  Use aruco_mapping-msg:marker_visibile instead.")
  (marker_visibile m))

(cl:ensure-generic-function 'num_of_visible_markers-val :lambda-list '(m))
(cl:defmethod num_of_visible_markers-val ((m <ArucoMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_mapping-msg:num_of_visible_markers-val is deprecated.  Use aruco_mapping-msg:num_of_visible_markers instead.")
  (num_of_visible_markers m))

(cl:ensure-generic-function 'global_camera_pose-val :lambda-list '(m))
(cl:defmethod global_camera_pose-val ((m <ArucoMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_mapping-msg:global_camera_pose-val is deprecated.  Use aruco_mapping-msg:global_camera_pose instead.")
  (global_camera_pose m))

(cl:ensure-generic-function 'marker_ids-val :lambda-list '(m))
(cl:defmethod marker_ids-val ((m <ArucoMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_mapping-msg:marker_ids-val is deprecated.  Use aruco_mapping-msg:marker_ids instead.")
  (marker_ids m))

(cl:ensure-generic-function 'global_marker_poses-val :lambda-list '(m))
(cl:defmethod global_marker_poses-val ((m <ArucoMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_mapping-msg:global_marker_poses-val is deprecated.  Use aruco_mapping-msg:global_marker_poses instead.")
  (global_marker_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArucoMarker>) ostream)
  "Serializes a message object of type '<ArucoMarker>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'marker_visibile) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_of_visible_markers)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'global_camera_pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'marker_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'marker_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'global_marker_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'global_marker_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArucoMarker>) istream)
  "Deserializes a message object of type '<ArucoMarker>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'marker_visibile) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_of_visible_markers) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'global_camera_pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'marker_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'marker_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'global_marker_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'global_marker_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArucoMarker>)))
  "Returns string type for a message object of type '<ArucoMarker>"
  "aruco_mapping/ArucoMarker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArucoMarker)))
  "Returns string type for a message object of type 'ArucoMarker"
  "aruco_mapping/ArucoMarker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArucoMarker>)))
  "Returns md5sum for a message object of type '<ArucoMarker>"
  "e73493d4620efa2f38fe39e7896d4192")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArucoMarker)))
  "Returns md5sum for a message object of type 'ArucoMarker"
  "e73493d4620efa2f38fe39e7896d4192")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArucoMarker>)))
  "Returns full string definition for message of type '<ArucoMarker>"
  (cl:format cl:nil "std_msgs/Header header~%bool marker_visibile~%int32 num_of_visible_markers~%geometry_msgs/Pose global_camera_pose~%int32[] marker_ids~%geometry_msgs/Pose[] global_marker_poses~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArucoMarker)))
  "Returns full string definition for message of type 'ArucoMarker"
  (cl:format cl:nil "std_msgs/Header header~%bool marker_visibile~%int32 num_of_visible_markers~%geometry_msgs/Pose global_camera_pose~%int32[] marker_ids~%geometry_msgs/Pose[] global_marker_poses~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArucoMarker>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'global_camera_pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'marker_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'global_marker_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArucoMarker>))
  "Converts a ROS message object to a list"
  (cl:list 'ArucoMarker
    (cl:cons ':header (header msg))
    (cl:cons ':marker_visibile (marker_visibile msg))
    (cl:cons ':num_of_visible_markers (num_of_visible_markers msg))
    (cl:cons ':global_camera_pose (global_camera_pose msg))
    (cl:cons ':marker_ids (marker_ids msg))
    (cl:cons ':global_marker_poses (global_marker_poses msg))
))
