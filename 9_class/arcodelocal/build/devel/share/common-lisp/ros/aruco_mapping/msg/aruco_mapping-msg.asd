
(cl:in-package :asdf)

(defsystem "aruco_mapping-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ArucoMarker" :depends-on ("_package_ArucoMarker"))
    (:file "_package_ArucoMarker" :depends-on ("_package"))
  ))