
(cl:in-package :asdf)

(defsystem "robotclient-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :robotclient-msg
)
  :components ((:file "_package")
    (:file "RotateRobot" :depends-on ("_package_RotateRobot"))
    (:file "_package_RotateRobot" :depends-on ("_package"))
    (:file "MoveRobot" :depends-on ("_package_MoveRobot"))
    (:file "_package_MoveRobot" :depends-on ("_package"))
    (:file "GetCoord" :depends-on ("_package_GetCoord"))
    (:file "_package_GetCoord" :depends-on ("_package"))
  ))