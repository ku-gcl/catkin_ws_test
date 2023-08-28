
(cl:in-package :asdf)

(defsystem "test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CustomState" :depends-on ("_package_CustomState"))
    (:file "_package_CustomState" :depends-on ("_package"))
  ))