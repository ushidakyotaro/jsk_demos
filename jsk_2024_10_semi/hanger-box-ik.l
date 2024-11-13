#!/usr/bin/env roseus

;; PR2のモデルを読み込む
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(pr2-init)
;; PR2のインスタンスを作成
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))


;; 三角柱を作成（薄いハンガー形状を模擬）
(defun make-hanger ()
  (let ((hanger (make-prism 
                 (list (float-vector 0 0 0)       ;; 下の頂点
                       (float-vector 150 0 -100)    ;; 右上の頂点
                       (float-vector -150 0 -100))  ;; 左上の頂点
                 10)))  ;; 厚さ10mm
    
    ;; 三角柱の下の角に座標系を設定（把持の手前の位置）
    (send hanger :put :left-coords
          (make-cascoords
           :coords (send (send hanger :copy-worldcoords) 
                        :translate (float-vector 80 0 -50) ;; ハンガーの
                        )  ;; ハンガーの下端から相対位置
           :rot #2f((0 0 1)   ;; y軸方向を向くための回転行列
                    (-1 0 0)
                    (0 -1 0))
           :parent hanger))
    hanger))

;; ハンガーをグローバル変数として保持
(setq *hanger* (make-hanger))
;; 初期位置に配置
;(send *hanger* :translate (float-vector 800 100 1100))

(defun step1-init()
  (send *ri* :speak-jp "今日は何を着たい?")

  (send *pr2* :reset-pose)
  (send *pr2* :larm :collar-y :joint-angle 0)
  (send *pr2* :larm :shoulder-p :joint-angle 0)
  (send *pr2* :larm :shoulder-r :joint-angle 0)
  (send *pr2* :larm :elbow-p :joint-angle -90)
  (send *pr2* :larm :elbow-r :joint-angle 180)
  (send *pr2* :larm :wrist-p :joint-angle -30)
  (send *pr2* :larm :wrist-r :joint-angle 180)
  (send *pr2* :larm :gripper :joint-angle 40)
  (send *irtviewer* :draw-objects)


  ; これ書けばロボットが動く
  (send *ri* :stop-grasp :arms)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :speak-jp "準備完了しました")
)

(defun step2-approach ()
  (send *ri* :speak-jp "ハンガーに腕を伸ばすよ。")

  ;   ;; ハンガーの位置を*target-coords*に合わせて更新
  ; (when (boundp '*target-coords*)
  ;   (send *hanger* :move-to *target-coords* :world)
  ;   (send *irtviewer* :draw-objects))

  ;; 左腕で三角柱を掴む
  (send *pr2* :larm :inverse-kinematics
        (send (send *hanger* :get :left-coords) :copy-worldcoords)
        :rotation-axis :y)
  
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (unix:sleep 3)
  (send *ri* :speak-jp "ハンガーに手を伸ばしました。")


  ; ;; 左腕で三角柱を掴む - 逆運動学の結果を変数に保存
  ; (let ((ik-ret (send *pr2* :larm :inverse-kinematics
  ;                     (send (send *hanger* :get :left-coords) :copy-worldcoords)
  ;                     :rotation-axis t
  ;                     :debug-view t)))  ;; デバッグビューを有効に
    
  ;   (if ik-ret
  ;       (progn
  ;         (format t "IK succeeded: ~A~%" ik-ret)
  ;         (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)  ;; 5秒に延長
  ;         (send *ri* :wait-interpolation)
  ;         (unix:sleep 3))
  ;       (send *ri* :speak-jp "ハンガーに手が届きません。")))  ;; IKが解けない場合
  
)

(defun step3-grasp ()
  (send *ri* :speak-jp "ハンガーを掴むよ。")
  "ハンドを閉じる動作（シミュレーション用）"
  ;(send *pr2* :larm :gripper :joint-angle 0)
  (send *pr2* :larm :end-coords :assoc *hanger*)
  (send *ri* :start-grasp :larm)
  (send *irtviewer* :draw-objects)
  (unix:sleep 1))


(defun step4-lift ()
  (send *ri* :speak-jp "持ち上げるよ。")
  "斜め上に移動"
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *hanger* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector -100 0 150))  ;; 15cm上、10cm手前
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis t
        :debug-view nil
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *irtviewer* :draw-objects)
  )

(defun step5-pull ()
  (send *ri* :speak-jp "手元に引くよ。")
  "手元に引く"
  (send *pr2*
        :inverse-kinematics (send (send (send *hanger* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector -100 0 50))  ;; さらに30cm手前
        ;:move-target (send *hanger* :get :left-coords)
        ;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis t
        :debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *irtviewer* :draw-objects))

(defun step6-release ()
  (send *ri* :speak-jp "離すよ。")
  "ハンドを開く動作（シミュレーション用）"
  ;(send *pr2* :larm :gripper :joint-angle 40)

  ; シミュレーションでハンガーの関連付けを解除
  (send *pr2* :larm :end-coords :dissoc *hanger*)
  (send *ri* :stop-grasp :larm)
  (send *irtviewer* :draw-objects)
)

;ハンガーを把持しようとしているかどうかのフラグ
(setq *found-hanger* nil)
(setq *is-moving* nil)

;; コールバック関数
(defun box-cb (msg)

  (dolist (box (send msg :boxes))
    (when (member (send box :label) '(566 615 469 923))
      (ros::ros-info "hook is detected")
      ;(send *ri* :speak-jp "ハンガー発見！")
      (setq *target-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
      (setq *target-dimensions* (send box :dimensions))
      
      ;; 座標を表示
      (format t "Target coords: x=~A y=~A z=~A~%"
              (elt (send *target-coords* :worldpos) 0)
              (elt (send *target-coords* :worldpos) 1)
              (elt (send *target-coords* :worldpos) 2))
      
      (when (and (< (elt (send *target-coords* :worldpos) 2) 1800)
                 (> (elt (send *target-coords* :worldpos) 2) 0)
                 (> (elt (send *target-coords* :worldpos) 0) 500)
                 (< (elt (send *target-coords* :worldpos) 0) 1500)
                 (> (elt (send *target-coords* :worldpos) 1) -1500)
                 (< (elt (send *target-coords* :worldpos) 1) 1500))
        
        (ros::ros-info "target is in reachable area")
        (send *ri* :speak-jp "ハンガーが手の届く範囲にありそうます。")

        ;; ハンガーの位置を更新して確認
        (send *hanger* :move-to *target-coords* :world)
        (format t "Updated hanger position: ~A~%" (send *hanger* :worldpos))
        (send *irtviewer* :draw-objects)
        (unix:sleep 1)  ;; 位置更新を確認するための待機
        (setq *found-hanger* t)))))  ;; 処理が完了したらループを抜ける

(defun grasp-hanger ()
  (setq *is-moving* t)
  (unwind-protect
      (progn
        (step2-approach)
        (step3-grasp)
        (unix:sleep 2)
        ;(step4-lift)
        ;(step5-pull)
        (step6-release))
    (setq *is-moving* nil)))

(defun main ()
  (ros::ros-info "start main loop")
  (objects (list *pr2* *hanger*))
  (step1-init)
  (unix:sleep 1)

  (ros::ros-info "start searching hanger")
  (send *ri* :speak-jp "ハンガーを探します。")
  (ros::subscribe "/synchronized_detic_label_boxes"
                jsk_recognition_msgs::BoundingBoxArray #'box-cb)

  (do-until-key
  (when *found-hanger*
    (return))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  (ros::spin-once))

  (when *found-hanger*
    (send *ri* :speak-jp "ハンガーを掴みに行きます。")
    (grasp-hanger)
    (send *ri* :speak-jp "終了しました。")
    )
)

;; メイン関数を実行
(main)



