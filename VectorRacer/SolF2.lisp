;(load "datastructures.lisp")
;(load "auxfuncs.lisp")
(load "datastructures.fas")
(load "auxfuncs.fas")

;;; TAI position
(defun make-pos (c l)
  (list c l))
(defun pos-l (pos)
  (first pos))
(defun pos-c (pos)
  (second pos))

;;; TAI acceleration
(defun make-acce (c l)
  (list c l))
(defun acce-l (pos)
  (first pos))
(defun acce-c (pos)
  (second pos))

;;; TAI velocity
(defun make-vel (c l)
  (list c l))
(defun vel-l (pos)
  (first pos))
(defun vel-c (pos)
  (second pos))


;; Solution of phase 1

(defun getTrackContent (pos track)
  (nth (pos-c pos) (nth (pos-l pos) (track-env track))))

;; Pedir 0,4
(defun isObstaclep (pos track)
  "check if the position pos is an obstacle"
  (or (< (pos-l pos) 0) (< (pos-c pos) 0)
      (>= (pos-l pos) (pos-l (track-size track)))
      (>= (pos-c pos) (pos-c (track-size track)))
      (null (getTrackContent pos track))))

;; Pedir 0,4
(defun isGoalp (st) 
  "check if st is a solution of the problem"
  (let ((current-position (state-pos st))
	(track (state-track st)))
    (and (member current-position (track-endpositions track) :test #'equalp)
	 T)))

;; Pedir 1,2
(defun nextState (st act)
  "generate the nextState after state st and action act from prolem"
  (let ((new-state (make-state :action act :track (state-track st))))
    (setf (state-vel new-state)
	  (make-vel (+ (vel-l (state-vel st)) (acce-l act))
		    (+ (vel-c (state-vel st)) (acce-c act))))
    (setf (state-pos new-state)
	  (make-pos (+ (pos-l (state-pos st)) (vel-l (state-vel new-state)))
		    (+ (pos-c (state-pos st)) (vel-c (state-vel new-state)))))
    (setf (state-cost new-state)
	  (cond ((isGoalp new-state) -100)
		((isObstaclep (state-pos new-state) (state-track new-state)) 20)
		(T 1)))
    (when (= (state-cost new-state) 20)
      (setf (state-vel new-state) (make-vel 0 0))
      (setf (state-pos new-state) (make-pos (pos-l (state-pos st))
					    (pos-c (state-pos st)))))
    (values new-state)))



;; Solution of phase 2

;;; Pedir 
(defun nextStates (st)
  "generate all possible next states"
  (let* ((actionlist (reverse (possible-actions)))
     (nbract (length actionlist))
     (stts (make-list nbract :initial-element st)))
  (mapcar #'nextState stts actionlist)))

´;;;returns a list with nodes whose parent is parent
(defun setParents (parent nodes)
  (let* ((lengthNode (length nodes))
    (parents (make-list lengthNode :initial-element parent)))
  (mapcar #'make-node :state nodes :parent parents)))
)
;;; limdepthfirstsearch 
(defun limdepthfirstsearch (problem lim &key cutoff?)
  "limited depth first search
     st - initial state
     problem - problem information
     lim - depth limit"
	;(list (make-node :state (problem-initial-state problem))) 
    (let* ((node (make-node :state (problem-initial-state problem)))
        (chosen (list node)) 
        (expanded '())
        (generated '())
        (depth 0)
        (cuttoff nil))
      (block depthsearch
        (while (not (equal chosen nil))
          (push expanded node)
          (if (isGoalp node-state)
            (return-from depthsearch chosen)
            (progn
              (pop generated)
              (if (<= depth lim)
                (progn
                  (append (setParents (nextStates node-state)) generated)
                  (pop expanded)
                  (append chosen (list node))
                  (append expanded (list node))
                  (setf depth (1+ depth)))
                (progn 
                  (setf cutoff t)
                  (while (not (equal (node-parent) (last chosen)))
                    (progn 
                      (remove chosen :from-end t)
                      (setf depth (1- depth))          
                )
                        
              )
            )

          );close if(<= depth)
          (setf node (first generated)) 
    )

  )
				      

;iterlimdepthfirstsearch
(defun iterlimdepthfirstsearch (problem &key (lim most-positive-fixnum))
  "limited depth first search
     st - initial state
     problem - problem information
     lim - limit of depth iterations"
	(list (make-node :state (problem-initial-state problem))) )

