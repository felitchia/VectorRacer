(load "datastructures.lisp")
(load "auxfuncs.lisp")
; (load "datastructures.fas")
; (load "auxfuncs.fas")



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
  (let ((successors nil))
    (dolist (act (possible-actions) successors)
      (let ((new-state (nextState st act)))
	(if (not (member new-state successors :test #'equalp))
	    (push new-state successors))))))

;;; limdepthfirstsearch 
(defun limdepthfirstsearch (problem lim &key cutoff?)
  "limited depth first search
     st - initial state
     problem - problem information
     lim - depth limit"
  (labels ((limdepthfirstsearch-aux (node problem lim)
	     (if (isGoalp (node-state node))
		 (solution node)
		 (if (zerop lim)
		     :cutoff
		     (let ((cutoff? nil))
		       (dolist (new-state (nextStates (node-state node)))
			 (let* ((new-node (make-node :parent node :state new-state))
				(res (limdepthfirstsearch-aux new-node problem (1- lim))))
			   (if (eq res :cutoff)
			       (setf cutoff? :cutoff)
			       (if (not (null res))
				   (return-from limdepthfirstsearch-aux res)))))
		       (values cutoff?))))))
    (let ((res (limdepthfirstsearch-aux (make-node :parent nil :state (problem-initial-state problem))
					problem
					lim)))
      (if (eq res :cutoff)
	  (if cutoff?
	      :cutoff
	      nil)
	  res))))
				      

;iterlimdepthfirstsearch
(defun iterlimdepthfirstsearch (problem &key (lim most-positive-fixnum))
  "limited depth first search
     st - initial state
     problem - problem information
     lim - limit of depth iterations"
  (let ((i 0))
    (loop
      (let ((res (limdepthfirstsearch problem i :cutoff? T)))
	(when (and res (not (eq res :cutoff)))
	  (return res))
	(incf i)
	(if (> i lim)
	    (return nil))))))
	

(defun isEndPoint (pos track)
	(integerp (position pos (track-endpositions track) :test #'equal )))
	
;; Solution of phase 3

;; Heuristic
(defun compute-heuristic(st)
	(let ((dist most-positive-fixnum)
		(aux 0)
		(pos (state-pos st)))
		(if (isObstaclep pos (state-track st)) 
			most-positive-fixnum
			(progn 
				(dolist (endposition (track-endpositions (state-track st)))
					(setf aux (max (abs (- (first endposition) (first pos))) (abs (- (second endposition) (second pos)))))
					(setf dist (min dist aux))
				)
				dist
			)
		)
	)
)


;;; A*
(defun a* (problem)
		;recebe problema (initial-state, fn-nextStates, fn-isGoal, fn-h)
		;devolve lista de estados de inicial ao objectivo ou NIL cc
		;estrutura node (parent, state, f, g, h)
		
		; initialize the open list
		; initialize the closed list
		; put the starting node on the open list (you can leave its f at zero)

		; while the open list is not empty
			; find the node with the least f on the open list, call it "q"
			; pop q off the open list
			; generate q's 8 successors and set their parents to q
			; for each successor
				; if successor is the goal, stop the search
				; successor.g = q.g + distance between successor and q
				; successor.h = distance from goal to successor
				; successor.f = successor.g + successor.h

				; if a node with the same position as successor is in the OPEN list \
					; which has a lower f than successor, skip this successor
				; if a node with the same position as successor is in the CLOSED list \ 
					; which has a lower f than successor, skip this successor
				; otherwise, add the node to the open list
			; end
			; push q on the closed list
		; end
		
		
	(let* ((node (make-node :state (problem-initial-state problem) :g 0))
		 (closedlist nil)
		 (openlist (list node))
		 (generated nil))

		(block a*-loop
			(setf (node-h node) (funcall (problem-fn-h problem) (node-state node)))
			(setf (node-f node) (node-h node))
			(loop 
				(when (equal openlist nil) (return-from a*-loop nil))
				(setf node (find-lowest-f openlist))
				(setf openlist (remove node openlist))
				(if (funcall (problem-fn-isGoal problem) (node-state node)) 
					(return-from a*-loop (solution node)))
				(setf closedlist (append closedlist (list node)))
				(setf generated (childNodes node (funcall (problem-fn-nextStates problem) (node-state node)) problem))
				(dolist (child generated)
					;quando o estado do chlild nao esta na lista de abertos nem fechados adicionamos esse no na lista de abertos
					(when (or (not (member (node-state child) (mapcar #'node-state openlist))) (not (member (node-state child) (mapcar #'node-state closedlist))))
						(setf openlist (append (list child)  openlist))
					)
					(dolist (auxnode openlist) 
						;se o estado de child ja estiver na lista de abertos com um f maior, entao substituimos esse no na lista de abertos pelo child
						(when (and (eq (node-state child) (node-state auxnode)) (> (node-f auxnode) (node-f child)))
							(progn (setf openlist (substitute child auxnode openlist))
							)
						)
					)
				)

			)	
		)
	)
)


(defun find-lowest-f (a*list)
	(let ((f (node-f (first a*list)))
		  (node (first a*list)))
		(dolist (el a*list)
			(if (< (node-f el) f)
					(progn
						(setf f (node-f el))
						(setf node el)
					)
			)
		)
	node)
)

(defun childNodes (parent nodes problem)
  (let ((retlist '())
    (newnode nil))
    (dolist (no nodes)
		(setf newnode (make-node :parent parent :state no))
		(setf (node-h newnode) (funcall (problem-fn-h problem) (node-state newnode)))
		(setf (node-g newnode) (+ (getCost (node-state newnode) problem) (node-g parent)))
		(setf (node-f newnode) (+ (node-h newnode) (node-g newnode)))
		(setf retlist (append retlist (list newnode)))
	)
  retlist)
)

(defun getCost (st problem)
	(let ((cost 0))
		(cond 
			((isObstaclep (state-pos st) (state-track st))(setf cost 20))
			((funcall(problem-fn-isGoal problem) st)(setf cost -100))
			(t (setf cost 1))
		)
	cost)
)

(defun solution (node)
  (let ((seq-states nil))
    (loop 
      (when (null node)
	(return))
      (push (node-state node) seq-states)
      (setf node (node-parent node)))
    (values seq-states)
	)
)