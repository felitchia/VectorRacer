
;;; These functions, and any other ones needed must be implemented

;;; Utilizar estes includes para os testes na versao local
;;; comentar antes de submeter
;(load "datastructures.lisp")
;(load "auxfuncs.lisp")

;;; Utilizar estes includes para a versao a submeter
; tirar o comentario antes de submeter
(load "datastructures.fas")
(load "auxfuncs.fas")

(defun getTrackElem (pos track)
	(nth (second pos) (nth (car pos) (track-env track))))

(defun isObstaclep (pos track) 
	"check if there is an obstacle at position pos of the track"
	(not (getTrackElem pos track)))

(defun isEndPoint (pos track)
	(integerp (position pos (track-endpositions track) :test #'equal )))

(defun isGoalp (st) 
  "check if st is a goal state"
  (isEndPoint (state-pos st) (state-track st)))


(defun updateVel (act vel pos track)
	(if (isObstaclep pos track)
		pos
		(mapcar #'+ vel act)))

(defun updatePos (act vel pos track)
	(if (isObstaclep pos track)
		'(0 0)
		(mapcar #'+ pos vel act)))

(defun cost (pos track)
	(cond ((isEndPoint pos track) -100)
			((isObstaclep pos track) 20)
			((or (not (isObstaclep pos track)) (not (isEndPoint pos track))) 1)))


(defun nextState (st act)
  "generate the nextState after state st and action act"
  (let* (( p (updatePos act (state-vel st) (state-pos st) (state-track st)))
  		(v (updateVel act (state-vel st) (state-pos st) (state-track st)))
  		(tr (state-track st))	
  		(o (state-other st))
  		(c (cost p (state-track st))))

	  (make-STATE :POS p
		      :VEL v
		      :ACTION act
		      :COST c 
		      :TRACK tr
		      :OTHER o)))



