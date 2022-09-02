(define (domain temporal)
  (:requirements :equality :durative-actions :derived-predicates) ; :typing :action-costs :conditional-effects :numeric-fluents :disjunctive-preconditions
  (:predicates
    ;(Time ?t)
    ;(Duration ?dt)
    ;(Sum ?t1 ?dt ?t2)
    ;(GE ?t1 ?t2)

    (Food ?f)
    (Stove ?s) ; TODO: Oven

    (CookDuration ?dt ?f ?s)
    (Cooking ?t ?f ?s)

    (AtTime ?t)
    (Cooked ?f)
    (Locked ?s)
    (On ?s)
    ;(Premature ?t)
    ;(Invalid)
    ;(CanWait)
  )
  (:functions
    (Elapsed ?dt)
    (Difference ?t2 ?t1)
    (GasCost ?s)
  )

  (:action turn-on
    :parameters (?s)
    :precondition (and (Stove ?s)
                       (PowerOn)
                       ;(not (On ?s))
                  )
    :effect (and (On ?s)
                 ;(increase (total-cost) 0)
                 ;(increase (total-cost) (GasCost ?s))
            )
  )
  (:action turn-off
    :parameters (?s)
    :precondition (and (Stove ?s)
                       ;(On ?s)
                  )
    :effect (and (not (On ?s))
                 ;(increase (total-cost) 0)
                 ;(increase (total-cost) (GasCost ?s))
            )
  )

  (:durative-action power-on ; TODO: exogenous process
   :parameters ()
   :duration (= ?duration (power-on_duration))
   :condition (and
     ;(at start (_Noop))
     (at start (not (Advanced)))
     ;(at start (not (PowerOn)))
     (at start (Controllable))
   )
   :effect (and
     (at end (PowerOn))
     (at start (not (Controllable))) ; TODO: hack to make Controllable a fluent
   )
  )
  ; TODO: schedule an effect to start at a particular time

  (:durative-action cook
   :parameters (?f ?s)
   :duration (= ?duration (cook_duration ?f ?s)) ; TODO: expressions
   :condition (and
     (at start (Food ?f))
     (at start (Stove ?s))
     (at start (not (Locked ?f)))
     (at start (not (Locked ?s)))
     (at start (On ?s))

     ;(at start (and (Food ?f) (Stove ?s) (On ?s)))
     ;(at start (not (and (Locked ?f) (Locked ?s))))
     ;(at start (and (Food ?f) (Stove ?s) (On ?s)
     ;     (not (and (Locked ?f) (Locked ?s)))))

     (over all (On ?s))
   )
   :effect (and
     ;(at start (On ?s))
     (at start (Locked ?f))
     (at start (Locked ?s))

     ;(at start (increase (total-cost) 1)) ; Many temporal planners don't support costs
	 ;(at start (decrease (Gas) 1)) ; Numeric effects not currently supported
	 ;(at start (assign (Test) 1)) ; Not supported
	 ;(at start (scale-up (Test) 1)) ; Not supported
	 ;(at start (scale-down (Test) 1)) ; Not supported

     ;(at end (not (On ?s)))
     (at end (not (Locked ?f)))
     (at end (not (Locked ?s)))
     (at end (Cooked ?f))
   )
  )
)