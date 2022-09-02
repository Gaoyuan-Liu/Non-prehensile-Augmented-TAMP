(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:constants @sink @stove)
  (:predicates
    (Arm ?a)
    (Stackable ?o ?r)
    (Sink ?r)
    (Stove ?r)
    (Type ?t ?b)

    (Pose ?o ?p)
    (Grasp ?o ?g)
    (Kin ?a ?o ?p ?g ?t) ; comment ?t
    (BaseMotion ?q1 ?t ?q2)
    (ArmMotion ?a ?q1 ?t ?q2)
    (Supported ?o ?p ?r)
    ; (BTraj ?t) ; commented the line
    (ATraj ?t) ; commented the line

    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)
    ; (CFreeTrajPose ?t ?o2 ?p2)
    ; (CFreeTrajGraspPose ?t ?a ?o1 ?g1 ?o2 ?p2)

    (AtPose ?o ?p)
    (AtGrasp ?a ?o ?g)
    (HandEmpty ?a)
    ; (AtBConf ?q)
    (AtAConf ?a ?q)
    (CanMove)
    (Cleaned ?o)
    (Cooked ?o)

    (On ?o ?r)
    (On2 ?o1 ?o2 ?r)
    (On3 ?o1 ?o2 ?o3 ?r)
    (On4 ?o1 ?o2 ?o3 ?o4 ?r)
    (On5 ?o1 ?o2 ?o3 ?o4 ?o5 ?r)

    (Holding ?a ?o)
    ; (UnsafePose ?o ?p)
    (UnsafeApproach ?o ?p ?g)
    (UnsafeATraj ?t)
    (UnsafeBTraj ?t)
  )
  (:functions
    (Distance ?q1 ?q2)
    (MoveCost ?t)
    (PickCost)
    (PlaceCost)
  )



  (:action pick
    :parameters (?a ?o ?p ?g ?q ?t) ; removed ?t
    :precondition (and (Kin ?a ?o ?p ?g ?t) ; removed ?t
                       (AtPose ?o ?p) (HandEmpty ?a) 
                       (not (UnsafeApproach ?o ?p ?g))
                      ;  (not (UnsafeATraj ?t)) ; comment this line
                  )
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 )
  )
  (:action place
    :parameters (?a ?o ?p ?g ?q ?t) ; removed ?t
    :precondition (and (Kin ?a ?o ?p ?g ?t) ; removed ?t
                       (AtGrasp ?a ?o ?g)
                      ;  (not (UnsafePose ?o ?p))
                      ;  (not (UnsafeApproach ?o ?p ?g))
                      ;  (not (UnsafeATraj ?t)) ; comment this line
                  )
    :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
                 (not (AtGrasp ?a ?o ?g))
                 )
  )



  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )

  (:derived (On2 ?o1 ?o2 ?r)
    (exists (?p1 ?p2) (and (Supported ?o1 ?p1 ?r)
                               (Supported ?o2 ?p2 ?r)
                               (AtPose ?o1 ?p1)
                               (AtPose ?o2 ?p2)))
  )

  (:derived (On3 ?o1 ?o2 ?o3 ?r)
    (exists (?p1 ?p2 ?p3) (and (Supported ?o1 ?p1 ?r)
                               (Supported ?o2 ?p2 ?r)
                               (Supported ?o3 ?p3 ?r)
                               (AtPose ?o1 ?p1)
                               (AtPose ?o2 ?p2)
                               (AtPose ?o3 ?p3)))
  )

  (:derived (On4 ?o1 ?o2 ?o3 ?o4 ?r)
    (exists (?p1 ?p2 ?p3 ?p4) (and (Supported ?o1 ?p1 ?r)
                               (Supported ?o2 ?p2 ?r)
                               (Supported ?o3 ?p3 ?r)
                               (Supported ?o4 ?p4 ?r)
                               (AtPose ?o1 ?p1)
                               (AtPose ?o2 ?p2)
                               (AtPose ?o3 ?p3)
                               (AtPose ?o4 ?p4)))
  )

  (:derived (On5 ?o1 ?o2 ?o3 ?o4 ?o5 ?r)
    (exists (?p1 ?p2 ?p3 ?p4 ?p5) (and (Supported ?o1 ?p1 ?r)
                               (Supported ?o2 ?p2 ?r)
                               (Supported ?o3 ?p3 ?r)
                               (Supported ?o4 ?p4 ?r)
                               (Supported ?o5 ?p5 ?r)
                               (AtPose ?o1 ?p1)
                               (AtPose ?o2 ?p2)
                               (AtPose ?o3 ?p3)
                               (AtPose ?o4 ?p4)
                               (AtPose ?o5 ?p5)))
  )


  (:derived (Holding ?a ?o)
    (exists (?g) (and (Arm ?a) (Grasp ?o ?g)
                      (AtGrasp ?a ?o ?g)))
  )

  ; (:derived (UnsafePose ?o ?p)
  ;   (exists (?o2 ?p2) (and (Pose ?o ?p) (Pose ?o2 ?p2) (not (= ?o ?o2))
  ;                          (not (CFreePosePose ?o ?p ?o2 ?p2))
  ;                          (AtPose ?o2 ?p2)))
  ; )

  (:derived (UnsafeApproach ?o ?p ?g)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Grasp ?o ?g) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreeApproachPose ?o ?p ?g ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )


  ; commented this section
  ; (:derived (UnsafeATraj ?t)
  ;   (exists (?o2 ?p2) (and (ATraj ?t) (Pose ?o2 ?p2)
  ;                          (not (CFreeTrajPose ?t ?o2 ?p2))
  ;                          (AtPose ?o2 ?p2)))
  ; )





 
)