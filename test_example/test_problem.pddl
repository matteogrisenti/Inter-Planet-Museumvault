(define (problem move-robot)
  (:domain museum-test)
  (:objects
    curator - robot
    hall-a hall-b - location
  )

  (:init
    (at curator hall-a)
    (connected hall-a hall-b)
  )

  (:goal (and (at curator hall-b)))
)