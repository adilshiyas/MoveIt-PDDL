(define (domain hanoi)
    (:requirements :strips :typing)

    (:types
        location
        peg disk - location
    )

    (:predicates
        (on ?d - disk ?x - location)
        (clear ?x - location)
        (smaller ?d2 - disk ?d1 - disk)
    )

    (:action move-to-peg
        :parameters (?d - disk ?from - location ?to - peg)
        :precondition (and
            (on ?d ?from)
            (clear ?d)
            (clear ?to)
        )
        :effect (and
            (not (on ?d ?from))
            (on ?d ?to)
            (clear ?from)
            (not (clear ?to))
        )
    )

    (:action move-to-disk
        :parameters(?d - disk ?from - location ?to - disk)
        :precondition (and
            (on ?d ?from)
            (clear ?d)
            (clear ?to)
            (smaller ?d ?to)
        )
        :effect (and
            (not (on ?d ?from))
            (on ?d ?to)
            (clear ?from)
            (not (clear ?to))
        )
    )
)
