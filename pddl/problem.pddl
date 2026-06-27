(define (problem hanoi-2)
    (:domain hanoi)

    (:objects
        disk1 disk2 - disk
        peg1 peg2 peg3 - peg
    )

    (:init
        (on disk2 disk1)
        (on disk1 peg1)

        (clear disk2)
        (clear peg2)
        (clear peg3)

        (smaller disk2 disk1)
    )

    (:goal 
        (and
            (on disk2 disk1)
            (on disk1 peg3)
        )
    )
)