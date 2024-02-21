from solver import through_front, through_side, danger_zone


def testThroughFront():
    p1 = (0.7, 4.5, 2.6)
    p2 = (-0.2, 3.3, 2.3)
    assert through_front(p1, p2) == 1
    assert through_front(p2, p1) == 0
    assert through_front((0.7, 4.5, 3.9), (-0.2, 3.3, 2.3)) == 1
    assert through_front((0.7, 4.5, 1.7), (-0.2, 3.3, 2.3)) == 0


def testThroughSide():
    p1 = (0.7, 2.9, 3.1)
    p2 = (-0.2, 4.5, 2.4)
    assert through_side(p1, p2) == 1
    assert through_side(p2, p1) == 0
    assert through_side((1.5, 2.9, 3.1), p2) == 0
    assert through_side((0.7, 5, 3.1), (-1.3, 3.5, 2.4)) == 1
    assert through_side((0.7, 5, 1.6), (-1.3, 3.5, 2.4)) == 0


testThroughSide()
testThroughSide()
