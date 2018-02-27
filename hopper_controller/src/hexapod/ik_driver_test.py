from .hexapod_ik_driver import calculate_ik_for_leg, calculate_fk_for_leg, Vector3, LegPositions, LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR, LEFT_MIDDLE, RIGHT_MIDDLE

LEG_HEIGHT = -9
LEG_DISTANCE_LONGITUDAL = 15
LEG_DISTANCE_LATERAL = 15

RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + 5, LEG_HEIGHT),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - 5, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
)

lf = calculate_ik_for_leg(RELAXED_POSITION.left_front, LEFT_FRONT)
print "Left front"
print RELAXED_POSITION.left_front
print calculate_fk_for_leg(lf, LEFT_FRONT)

rf = calculate_ik_for_leg(RELAXED_POSITION.right_front, RIGHT_FRONT)
print "Right front"
print RELAXED_POSITION.right_front
print calculate_fk_for_leg(rf, RIGHT_FRONT)

lm = calculate_ik_for_leg(RELAXED_POSITION.left_middle, LEFT_MIDDLE)
print "Left middle"
print RELAXED_POSITION.left_middle
print calculate_fk_for_leg(lm, LEFT_MIDDLE)

rm = calculate_ik_for_leg(RELAXED_POSITION.right_middle, RIGHT_MIDDLE)
print "Right middle"
print RELAXED_POSITION.right_middle
print calculate_fk_for_leg(rm, RIGHT_MIDDLE)

lr = calculate_ik_for_leg(RELAXED_POSITION.left_rear, LEFT_REAR)
print "Left rear"
print RELAXED_POSITION.left_rear
print calculate_fk_for_leg(lr, LEFT_REAR)

rr = calculate_ik_for_leg(RELAXED_POSITION.right_rear, RIGHT_REAR)
print "Right rear"
print RELAXED_POSITION.right_rear
print calculate_fk_for_leg(rr, RIGHT_REAR)
