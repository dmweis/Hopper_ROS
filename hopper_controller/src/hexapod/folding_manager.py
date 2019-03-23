import rospy

MOVE_CYCLE_PERIOD = 0.01


def move_towards(target, current, step=1):
    if abs(target-current) < step:
        return target, True
    else:
        if target > current:
            return current + step, False
        else:
            return current - step, False


def move_leg(leg, coxa=None, femur=None, tibia=None, step=1):
    coxa_done = True
    femur_done = True
    tibia_done = True
    if coxa:
        leg.coxa, coxa_done = move_towards(coxa, leg.coxa, step)
    if femur:
        leg.femur, femur_done = move_towards(femur, leg.femur, step)
    if tibia:
        leg.tibia, tibia_done = move_towards(tibia, leg.tibia, step)
    return coxa_done and femur_done and tibia_done


def is_leg_close(leg, coxa=None, femur=None, tibia=None, tolerance=20):
    coxa_close = True
    femur_close = True
    tibia_close = True
    if coxa:
        coxa_close = leg.coxa + tolerance > coxa > leg.coxa - tolerance
    if femur:
        femur_close = leg.femur + tolerance > femur > leg.femur - tolerance
    if tibia:
        tibia_close = leg.tibia + tolerance > tibia > leg.tibia - tolerance
    return coxa_close and femur_close and tibia_close


class FoldingManager(object):
    def __init__(self, body_controller):
        super(FoldingManager, self).__init__()
        self.body_controller = body_controller
        self.last_motor_position = None

    def position_femur_tibia(self):
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = move_leg(self.last_motor_position.left_front, None, 60, 240)
            lm = move_leg(self.last_motor_position.left_middle, None, 60, 240)
            lr = move_leg(self.last_motor_position.left_rear, None, 60, 240)
            rf = move_leg(self.last_motor_position.right_front, None, 240, 60)
            rm = move_leg(self.last_motor_position.right_middle, None, 240, 60)
            rr = move_leg(self.last_motor_position.right_rear, None, 240, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lm and lr and rf and rm and rr:
                break
        rospy.sleep(0.05)

    def check_if_folded(self):
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        lf = is_leg_close(self.last_motor_position.left_front, 240)
        lm = is_leg_close(self.last_motor_position.left_middle, 240) or is_leg_close(self.last_motor_position.left_middle, 60)
        lr = is_leg_close(self.last_motor_position.left_rear, 60)

        rf = is_leg_close(self.last_motor_position.right_front, 60)
        rm = is_leg_close(self.last_motor_position.right_middle, 60) or is_leg_close(self.last_motor_position.right_middle, 240)
        rr = is_leg_close(self.last_motor_position.right_rear, 240)
        return lf  and lm and lr and rf and rm and rr

    def unfold(self):
        self.position_femur_tibia()
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = False
            lr = False
            rf = False
            rr = False
            if self.last_motor_position.left_middle.coxa > 120:
                lf = move_leg(self.last_motor_position.left_front, 150)
            lm = move_leg(self.last_motor_position.left_middle, 150)
            if self.last_motor_position.left_middle.coxa < 180:
                lr = move_leg(self.last_motor_position.left_rear, 150)
            if self.last_motor_position.right_middle.coxa < 180:
                rf = move_leg(self.last_motor_position.right_front, 150)
            rm = move_leg(self.last_motor_position.right_middle, 150)
            if self.last_motor_position.right_middle.coxa > 120:
                rr = move_leg(self.last_motor_position.right_rear, 150)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lm and lr and rf and rm and rr:
                break
        rospy.sleep(0.5)
        self.body_controller.set_torque(False)

    def fold(self):
        self.position_femur_tibia()
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        if not self.check_if_folded():
            while True:
                rospy.sleep(MOVE_CYCLE_PERIOD)
                lm = move_leg(self.last_motor_position.left_middle, 150)
                rm = move_leg(self.last_motor_position.right_middle, 150)
                self.body_controller.set_motors(self.last_motor_position)
                if lm and rm:
                    break
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = move_leg(self.last_motor_position.left_front, 240)
            lr = move_leg(self.last_motor_position.left_rear, 60)
            rf = move_leg(self.last_motor_position.right_front, 60)
            rr = move_leg(self.last_motor_position.right_rear, 240)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr and rf and rr:
                break
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, 240)
            rm = move_leg(self.last_motor_position.right_middle, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        rospy.sleep(0.5)
        self.body_controller.set_torque(False)

    def unfold_on_ground(self):
        self.position_femur_tibia()
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        # lift middle legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, tibia=200)
            rm = move_leg(self.last_motor_position.right_middle, tibia=100)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        # fold out middle legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, coxa=150)
            rm = move_leg(self.last_motor_position.right_middle, coxa=150)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        # lower right leg
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            rm = move_leg(self.last_motor_position.right_middle, femur=170, tibia=100)
            self.body_controller.set_motors(self.last_motor_position)
            if rm:
                break
        # unfold right legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            rf = move_leg(self.last_motor_position.right_front, coxa=150)
            rr = move_leg(self.last_motor_position.right_rear, coxa=150)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # lift right legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            rf = move_leg(self.last_motor_position.right_front, tibia=90)
            rr = move_leg(self.last_motor_position.right_rear, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # switch lifted side
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, femur=130, tibia=200)
            rm = move_leg(self.last_motor_position.right_middle, femur=240, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if rm and lm:
                break
        # unfold left legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = move_leg(self.last_motor_position.left_front, coxa=150)
            lr = move_leg(self.last_motor_position.left_rear, coxa=150)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        # lift left legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = move_leg(self.last_motor_position.left_front, tibia=210)
            lr = move_leg(self.last_motor_position.left_rear, tibia=210)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        # lift middle left
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, femur=60, tibia=210)
            self.body_controller.set_motors(self.last_motor_position)
            if lm:
                break
        rospy.sleep(0.5)
        self.body_controller.set_torque(False)

    def fold_on_ground(self):
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = move_leg(self.last_motor_position.left_front, 150, femur=60, tibia=210)
            lm = move_leg(self.last_motor_position.left_middle, 150, femur=60, tibia=210)
            lr = move_leg(self.last_motor_position.left_rear, 150, femur=60, tibia=210)
            rf = move_leg(self.last_motor_position.right_front, 150, femur=240, tibia=90)
            rm = move_leg(self.last_motor_position.right_middle, 150, femur=240, tibia=90)
            rr = move_leg(self.last_motor_position.right_rear, 150, femur=240, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lm and lr and rf and rm and rr:
                break
        # lower right leg
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            rm = move_leg(self.last_motor_position.right_middle, femur=170, tibia=100)
            self.body_controller.set_motors(self.last_motor_position)
            if rm:
                break
        # compress right legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            rf = move_leg(self.last_motor_position.right_front, None, 240, 60)
            rr = move_leg(self.last_motor_position.right_rear, None, 240, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # fold right legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            rf = move_leg(self.last_motor_position.right_front, 60)
            rr = move_leg(self.last_motor_position.right_rear, 240)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # switch lifted side
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, femur=130, tibia=200)
            rm = move_leg(self.last_motor_position.right_middle, femur=240, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if rm and lm:
                break
        # compress left legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = move_leg(self.last_motor_position.left_front, None, 60, 240)
            lr = move_leg(self.last_motor_position.left_rear, None, 60, 240)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        # fold left legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lf = move_leg(self.last_motor_position.left_front, 240)
            lr = move_leg(self.last_motor_position.left_rear, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        # lift left middle leg
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, femur=60, tibia=210)
            self.body_controller.set_motors(self.last_motor_position)
            if lm:
                break
        # fold middle legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, 230)
            rm = move_leg(self.last_motor_position.right_middle, 70)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        # compress middle legs
        while True:
            rospy.sleep(MOVE_CYCLE_PERIOD)
            lm = move_leg(self.last_motor_position.left_middle, None, 60, 240)
            rm = move_leg(self.last_motor_position.right_middle, None, 240, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        rospy.sleep(0.5)
        self.body_controller.set_torque(False)
