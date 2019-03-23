import rospy


def move_towards(target, current, step=0.4):
    if abs(target-current) < step:
        return target, True
    else:
        if target > current:
            return current + step, False
        else:
            return current - step, False


def move_leg(leg, coxa=None, femur=None, tibia=None, step=0.4):
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


class FoldingManager(object):
    def __init__(self, body_controller):
        super(FoldingManager, self).__init__()
        self.body_controller = body_controller
        self.last_motor_position = None

    def position_femur_tibia(self):
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        while True:
            rospy.sleep(0.01)
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

    def unfold(self):
        self.position_femur_tibia()
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        left_middle_backwards = current_position.left_middle.coxa > 150.0
        right_middle_backwards = current_position.right_middle.coxa < 150.0
        while True:
            rospy.sleep(0.01)
            lf = True
            lr = True
            rf = True
            rr = True
            if left_middle_backwards:
                lf = move_leg(self.last_motor_position.left_front, 150)
            lm = move_leg(self.last_motor_position.left_middle, 150)
            if not left_middle_backwards:
                lr = move_leg(self.last_motor_position.left_rear, 150)
            if right_middle_backwards:
                rf = move_leg(self.last_motor_position.right_front, 150)
            rm = move_leg(self.last_motor_position.right_middle, 150)
            if not right_middle_backwards:
                rr = move_leg(self.last_motor_position.right_rear, 150)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lm and lr and rf and rm and rr:
                break
        while True:
            rospy.sleep(0.01)
            lf = move_leg(self.last_motor_position.left_front, 150)
            lm = move_leg(self.last_motor_position.left_middle, 150)
            lr = move_leg(self.last_motor_position.left_rear, 150)
            rf = move_leg(self.last_motor_position.right_front, 150)
            rm = move_leg(self.last_motor_position.right_middle, 150)
            rr = move_leg(self.last_motor_position.right_rear, 150)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lm and lr and rf and rm and rr:
                break

    def fold(self):
        self.position_femur_tibia()
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, 150)
            rm = move_leg(self.last_motor_position.right_middle, 150)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        while True:
            rospy.sleep(0.01)
            lf = move_leg(self.last_motor_position.left_front, 240)
            lr = move_leg(self.last_motor_position.left_rear, 60)
            rf = move_leg(self.last_motor_position.right_front, 60)
            rr = move_leg(self.last_motor_position.right_rear, 240)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr and rf and rr:
                break
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, 240)
            rm = move_leg(self.last_motor_position.right_middle, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break

    def unfold_on_ground(self):
        self.position_femur_tibia()
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        # lift middle legs
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, tibia=190)
            rm = move_leg(self.last_motor_position.right_middle, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        # fold out middle legs
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, coxa=150)
            rm = move_leg(self.last_motor_position.right_middle, coxa=150)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
        # lower right leg
        while True:
            rospy.sleep(0.01)
            rm = move_leg(self.last_motor_position.right_middle, femur=170, tibia=100)
            self.body_controller.set_motors(self.last_motor_position)
            if rm:
                break
        # unfold right legs
        while True:
            rospy.sleep(0.01)
            rf = move_leg(self.last_motor_position.right_front, coxa=150)
            rr = move_leg(self.last_motor_position.right_rear, coxa=150)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # lift right legs
        while True:
            rospy.sleep(0.01)
            rf = move_leg(self.last_motor_position.right_front, tibia=90)
            rr = move_leg(self.last_motor_position.right_rear, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # switch lifted side
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, femur=130, tibia=200)
            rm = move_leg(self.last_motor_position.right_middle, femur=240, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if rm and lm:
                break
        # unfold left legs
        while True:
            rospy.sleep(0.01)
            lf = move_leg(self.last_motor_position.left_front, coxa=150)
            lr = move_leg(self.last_motor_position.left_rear, coxa=150)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        # lift left legs
        while True:
            rospy.sleep(0.01)
            lf = move_leg(self.last_motor_position.left_front, tibia=210)
            lr = move_leg(self.last_motor_position.left_rear, tibia=210)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        # lift middle left
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, femur=60, tibia=210)
            self.body_controller.set_motors(self.last_motor_position)
            if lm:
                break

    def fold_on_ground(self):
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        while True:
            rospy.sleep(0.01)
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
            rospy.sleep(0.01)
            rm = move_leg(self.last_motor_position.right_middle, femur=170, tibia=100)
            self.body_controller.set_motors(self.last_motor_position)
            if rm:
                break
        # compress right legs
        while True:
            rospy.sleep(0.01)
            rf = move_leg(self.last_motor_position.right_front, None, 240, 60)
            rr = move_leg(self.last_motor_position.right_rear, None, 240, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # fold right legs
        while True:
            rospy.sleep(0.01)
            rf = move_leg(self.last_motor_position.right_front, 60)
            rr = move_leg(self.last_motor_position.right_rear, 240)
            self.body_controller.set_motors(self.last_motor_position)
            if rf and rr:
                break
        # switch lifted side
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, femur=130, tibia=200)
            rm = move_leg(self.last_motor_position.right_middle, femur=240, tibia=90)
            self.body_controller.set_motors(self.last_motor_position)
            if rm and lm:
                break
        # compress left legs
        while True:
            rospy.sleep(0.01)
            lf = move_leg(self.last_motor_position.left_front, None, 60, 240)
            lr = move_leg(self.last_motor_position.left_rear, None, 60, 240)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        # fold left legs
        while True:
            rospy.sleep(0.01)
            lf = move_leg(self.last_motor_position.left_front, 240)
            lr = move_leg(self.last_motor_position.left_rear, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lr:
                break
        while True:
            rospy.sleep(0.01)
            lm = move_leg(self.last_motor_position.left_middle, 240)
            rm = move_leg(self.last_motor_position.right_middle, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lm and rm:
                break
