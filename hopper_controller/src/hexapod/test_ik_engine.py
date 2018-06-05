from unittest import TestCase
from hexapod_ik_driver import Vector3, LegPositions


LEG_HEIGHT = -9
LEG_DISTANCE_LONGITUDAL = 15
MIDDLE_LEG_LONGITUDAL_OFFSET = 7
LEG_DISTANCE_LATERAL = 18

default_leg_positions = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET, LEG_HEIGHT),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
)


class TestLegPositions(TestCase):
    def test_get_center_point(self):
        # Arrange
        a = default_leg_positions.clone().turn(30)
        b = default_leg_positions.clone()
        # Act
        a_center = a.get_center_point()
        b_center = b.get_center_point()
        # Assert
        self.assertEquals(a_center, b_center)

    def test_leg_positions_equals(self):
        # Arrange
        a = default_leg_positions.clone()
        b = default_leg_positions.clone()
        # Assert
        self.assertTrue(a == b)

    def test_leg_positions_equals_fails(self):
        # Arrange
        a = default_leg_positions.clone()
        b = default_leg_positions.clone()
        # Act
        a.left_front.x = 25
        # Assert
        self.assertFalse(a == b)

    def test_get_moved_towards_handles_edges(self):
        # Arrange
        a = default_leg_positions.clone().transform(Vector3(x=5))
        b = default_leg_positions.clone()
        # Act
        should_be_b = b.get_moved_towards_by_portion(a, 0)
        should_be_a = b.get_moved_towards_by_portion(a, 1)
        should_be_neither = b.get_moved_towards_by_portion(a, 0.5)
        should_be_a_2 = b.get_moved_towards_by_portion(a, 7)
        # Assert
        self.assertIs(should_be_b, b)
        self.assertIs(should_be_a, a)
        self.assertIs(should_be_a_2, a)
        self.assertIsNot(should_be_neither, a)
        self.assertIsNot(should_be_neither, b)
