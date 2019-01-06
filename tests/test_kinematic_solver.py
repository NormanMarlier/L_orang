import pytest
import math
from catkin_ws.src.kinematic_robot import RRRSolver


class TestRRRSolver:
    """
    Test the class KinematicSolver
    """
    solver = 0

    def test_init_solver_1(self):
        # If there is no robot l1 = l2 = l3 = 0
        with pytest.raises(ValueError):
            self.solver = RRRSolver(0, 0, 0, [0, 180], [0, 180], [0, 180])

    def test_init_solver_2(self):
        # If there is no negative length
        with pytest.raises(ValueError):
            self.solver = RRRSolver(0, -1, 0, [0, 180], [0, 180], [0, 180])

    def test_init_solver_3(self):
        # If there is no robot l2 = l3 = 0
        with pytest.raises(ValueError):
            self.solver = RRRSolver(1, 0, 0, [0, 180], [0, 180], [0, 180])

    def test_check_angle_1(self):
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])

        result = self.solver.check_angle([-20, 0, 0])
        assert result is False

    def test_check_angle_2(self):
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])

        result = self.solver.check_angle([-0, 400, 0])
        assert result is False

    def test_check_angle_3(self):
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])

        result = self.solver.check_angle([-20, -20, -20])
        assert result is False

    def test_check_angle_4(self):
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])

        result = self.solver.check_angle([0, 0, 0])
        assert result is True

    def test_calibration_1(self):
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])

        with pytest.raises(ValueError):
            self.solver.calibration([0, 0, 0], [1, 1, 0])

    def test_calibration_2(self):
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])

        with pytest.raises(ValueError):
            self.solver.calibration([0, 0, 0], [0, 0, -1])

    def test_fkine(self):
        # Test one case that is good
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])
        # Init pose
        init_pose_angle = [0, math.pi / 4, 0]
        # fkine
        result = self.solver.fkine(init_pose_angle)
        assert result == (1.7071067811865475, 0.0, 1.7071067811865475)

    def test_ikine(self):
        # Test one case that is good
        self.solver = RRRSolver(1, 1, 1, [-10, 360], [-10, 360], [-10, 360])
        # Init pose
        init_pose_angle = (0., math.pi / 4, 0.)
        end_pose_angle = self.solver.ikine([1.7071067811865475, 0.0, 1.7071067811865475])
        assert end_pose_angle == init_pose_angle








