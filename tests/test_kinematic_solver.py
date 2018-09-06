import pytest
import math
from catkin_ws.src.kinematic_robot import KinematicSolver


class TestKinematicSolver:
    """
    Test the class KinematicSolver
    """
    solver = 0

    def test_init_solver_1(self):
        # If there is no robot l1 = l2 = l3 = 0
        with pytest.raises(ValueError):
            self.solver = KinematicSolver(0, 0, 0, [0, math.pi], [0, math.pi], [0, math.pi])

    def test_init_solver_2(self):
        # If there is no negative length
        with pytest.raises(ValueError):
            self.solver = KinematicSolver(-1, 0, 0, [0, math.pi], [0, math.pi], [0, math.pi])

    def test_fkine(self):
        # Test one case that is good
        self.solver = KinematicSolver(1, 1, 1, [-0.1, 3.14], [-0.1, 3.14], [-0.1, 3.14])
        # Init pose
        init_pose_angle = [0, math.pi / 4, 0]
        # fkine
        result = self.solver.fkine(init_pose_angle)
        assert result == (1.7071067811865475, 0.0, 1.7071067811865475)

    def test_ikine(self):
        # Test one case that is good
        self.solver = KinematicSolver(1, 1, 1, [-0.1, 3.14], [-0.1, 3.14], [-0.1, 3.14])
        # Init pose
        init_pose_angle = (0., math.pi / 4, 0.)
        end_pose_angle = self.solver.ikine([1.7071067811865475, 0.0, 1.7071067811865475])
        assert end_pose_angle == init_pose_angle



