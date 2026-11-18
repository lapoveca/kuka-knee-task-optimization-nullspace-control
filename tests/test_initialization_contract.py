import numpy as np

from integration.initialization import joint_sample


class FakeStamp:
    def __init__(self, sec):
        self.sec = sec
        self.nanosec = 0


class FakeHeader:
    def __init__(self, sec):
        self.stamp = FakeStamp(sec)


class FakeMessage:
    def __init__(self, names, positions, velocities, stamp):
        self.name = names
        self.position = positions
        self.velocity = velocities
        self.header = FakeHeader(stamp)


def test_joint_sample_reorders_by_name():
    message = FakeMessage(
        names=[
            "lbr_A3", "lbr_A1", "lbr_A2", "lbr_A4",
            "lbr_A5", "lbr_A6", "lbr_A7",
        ],
        positions=[3, 1, 2, 4, 5, 6, 7],
        velocities=[0.3, 0.1, 0.2, 0.4, 0.5, 0.6, 0.7],
        stamp=12,
    )

    stamp, q, qd = joint_sample(message)

    assert stamp == 12.0
    assert np.allclose(q, [1, 2, 3, 4, 5, 6, 7])
    assert np.allclose(qd, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])


def test_joint_sample_rejects_missing_joint():
    message = FakeMessage(
        names=[
            "lbr_A1", "lbr_A2", "lbr_A3",
            "lbr_A4", "lbr_A5", "lbr_A6",
        ],
        positions=[1, 2, 3, 4, 5, 6],
        velocities=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        stamp=0,
    )

    try:
        joint_sample(message)
    except ValueError as exc:
        assert "missing" in str(exc)
    else:
        raise AssertionError("Expected ValueError")