from ..model import Model, Data


def contraction(model: Model, data: Data, theta: float | None = None) -> float:
    """Calculate contraction as function of motor angle"""
    theta = data.theta if theta is None else theta

    data.theta = theta
    data.x = model.L - (model.L**2 - (theta * model.r) ** 2) ** 0.5
    return data.x


def motor_angle(model: Model, data: Data, x: float | None = None) -> float:
    """Calculate motor angle as function of contraction"""
    x = data.x if x is None else x

    data.x = x
    data.theta = (model.L**2 - (model.L - x) ** 2) ** 0.5 / model.r
    return data.theta
