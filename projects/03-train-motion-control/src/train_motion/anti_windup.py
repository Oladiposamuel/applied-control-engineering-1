"""Small, explicit conditional-integration helper."""


def conditional_integral_rate(error: float, raw_force: float, gain: float,
                              lower: float, upper: float) -> tuple[float, bool]:
    if gain == 0:
        return 0.0, False
    blocked = ((raw_force >= upper and error > 0) or
               (raw_force <= lower and error < 0))
    return (0.0, True) if blocked else (error, False)
