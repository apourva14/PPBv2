"""Exact integer-nanosecond handling for ROS time stamps (no ROS imports)."""

NS_PER_S = 1_000_000_000


def stamp_to_ns(sec, nanosec):
    """builtin_interfaces/Time fields -> integer nanoseconds."""
    if not 0 <= nanosec < NS_PER_S:
        raise ValueError(f'nanosec out of range: {nanosec}')
    return int(sec) * NS_PER_S + int(nanosec)


def ns_to_stamp(ns):
    """Integer nanoseconds -> (sec, nanosec) for builtin_interfaces/Time."""
    ns = int(ns)
    if ns < 0:
        raise ValueError(f'negative time: {ns}')
    return ns // NS_PER_S, ns % NS_PER_S
