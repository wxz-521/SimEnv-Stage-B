#!/usr/bin/env python3
"""ROS-independent localization continuity primitives."""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def constrain_to_corridor(position, axis_origin, axis_yaw, lateral_error):
    """Keep longitudinal progress while replacing drifted corridor lateral offset."""
    direction_x = math.cos(axis_yaw)
    direction_y = math.sin(axis_yaw)
    normal_x = -direction_y
    normal_y = direction_x
    along = (
        (position[0] - axis_origin[0]) * direction_x
        + (position[1] - axis_origin[1]) * direction_y
    )
    return (
        axis_origin[0] + along * direction_x + lateral_error * normal_x,
        axis_origin[1] + along * direction_y + lateral_error * normal_y,
    )


class PoseContinuityFilter:
    """Integrate plausible scan-matcher increments while absorbing relocalization jumps."""

    def __init__(self, minimum_jump=0.45, maximum_speed=1.5, jump_rotation=0.60):
        self.minimum_jump = float(minimum_jump)
        self.maximum_speed = float(maximum_speed)
        self.jump_rotation = float(jump_rotation)
        self.previous = None
        self.output = None
        self.absorbed_jumps = 0

    def update(self, stamp, raw_x, raw_y, raw_yaw, output_yaw):
        if self.previous is None:
            self.previous = (stamp, raw_x, raw_y, raw_yaw, output_yaw)
            self.output = (raw_x, raw_y)
            return raw_x, raw_y, False

        previous_stamp, previous_x, previous_y, previous_raw_yaw, previous_output_yaw = self.previous
        delta_time = max(0.0, stamp - previous_stamp)
        delta_x = raw_x - previous_x
        delta_y = raw_y - previous_y
        translation = math.hypot(delta_x, delta_y)
        rotation = abs(normalize_angle(raw_yaw - previous_raw_yaw))
        allowed_translation = max(
            self.minimum_jump,
            self.maximum_speed * delta_time + 0.15,
        )
        jumped = translation > allowed_translation or rotation > self.jump_rotation

        output_x, output_y = self.output
        if jumped:
            self.absorbed_jumps += 1
        else:
            frame_delta = normalize_angle(previous_output_yaw - previous_raw_yaw)
            cosine = math.cos(frame_delta)
            sine = math.sin(frame_delta)
            output_x += cosine * delta_x - sine * delta_y
            output_y += sine * delta_x + cosine * delta_y

        self.previous = (stamp, raw_x, raw_y, raw_yaw, output_yaw)
        self.output = (output_x, output_y)
        return output_x, output_y, jumped


class CommandPoseIntegrator:
    """Integrate accepted body-frame velocity commands at ROS simulation time."""

    def __init__(
        self,
        start_x,
        start_y,
        response_scale=0.95,
        command_timeout=0.35,
        minimum_motion_fraction=0.20,
    ):
        self.x = float(start_x)
        self.y = float(start_y)
        self.response_scale = float(response_scale)
        self.command_timeout = float(command_timeout)
        self.minimum_motion_fraction = float(minimum_motion_fraction)
        self.command = (0.0, 0.0)
        self.command_stamp = None
        self.previous = None

    def set_command(self, stamp, linear_x, linear_y=0.0):
        self.command = (float(linear_x), float(linear_y))
        self.command_stamp = float(stamp)

    def synchronize(self, x, y, stamp=None, yaw=None):
        """Anchor command integration to a trusted localization observation."""
        self.x = float(x)
        self.y = float(y)
        if stamp is not None and yaw is not None:
            self.previous = (float(stamp), float(yaw))
        return self.x, self.y

    def update(
        self, stamp, yaw, observed_translation=None, trust_command_motion=False
    ):
        stamp = float(stamp)
        yaw = float(yaw)
        if self.previous is None:
            self.previous = (stamp, yaw)
            return self.x, self.y
        previous_stamp, previous_yaw = self.previous
        self.previous = (stamp, yaw)
        dt = stamp - previous_stamp
        command_age = (
            stamp - self.command_stamp if self.command_stamp is not None else float("inf")
        )
        if not 0.0 < dt <= 0.5 or not -0.1 <= command_age <= self.command_timeout:
            return self.x, self.y
        forward, lateral = self.command
        commanded_translation = math.hypot(forward, lateral) * dt
        if (
            not trust_command_motion
            and observed_translation is not None
            and commanded_translation > 0.002
            and float(observed_translation)
            < self.minimum_motion_fraction * commanded_translation
        ):
            return self.x, self.y
        midpoint_yaw = previous_yaw + 0.5 * normalize_angle(yaw - previous_yaw)
        cosine = math.cos(midpoint_yaw)
        sine = math.sin(midpoint_yaw)
        self.x += self.response_scale * dt * (cosine * forward - sine * lateral)
        self.y += self.response_scale * dt * (sine * forward + cosine * lateral)
        return self.x, self.y
