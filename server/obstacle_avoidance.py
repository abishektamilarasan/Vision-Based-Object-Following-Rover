import config
import time

class ObstacleAvoidance:
    def __init__(self):
        self.left_blocked = False
        self.right_blocked = False
        self.both_blocked_start = 0

    def update_sensor_state(self, left, right):
        self.left_blocked = left
        self.right_blocked = right
        
        if self.left_blocked and self.right_blocked:
            if self.both_blocked_start == 0:
                self.both_blocked_start = time.time()
        else:
            self.both_blocked_start = 0

    def apply(self, left_speed, right_speed):
        """
        Modifies intended motor speeds based on IR sensor flags.
        NOTE: IR obstacle avoidance is currently disabled/bypassed.
        Returns: (modified_left_speed, modified_right_speed, alert_message)
        """
        # IR Sensors disabled - pass motor commands directly through
        left_speed = max(min(int(left_speed), config.MAX_SPEED), -config.MAX_SPEED)
        right_speed = max(min(int(right_speed), config.MAX_SPEED), -config.MAX_SPEED)
        return left_speed, right_speed, None

        # --- IR Avoidance Logic (Disabled/Commented Out) ---
        # alert = None
        # if self.left_blocked and self.right_blocked:
        #     left_speed, right_speed = -140, -140
        #     alert = "COLLISION ALERT: Both IRs triggered! Backing up."
        # elif self.left_blocked and not self.right_blocked:
        #     left_speed = config.MAX_SPEED
        #     right_speed = -config.MAX_SPEED
        #     alert = "Obstacle on left! Pivoting right."
        # elif self.right_blocked and not self.left_blocked:
        #     left_speed = -config.MAX_SPEED
        #     right_speed = config.MAX_SPEED
        #     alert = "Obstacle on right! Pivoting left."
        # left_speed = max(min(int(left_speed), config.MAX_SPEED), -config.MAX_SPEED)
        # right_speed = max(min(int(right_speed), config.MAX_SPEED), -config.MAX_SPEED)
        # return left_speed, right_speed, alert
