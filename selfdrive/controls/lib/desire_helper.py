from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# Experimental auto overtake thresholds
AUTO_OVERTAKE_VREL_THRESHOLD = -1.5  # m/s (~5.4 km/h slower)
AUTO_OVERTAKE_MIN_LEAD_DIST = 15.0
AUTO_OVERTAKE_MAX_LEAD_DIST = 80.0

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # FrogPilot variables
    self.lane_change_completed = False

    self.lane_change_wait_timer = 0

    # Auto overtake
    self.auto_overtake_active = False

  def update(self, carstate, lateral_active, lane_change_prob, frogpilotPlan, frogpilot_toggles, radar_state=None):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < frogpilot_toggles.minimum_lane_change_speed

    # Detect slow lead for auto overtake
    slow_lead = False
    if radar_state is not None and radar_state.leadOne.status:
      lead = radar_state.leadOne
      if (AUTO_OVERTAKE_MIN_LEAD_DIST < lead.dRel < AUTO_OVERTAKE_MAX_LEAD_DIST and
          lead.vRel < AUTO_OVERTAKE_VREL_THRESHOLD):
        slow_lead = True

    # Left lane available and safe for auto overtake
    left_lane_safe = (frogpilotPlan.laneWidthLeft >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.lane_detection)
    left_blindspot_clear = not carstate.leftBlindspot

    auto_overtake_enabled = getattr(frogpilot_toggles, 'auto_overtake', False)
    auto_overtake_ready = (auto_overtake_enabled and not one_blinker and not below_lane_change_speed and slow_lead and
                           left_lane_safe and left_blindspot_clear and lateral_active)

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or not frogpilot_toggles.lane_changes:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.auto_overtake_active = False
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off:
        if one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
          self.lane_change_state = LaneChangeState.preLaneChange
          self.lane_change_ll_prob = 1.0
          self.lane_change_wait_timer = 0.0
          self.auto_overtake_active = False
        elif auto_overtake_ready:
          self.lane_change_state = LaneChangeState.preLaneChange
          self.lane_change_ll_prob = 1.0
          self.lane_change_wait_timer = 0.0
          self.auto_overtake_active = True
          self.lane_change_direction = LaneChangeDirection.left

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL

        if not self.auto_overtake_active:
          # Normal blinker-based
          self.lane_change_direction = LaneChangeDirection.left if carstate.leftBlinker else LaneChangeDirection.right

          torque_applied = carstate.steeringPressed and \
                           ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                            (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

          if torque_applied:
            self.lane_change_wait_timer = frogpilot_toggles.lane_change_delay
          else:
            desired_lane_width = frogpilotPlan.laneWidthLeft if carstate.leftBlinker else frogpilotPlan.laneWidthRight
            lane_available = desired_lane_width >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.lane_detection
            torque_applied = lane_available and self.lane_change_wait_timer >= frogpilot_toggles.lane_change_delay and frogpilot_toggles.nudgeless

          blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                                (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

          if not one_blinker or below_lane_change_speed or self.lane_change_completed:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
          elif torque_applied and not blindspot_detected:
            self.lane_change_state = LaneChangeState.laneChangeStarting
            self.lane_change_completed = frogpilot_toggles.one_lane_change
            self.lane_change_wait_timer = 0.0
        else:
          # Auto overtake path (left only, no blinker required)
          if not auto_overtake_ready or below_lane_change_speed:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            self.auto_overtake_active = False
          elif self.lane_change_wait_timer >= 1.0 and not carstate.leftBlindspot:  # short delay
            self.lane_change_state = LaneChangeState.laneChangeStarting
            self.lane_change_wait_timer = 0.0

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          self.auto_overtake_active = False
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.lane_change_completed &= one_blinker
    self.prev_one_blinker = one_blinker

    if one_blinker and below_lane_change_speed and not carstate.standstill and frogpilot_toggles.use_turn_desires:
      self.turn_direction = TurnDirection.turnLeft if carstate.leftBlinker else TurnDirection.turnRight
      self.desire = TURN_DESIRES[self.turn_direction]
    else:
      self.turn_direction = TurnDirection.none
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
