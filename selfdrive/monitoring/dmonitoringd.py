#!/usr/bin/env python3
import gc

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring, DRIVER_MONITOR_SETTINGS


def dmonitoringd_thread():
  gc.disable()
  set_realtime_priority(2)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'controlsState', 'modelV2', 'carControl'], poll='driverStateV2')

  # Load configurable DM times (defaults match stock)
  settings = DRIVER_MONITOR_SETTINGS()
  try:
    distracted = float(params.get("DMDistractedTime") or 11.0)
    awareness = float(params.get("DMAwarenessTime") or 30.0)
    settings._DISTRACTED_TIME = max(5.0, min(60.0, distracted))
    settings._AWARENESS_TIME = max(10.0, min(120.0, awareness))
    # Scale the pre/prompt times proportionally
    settings._DISTRACTED_PRE_TIME_TILL_TERMINAL = settings._DISTRACTED_TIME * 0.72
    settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL = settings._DISTRACTED_TIME * 0.55
    settings._AWARENESS_PRE_TIME_TILL_TERMINAL = settings._AWARENESS_TIME * 0.5
    settings._AWARENESS_PROMPT_TIME_TILL_TERMINAL = settings._AWARENESS_TIME * 0.2
  except Exception:
    pass

  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), settings=settings, always_on=params.get_bool("AlwaysOnDM"))

  # FrogPilot variables
  driver_view_enabled = params.get_bool("IsDriverViewEnabled")

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      continue

    valid = sm.all_checks()
    if valid:
      DM.run_step(sm)
    elif driver_view_enabled:
      DM.face_detected = sm['driverStateV2'].leftDriverData.faceProb > DM.settings._FACE_THRESHOLD or sm['driverStateV2'].rightDriverData.faceProb > DM.settings._FACE_THRESHOLD

    # publish
    dat = DM.get_state_packet(valid=valid or driver_view_enabled)
    pm.send('driverMonitoringState', dat)

    # Reload configurable times and always-on toggle periodically
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
      try:
        new_distracted = float(params.get("DMDistractedTime") or settings._DISTRACTED_TIME)
        new_awareness = float(params.get("DMAwarenessTime") or settings._AWARENESS_TIME)
        if abs(new_distracted - settings._DISTRACTED_TIME) > 0.1 or abs(new_awareness - settings._AWARENESS_TIME) > 0.1:
          settings._DISTRACTED_TIME = max(5.0, min(60.0, new_distracted))
          settings._AWARENESS_TIME = max(10.0, min(120.0, new_awareness))
          settings._DISTRACTED_PRE_TIME_TILL_TERMINAL = settings._DISTRACTED_TIME * 0.72
          settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL = settings._DISTRACTED_TIME * 0.55
          settings._AWARENESS_PRE_TIME_TILL_TERMINAL = settings._AWARENESS_TIME * 0.5
          settings._AWARENESS_PROMPT_TIME_TILL_TERMINAL = settings._AWARENESS_TIME * 0.2
          DM.settings = settings
          DM._set_timers(DM.active_monitoring_mode)
      except Exception:
        pass

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and
     DM.wheelpos_learner.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_learner.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool_nonblocking("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
