#!/usr/bin/env python3
"""
Simple openpilot-style simulator for Roundabout Yield logic.
Tests the experimental leadLeft/leadRight slowdown behavior.
"""

import numpy as np

class FakeRadarLead:
  def __init__(self, status=False, dRel=100.0, vLead=0.0, yRel=0.0):
    self.status = status
    self.dRel = dRel
    self.vLead = vLead
    self.yRel = yRel

class FakeRadarState:
  def __init__(self):
    self.leadLeft = FakeRadarLead()
    self.leadRight = FakeRadarLead()

class RoundaboutSimulator:
  def __init__(self):
    self.roundabout_yield = True
    self.v_cruise = 15.0  # m/s (~54 km/h)
    self.forcingStop = False

  def update(self, radar: FakeRadarState):
    v_cruise = self.v_cruise

    # Roundabout logic
    if self.roundabout_yield and radar.leadLeft.status:
      left = radar.leadLeft
      if left.dRel < 18.0 and left.vLead > 1.5 and abs(left.yRel) > 1.0:
        v_cruise = min(v_cruise, 5.0)
        print(f"  → Roundabout yield: left lead at {left.dRel:.1f}m, reducing to 5.0 m/s")

    # Stop logic
    if self.roundabout_yield and self.forcingStop:
      conflict = False
      if radar.leadLeft.status and radar.leadLeft.dRel < 25.0 and radar.leadLeft.vLead > 1.0:
        conflict = True
      if radar.leadRight.status and radar.leadRight.dRel < 25.0 and radar.leadRight.vLead > 1.0:
        conflict = True
      if conflict:
        v_cruise = min(v_cruise, 0.5)
        print(f"  → Stop yield: conflict detected, reducing to 0.5 m/s")

    return v_cruise

def run_scenarios():
  sim = RoundaboutSimulator()
  print("=== Roundabout Yield Simulator ===\n")

  # Scenario 1: Clear roundabout
  print("1. Clear roundabout (no left lead)")
  radar = FakeRadarState()
  v = sim.update(radar)
  print(f"   Result: {v:.1f} m/s\n")

  # Scenario 2: Left vehicle in roundabout
  print("2. Left vehicle approaching in roundabout")
  radar = FakeRadarState()
  radar.leadLeft = FakeRadarLead(status=True, dRel=12.0, vLead=8.0, yRel=-2.5)
  v = sim.update(radar)
  print(f"   Result: {v:.1f} m/s\n")

  # Scenario 3: Stop with left conflict
  print("3. Stop sign with left vehicle")
  sim.forcingStop = True
  radar = FakeRadarState()
  radar.leadLeft = FakeRadarLead(status=True, dRel=15.0, vLead=6.0, yRel=-1.8)
  v = sim.update(radar)
  print(f"   Result: {v:.1f} m/s\n")

  # Scenario 4: Stop with both sides
  print("4. Stop sign with left and right vehicles")
  radar = FakeRadarState()
  radar.leadLeft = FakeRadarLead(status=True, dRel=10.0, vLead=5.0, yRel=-2.0)
  radar.leadRight = FakeRadarLead(status=True, dRel=18.0, vLead=7.0, yRel=2.2)
  v = sim.update(radar)
  print(f"   Result: {v:.1f} m/s\n")

  # Scenario 5: Far left vehicle (no yield)
  print("5. Far left vehicle (should not yield)")
  sim.forcingStop = False
  radar = FakeRadarState()
  radar.leadLeft = FakeRadarLead(status=True, dRel=30.0, vLead=10.0, yRel=-3.0)
  v = sim.update(radar)
  print(f"   Result: {v:.1f} m/s\n")

if __name__ == "__main__":
  run_scenarios()
