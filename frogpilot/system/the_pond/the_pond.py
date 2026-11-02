#!/usr/bin/env python3
import threading
import time

from flask import Flask, abort, jsonify, render_template, request
from werkzeug.exceptions import HTTPException

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import PC

from openpilot.frogpilot.common.frogpilot_api import FrogPilotAPI
from openpilot.frogpilot.system.the_pond import device, discovery, helpers, logs, media, navigation, tailscale, themes

PAGE_PATHS = {
  "/", "/dashcam_routes", "/download_speed_limits", "/lock_or_unlock_doors", "/manage_error_logs", "/manage_navigation_keys",
  "/manage_tailscale", "/manage_tmux", "/manage_toggles", "/screen_recordings", "/set_navigation_destination", "/theme_maker", "/tsk_manager",
}

PORT = 8082
PORTS = {80, PORT}
if PC:
  PORT = 8083
  PORTS = {PORT}

CSP = "; ".join([
  "default-src 'self'",
  "script-src 'self'",
  "style-src 'self' 'unsafe-inline'",
  "img-src 'self' data: blob: https://api.mapbox.com https://*.tiles.mapbox.com",
  "media-src 'self' blob:",
  "font-src 'self' data:",
  "connect-src 'self' https://api.mapbox.com https://events.mapbox.com https://*.tiles.mapbox.com",
  "worker-src blob:",
  "child-src blob:",
  "object-src 'none'",
  "base-uri 'self'",
  "form-action 'self'",
  "frame-ancestors 'none'",
])


def locked_stream(source):
  checked = 0
  try:
    for chunk in source:
      now = time.monotonic()
      if now - checked >= 1:
        if device.is_driving():
          return
        checked = now

      yield chunk
  finally:
    close = getattr(source, "close", None)
    if close is not None:
      close()


def create_app():
  app = Flask(__name__, static_folder="assets", static_url_path="/assets")
  app.extensions["frogpilot_api"] = FrogPilotAPI(Params())
  app.config["MAX_CONTENT_LENGTH"] = 32 * 1024 * 1024
  app.config["POND_PORTS"] = PORTS

  for feature in (device, logs, media, navigation, tailscale, themes):
    app.register_blueprint(feature.blueprint)

  def shell_request():
    return request.method in ("GET", "HEAD") and (request.endpoint == "static" or request.path in PAGE_PATHS or request.path == "/api/onroad")

  @app.before_request
  def check_request():
    if not helpers.host_allowed(request.scheme, request.host):
      abort(421, "Unrecognized Host")

    if request.method not in ("GET", "HEAD", "OPTIONS"):
      origin = request.headers.get("Origin")
      if origin is not None:
        allowed = helpers.origin_allowed(origin, request.scheme, request.host, app.config["POND_PORTS"])
      else:
        allowed = helpers.referer_allowed(request.headers.get("Referer"), request.scheme, request.host, app.config["POND_PORTS"])
      if not allowed:
        abort(403, "Cross-origin request blocked")

    if not shell_request() and device.is_driving():
      abort(423, "Unavailable while driving. Shift into Park or go offroad to use The Pond.")

  @app.after_request
  def complete_response(response):
    response.headers["Content-Security-Policy"] = CSP
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["Referrer-Policy"] = "same-origin"
    response.headers["Cross-Origin-Resource-Policy"] = "same-origin"

    if request.path.startswith("/api/"):
      response.headers["Cache-Control"] = "no-store"

    if request.method != "HEAD" and response.is_streamed and 200 <= response.status_code < 300 and not shell_request():
      response.response = locked_stream(response.response)

    return response

  @app.errorhandler(HTTPException)
  def request_error(error):
    response = error.get_response()
    response.data = app.json.dumps({"error": error.description})
    response.content_type = "application/json"
    return response

  @app.errorhandler(Exception)
  def internal_error(error):
    cloudlog.exception("The Pond could not complete a request")
    return jsonify(error="Could not complete this request. Please try again."), 500

  @app.get("/api/live/status")
  def live_status():
    from openpilot.common.realtime import DT_CTRL
    import cereal.messaging as messaging
    sm = messaging.SubMaster(["carState", "controlsState", "frogpilotPlan", "frogpilotRadarState", "modelV2"])
    sm.update(0)
    cs = sm["carState"] if sm.alive["carState"] else None
    plan = sm["frogpilotPlan"] if sm.alive["frogpilotPlan"] else None
    radar = sm["frogpilotRadarState"] if sm.alive["frogpilotRadarState"] else None
    data = {
      "vEgo": round(cs.vEgo * 3.6, 1) if cs else 0,
      "vCruise": round(plan.vCruise * 3.6, 1) if plan else 0,
      "standstill": bool(cs.standstill) if cs else False,
      "enabled": bool(sm["controlsState"].enabled) if sm.alive["controlsState"] else False,
      "forcingStop": bool(plan.forcingStop) if plan else False,
      "cscControlling": bool(plan.cscControllingSpeed) if plan else False,
      "leadLeft": {
        "status": bool(radar.leadLeft.status) if radar else False,
        "dRel": round(radar.leadLeft.dRel, 1) if radar and radar.leadLeft.status else 0,
        "vLead": round(radar.leadLeft.vLead * 3.6, 1) if radar and radar.leadLeft.status else 0,
      } if radar else {"status": False},
      "leadRight": {
        "status": bool(radar.leadRight.status) if radar else False,
        "dRel": round(radar.leadRight.dRel, 1) if radar and radar.leadRight.status else 0,
        "vLead": round(radar.leadRight.vLead * 3.6, 1) if radar and radar.leadRight.status else 0,
      } if radar else {"status": False},
    }
    return jsonify(data)

  @app.post("/api/experimental/driver_monitoring")
  def set_driver_monitoring():
    params = Params()
    data = request.get_json(force=True)
    enabled = bool(data.get("enabled", True))
    # Common openpilot param for driver monitoring
    params.put_bool("DriverMonitoring", enabled)
    # Also set related FrogPilot params if present
    try:
      params.put_bool("HandsOnWheelMonitoring", enabled)
    except Exception:
      pass
    return jsonify({"status": "ok", "enabled": enabled})

  @app.get("/experimental/roundabout")
  def experimental_roundabout_page():
    return render_template("experimental/roundabout.html")

  @app.get("/api/experimental/roundabout")
  def get_roundabout_settings():
    params = Params()
    try:
      data = {
        "enabled": params.get_bool("RoundaboutYield") if params.check_key("RoundaboutYield") else True,
        "speed_conflict": float(params.get("RoundaboutSpeedConflict") or 14),
        "speed_no_conflict": float(params.get("RoundaboutSpeedNoConflict") or 29),
        "speed_fallback": float(params.get("RoundaboutSpeedFallback") or 18),
        "max_dist": float(params.get("RoundaboutMaxDist") or 20),
        "min_vlead": float(params.get("RoundaboutMinVLead") or 1.0),
        "min_yrel": float(params.get("RoundaboutMinYRel") or 0.8),
        "stop_speed": float(params.get("RoundaboutStopSpeed") or 2),
      }
    except Exception:
      data = {"enabled": True, "speed_conflict": 14, "speed_no_conflict": 29, "speed_fallback": 18,
              "max_dist": 20, "min_vlead": 1.0, "min_yrel": 0.8, "stop_speed": 2}
    return jsonify(data)

  @app.post("/api/experimental/roundabout")
  def set_roundabout_settings():
    params = Params()
    data = request.get_json(force=True)
    try:
      params.put_bool("RoundaboutYield", bool(data.get("enabled", True)))
      params.put("RoundaboutSpeedConflict", str(data.get("speed_conflict", 14)))
      params.put("RoundaboutSpeedNoConflict", str(data.get("speed_no_conflict", 29)))
      params.put("RoundaboutSpeedFallback", str(data.get("speed_fallback", 18)))
      params.put("RoundaboutMaxDist", str(data.get("max_dist", 20)))
      params.put("RoundaboutMinVLead", str(data.get("min_vlead", 1.0)))
      params.put("RoundaboutMinYRel", str(data.get("min_yrel", 0.8)))
      params.put("RoundaboutStopSpeed", str(data.get("stop_speed", 2)))
      return jsonify({"status": "ok"})
    except Exception as e:
      return jsonify({"status": "error", "message": str(e)}), 500

  @app.get("/")
  @app.get("/<path:path>")
  def index(path=""):
    if request.path not in PAGE_PATHS:
      abort(404, "Page not found")

    return render_template("index.html")

  return app


def main():
  app = create_app()
  if not PC:
    threading.Thread(target=device.state.run, name="pond_state", daemon=True).start()
    threading.Thread(target=discovery.mdns_responder, name="pond_discovery", daemon=True).start()
    discovery.redirect_port(PORT)

  app.run(host="0.0.0.0", port=PORT, threaded=True, debug=False)


if __name__ == "__main__":
  main()
