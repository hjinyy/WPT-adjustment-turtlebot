"""HTTP client for the charging-control server (see GET /openapi.json on the
server for the full contract).

Robot -> server: status pushes via post_event().
Server -> robot: navigation/control commands, fetched via next_command() and
acknowledged via ack_command() once executed.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any

DEFAULT_BASE_URL = "http://tserver.local:8000"
DEFAULT_ROBOT_ID = "TB3-01"


class ServerClient:
    def __init__(
        self,
        base_url: str = DEFAULT_BASE_URL,
        robot_id: str = DEFAULT_ROBOT_ID,
        timeout_s: float = 5.0,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.timeout_s = timeout_s

    def _request(self, method: str, path: str, body: dict | None = None) -> Any:
        url = f"{self.base_url}{path}"
        data = json.dumps(body).encode("utf-8") if body is not None else None
        request = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            raw = response.read()
            return json.loads(raw) if raw else None

    def post_event(
        self,
        node_id: str | None = None,
        target_node_id: str | None = None,
        battery_percent: float | None = None,
        battery_voltage: float | None = None,
        charging: bool | None = None,
        alignment_state: str | None = None,
        detected_tag_ids: list[str] | None = None,
        mode: str | None = None,
        message: str | None = None,
        severity: str = "Info",
        *,
        command_id: str | None = None,
        command_status: str | None = None,
        phase: str | None = None,
        heading: str | None = None,
        linear_velocity: float | None = None,
        angular_velocity: float | None = None,
    ) -> Any:
        body = {
            "robot_id": self.robot_id,
            "node_id": node_id,
            "target_node_id": target_node_id,
            "battery_percent": battery_percent,
            "battery_voltage": battery_voltage,
            "charging": charging,
            "alignment_state": alignment_state,
            "detected_tag_ids": detected_tag_ids or [],
            "mode": mode,
            "severity": severity,
            "message": message,
            # 10 Hz telemetry fields (optional; server stores latest per robot)
            "command_id": command_id,
            "command_status": command_status,
            "phase": phase,
            "heading": heading,
            "linear_velocity": linear_velocity,
            "angular_velocity": angular_velocity,
        }
        return self._request("POST", "/api/robot/events", body)

    def post_status(
        self,
        *,
        phase: str,
        node_id: str | None = None,
        target_node_id: str | None = None,
        heading: str | None = None,
        linear_velocity: float = 0.0,
        angular_velocity: float = 0.0,
        alignment_state: str | None = None,
        detected_tag_ids: list[str] | None = None,
        charging: bool | None = None,
        battery_percent: float | None = None,
        battery_voltage: float | None = None,
        command_id: str | None = None,
        command_status: str | None = None,
    ) -> Any:
        """Lightweight 10 Hz telemetry heartbeat.

        No `message`/elevated severity, so the server updates the live robot
        state but does NOT append an event-log row (avoids 10 rows/sec spam).
        Send discrete milestones (arrival, lock, fault) via post_event with a
        message instead.
        """
        return self.post_event(
            node_id=node_id,
            target_node_id=target_node_id,
            battery_percent=battery_percent,
            battery_voltage=battery_voltage,
            charging=charging,
            alignment_state=alignment_state,
            detected_tag_ids=detected_tag_ids,
            mode="Auto",
            command_id=command_id,
            command_status=command_status,
            phase=phase,
            heading=heading,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )

    def next_command(self) -> dict | None:
        """Return the next queued command for this robot, or None if there isn't one.

        The MACS server wraps the command in an envelope:
            {"command": {"id": ..., "command": "navigate_to", "targetNodeId": ...}}
        or {"command": null} when the queue is empty. Unwrap it here so callers
        get the inner command dict (camelCase fields) directly.
        """
        response = self._request("GET", f"/api/robots/{self.robot_id}/commands/next")
        if not response:
            return None
        return response.get("command")

    def ack_command(self, command_id: str, status: str = "acked", message: str | None = None) -> Any:
        return self._request(
            "POST",
            f"/api/robots/{self.robot_id}/commands/{command_id}/ack",
            {"status": status, "message": message},
        )
