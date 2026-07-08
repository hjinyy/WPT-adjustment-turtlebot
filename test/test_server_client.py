import json
from unittest.mock import MagicMock, patch

from wpt_adjustment_turtlebot.server_client import ServerClient


def _mock_response(payload):
    response = MagicMock()
    response.read.return_value = json.dumps(payload).encode("utf-8")
    response.__enter__.return_value = response
    return response


@patch("wpt_adjustment_turtlebot.server_client.urllib.request.urlopen")
def test_post_event_sends_expected_body(mock_urlopen):
    mock_urlopen.return_value = _mock_response({"ok": True})
    client = ServerClient(base_url="http://tserver.local:8000", robot_id="TB3-01")

    result = client.post_event(node_id="A02", battery_percent=76.0, alignment_state="Locked")

    assert result == {"ok": True}
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://tserver.local:8000/api/robot/events"
    assert request.method == "POST"
    body = json.loads(request.data.decode("utf-8"))
    assert body["robot_id"] == "TB3-01"
    assert body["node_id"] == "A02"
    assert body["battery_percent"] == 76.0
    assert body["alignment_state"] == "Locked"


@patch("wpt_adjustment_turtlebot.server_client.urllib.request.urlopen")
def test_next_command_uses_robot_id_in_path(mock_urlopen):
    mock_urlopen.return_value = _mock_response({"command": "navigate_to", "target_node_id": "B02"})
    client = ServerClient(robot_id="TB3-01")

    result = client.next_command()

    assert result["target_node_id"] == "B02"
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://tserver.local:8000/api/robots/TB3-01/commands/next"
    assert request.method == "GET"


@patch("wpt_adjustment_turtlebot.server_client.urllib.request.urlopen")
def test_ack_command_sends_status(mock_urlopen):
    mock_urlopen.return_value = _mock_response({"ok": True})
    client = ServerClient(robot_id="TB3-01")

    client.ack_command("cmd-123", status="failed", message="tag lost")

    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://tserver.local:8000/api/robots/TB3-01/commands/cmd-123/ack"
    body = json.loads(request.data.decode("utf-8"))
    assert body == {"status": "failed", "message": "tag lost"}
