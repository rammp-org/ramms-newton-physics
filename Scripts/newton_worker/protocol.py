"""Wire protocol for the RAMMS Newton worker.

One request map, one reply map per ZMQ REQ/REP exchange. Encoded as msgpack
when available (the UE client's default), JSON otherwise; the server replies
in whatever codec the request arrived in.

Request:  {"id": <int>, "op": <str>, "params": {...}}
Reply:    {"id": <int>, "ok": true,  "result": {...}}
       or {"id": <int>, "ok": false, "error": {"code": <str>, "message": <str>}}

Array-valued fields (ctrl, qpos, ...) are flat lists of floats in v1.
"""

from __future__ import annotations

import json
from typing import Any

try:
    import msgpack

    HAVE_MSGPACK = True
except ImportError:  # pragma: no cover - msgpack is in the pinned env
    msgpack = None
    HAVE_MSGPACK = False

PROTOCOL_VERSION = 1

# Ops
OP_HELLO = "hello"
OP_LOAD_MODEL = "load_model"
OP_STEP = "step"
OP_RESET = "reset"
OP_SET_STATE = "set_state"
OP_SHUTDOWN = "shutdown"

ALL_OPS = (OP_HELLO, OP_LOAD_MODEL, OP_STEP, OP_RESET, OP_SET_STATE, OP_SHUTDOWN)

# Error codes
ERR_BAD_REQUEST = "bad_request"
ERR_UNKNOWN_OP = "unknown_op"
ERR_NO_MODEL = "no_model"
ERR_LOAD_FAILED = "load_failed"
ERR_STEP_FAILED = "step_failed"
ERR_NOT_IMPLEMENTED = "not_implemented"
ERR_INTERNAL = "internal"

CODEC_MSGPACK = "msgpack"
CODEC_JSON = "json"


def ok_reply(request_id: int, result: dict[str, Any]) -> dict[str, Any]:
    return {"id": request_id, "ok": True, "result": result}


def error_reply(request_id: int, code: str, message: str) -> dict[str, Any]:
    return {"id": request_id, "ok": False, "error": {"code": code, "message": message}}


def encode(message: dict[str, Any], codec: str) -> bytes:
    if codec == CODEC_MSGPACK:
        if not HAVE_MSGPACK:
            raise RuntimeError("msgpack requested but not installed")
        return msgpack.packb(message, use_bin_type=True)
    return json.dumps(message).encode("utf-8")


def decode(payload: bytes) -> tuple[dict[str, Any], str]:
    """Decode a request, sniffing the codec. Returns (message, codec)."""
    if HAVE_MSGPACK:
        try:
            message = msgpack.unpackb(payload, raw=False)
            if isinstance(message, dict):
                return message, CODEC_MSGPACK
        except Exception:
            pass
    try:
        message = json.loads(payload.decode("utf-8"))
    except Exception as exc:
        raise ValueError(f"payload is neither msgpack nor JSON: {exc}") from exc
    if not isinstance(message, dict):
        raise ValueError("decoded payload is not a map")
    return message, CODEC_JSON


def validate_request(message: dict[str, Any]) -> tuple[int, str, dict[str, Any]]:
    """Returns (id, op, params) or raises ValueError."""
    request_id = message.get("id")
    if not isinstance(request_id, int):
        raise ValueError("missing or non-integer 'id'")
    op = message.get("op")
    if not isinstance(op, str) or not op:
        raise ValueError("missing 'op'")
    params = message.get("params", {})
    if params is None:
        params = {}
    if not isinstance(params, dict):
        raise ValueError("'params' must be a map")
    return request_id, op, params
