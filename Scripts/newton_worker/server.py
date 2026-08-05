"""Request dispatch: protocol ops -> NewtonSim calls.

Transport-agnostic (mirrors URLab's dispatcher/transport split): handle()
takes and returns plain dicts; the ZMQ loop in transport.py owns bytes.
"""

from __future__ import annotations

import base64
import logging
from typing import Any

from . import protocol
from .probe import capabilities
from .sim import NewtonSim, SimError

log = logging.getLogger("newton_worker")


class WorkerServer:
    def __init__(self, sim: NewtonSim | None = None) -> None:
        self.sim = sim if sim is not None else NewtonSim()
        self.shutdown_requested = False

    def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        try:
            request_id, op, params = protocol.validate_request(message)
        except ValueError as exc:
            return protocol.error_reply(-1, protocol.ERR_BAD_REQUEST, str(exc))

        try:
            handler = self._handlers().get(op)
            if handler is None:
                return protocol.error_reply(
                    request_id, protocol.ERR_UNKNOWN_OP, f"unknown op '{op}'"
                )
            return protocol.ok_reply(request_id, handler(params))
        except SimError as exc:
            code = protocol.ERR_NO_MODEL if "no model loaded" in str(exc) else (
                protocol.ERR_LOAD_FAILED if op == protocol.OP_LOAD_MODEL else protocol.ERR_STEP_FAILED
            )
            return protocol.error_reply(request_id, code, str(exc))
        except NotImplementedError as exc:
            return protocol.error_reply(request_id, protocol.ERR_NOT_IMPLEMENTED, str(exc))
        except Exception as exc:
            log.exception("unhandled error in op '%s'", op)
            return protocol.error_reply(
                request_id, protocol.ERR_INTERNAL, f"{type(exc).__name__}: {exc}"
            )

    def _handlers(self):
        return {
            protocol.OP_HELLO: self._op_hello,
            protocol.OP_LOAD_MODEL: self._op_load_model,
            protocol.OP_STEP: self._op_step,
            protocol.OP_RESET: self._op_reset,
            protocol.OP_SET_STATE: self._op_set_state,
            protocol.OP_SHUTDOWN: self._op_shutdown,
        }

    # ------------------------------------------------------------------- ops

    def _op_hello(self, params: dict[str, Any]) -> dict[str, Any]:
        client_protocol = params.get("protocol")
        caps = capabilities()
        caps["protocol_compatible"] = client_protocol == protocol.PROTOCOL_VERSION
        caps["model_loaded"] = self.sim.loaded
        return caps

    def _op_load_model(self, params: dict[str, Any]) -> dict[str, Any]:
        mjcf_xml = params.get("mjcf_xml")
        if not isinstance(mjcf_xml, str) or not mjcf_xml.strip():
            raise SimError("load_model requires non-empty 'mjcf_xml'")
        assets = self._decode_assets(params.get("assets"))
        return self.sim.load(
            mjcf_xml,
            assets=assets,
            solver=params.get("solver", "mujoco"),
            solver_options=params.get("solver_options"),
        )

    def _op_step(self, params: dict[str, Any]) -> dict[str, Any]:
        return self.sim.step(
            ctrl=params.get("ctrl"),
            mocap_pos=params.get("mocap_pos"),
            mocap_quat=params.get("mocap_quat"),
            nsteps=int(params.get("nsteps", 1)),
        )

    def _op_reset(self, params: dict[str, Any]) -> dict[str, Any]:
        return self.sim.reset()

    def _op_set_state(self, params: dict[str, Any]) -> dict[str, Any]:
        raise NotImplementedError("set_state lands with the lifecycle milestone")

    def _op_shutdown(self, params: dict[str, Any]) -> dict[str, Any]:
        self.shutdown_requested = True
        self.sim.close()
        return {"shutting_down": True}

    @staticmethod
    def _decode_assets(raw: Any) -> dict[str, bytes]:
        """Assets arrive as bytes (msgpack) or base64 strings (JSON)."""
        if raw is None:
            return {}
        if not isinstance(raw, dict):
            raise SimError("'assets' must be a map of path -> data")
        decoded: dict[str, bytes] = {}
        for path, data in raw.items():
            if isinstance(data, bytes):
                decoded[path] = data
            elif isinstance(data, str):
                decoded[path] = base64.b64decode(data)
            else:
                raise SimError(f"asset '{path}' must be bytes or base64 string")
        return decoded
