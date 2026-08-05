"""ZMQ REP transport: owns bytes <-> dicts, nothing else.

The reply is encoded in the codec the request arrived in, so a JSON client
(debugging, shell scripts) and the msgpack UE client can share a server.
"""

from __future__ import annotations

import logging

import zmq

from . import protocol
from .server import WorkerServer

log = logging.getLogger("newton_worker")


def serve(endpoint: str, server: WorkerServer, *, context: zmq.Context | None = None) -> None:
    """Blocking REP loop. Returns after a shutdown op (reply is sent first)."""
    own_context = context is None
    ctx = context or zmq.Context.instance()
    socket = ctx.socket(zmq.REP)
    try:
        socket.bind(endpoint)
        log.info("newton_worker serving on %s", endpoint)
        print(f"READY {endpoint}", flush=True)  # spawn handshake line for the UE side
        while not server.shutdown_requested:
            payload = socket.recv()
            try:
                message, codec = protocol.decode(payload)
            except ValueError as exc:
                reply = protocol.error_reply(-1, protocol.ERR_BAD_REQUEST, str(exc))
                codec = protocol.CODEC_JSON
            else:
                reply = server.handle(message)
            socket.send(protocol.encode(reply, codec))
    finally:
        socket.close(linger=0)
        if own_context:
            pass  # Context.instance() is shared; never terminate it here
