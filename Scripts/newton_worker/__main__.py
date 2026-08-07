"""CLI entry point.

  python -m newton_worker --probe
      Print one JSON capability line to stdout and exit (never fails on
      missing packages; availability is in the payload).

  python -m newton_worker serve --endpoint tcp://127.0.0.1:5580
      Run the blocking ZMQ REP server. Prints "READY <endpoint>" to stdout
      once bound; exits after a shutdown op.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # Dispatched before argparse: REMAINDER-style forwarding through a
    # subparser drops leading optionals (--mjcf ...) on Python 3.12.
    if argv and argv[0] == "parity":
        from . import parity

        return parity.main(argv[1:])

    parser = argparse.ArgumentParser(prog="newton_worker")
    parser.add_argument("--probe", action="store_true", help="print capability JSON and exit")
    parser.add_argument("--verbose", action="store_true")
    subparsers = parser.add_subparsers(dest="command")
    serve_parser = subparsers.add_parser("serve", help="run the ZMQ REP server")
    serve_parser.add_argument("--endpoint", default="tcp://127.0.0.1:5580")
    canary_parser = subparsers.add_parser(
        "canary", help="load+step a tiny model in a subprocess; print JSON verdict"
    )
    canary_parser.add_argument("--solver", default="mujoco")
    subparsers.add_parser(
        "parity",
        help="qpos-trace parity: plain mj_step vs the worker (own --help; see parity.py)",
        add_help=False,
    )
    inner_parser = subparsers.add_parser("canary-inner")  # internal: runs in-process
    inner_parser.add_argument("--solver", default="mujoco")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[newton_worker] %(levelname)s %(message)s",
        stream=sys.stderr,
    )
    # Mesh importers (trimesh) warn per degenerate face — on real scenes that
    # is thousands of lines into a pipe the UE client must drain. Keep it down
    # unless explicitly debugging.
    if not args.verbose:
        logging.getLogger("trimesh").setLevel(logging.ERROR)

    if args.probe:
        from .probe import capabilities

        print(json.dumps(capabilities()), flush=True)
        return 0

    if args.command == "serve":
        from .server import WorkerServer
        from .transport import serve

        serve(args.endpoint, WorkerServer())
        return 0

    if args.command == "canary":
        from .probe import run_canary

        verdict = run_canary(solver=args.solver)
        print(json.dumps(verdict), flush=True)
        return 0 if verdict["ok"] else 1

    if args.command == "canary-inner":
        from .probe import CANARY_MJCF
        from .sim import NewtonSim

        sim = NewtonSim()
        sim.load(CANARY_MJCF, solver=args.solver)
        state = sim.step(ctrl=[0.0], nsteps=50)
        sim.close()
        if state["qpos"][0] == 0.0 and state["qvel"][0] == 0.0:
            # Loaded but frozen: silent-miscompile / dead-readback signature.
            # The rod must sag under gravity; "no crash" is not "alive".
            print("CANARY_DEAD", flush=True)
            return 1
        print("CANARY_OK", flush=True)
        return 0

    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
