#!/usr/bin/env python3
"""
BlockSI Dashboard — NiceGUI, Recipe-Based Sequence Protocol
Run with:  .venv\\Scripts\\python.exe blocksi_dashboard.py [--port 5000]

Architecture ("ESP32 = Arms, PC = Brains"):
  - ESP32 connects to PC via TCP on port 5000
  - PC generates recipes (step lists, prompts) and sends to ESP32
  - ESP32 executes blindly: set power, count samples, stream SEQ messages
  - PC does ALL analysis: statistics, pass/fail, model fitting, CSV saving
  - PC is sole authority for power_target_pct (never from telemetry)

CRITICAL: Commands are COMMA-separated  →  CMD,power_set,50\\n
          NEVER use colons (CMD,power_set:50) — silently fails on ESP32.
"""
from __future__ import annotations

import argparse
import asyncio

from nicegui import ui, app

from dashboard.state import S, DEFAULT_PORT
from dashboard.tcp_server import TCPServer
import dashboard.tcp_server as _tcp_mod
import dashboard.ui_main  # noqa: F401 — registers @ui.page("/")


async def _startup() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args, _ = parser.parse_known_args()
    S.load_model_for_current_condition()
    S.load_cstr_model()
    tcp = TCPServer(args.port)
    _tcp_mod.tcp = tcp
    await tcp.start()


app.on_startup(_startup)

ui.run(title="BlockSI Control", port=8080, reload=False)
