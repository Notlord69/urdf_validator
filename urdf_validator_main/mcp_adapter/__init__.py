"""Open MCP adapter (PRD Q2 §3.13): the existing capability surface over MCP.

A thin, local, stdio-based adapter around the v1.0-v1.4 pipeline and `api/`
surface. It adds **no validation logic** — every number it returns is produced
by the existing parser/physics/checks/report modules (INV-1).

This package deliberately imports **nothing** at package-import time: the
optional `mcp` SDK is imported lazily inside `server.main()` (the mujoco/xacro
precedent), so `import urdf_validator_main.mcp_adapter` succeeds on a core
install and changes no existing behavior (INV-3).

Entry point: `urdf_validate_mcp` -> `urdf_validator_main.mcp_adapter.server:main`.
"""
