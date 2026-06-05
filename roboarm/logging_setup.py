"""Central logging configuration.

One place to control how chatty the whole stack is. Use ``-v`` / ``-vv`` on the
CLI, or set the ``ROBOARM_LOG`` environment variable (DEBUG/INFO/WARNING/...).
"""

from __future__ import annotations

import logging
import os

from rich.logging import RichHandler

_CONFIGURED = False


def configure_logging(level: int | str | None = None) -> None:
    """Configure root logging once, with pretty Rich output.

    Resolution order for the level: explicit ``level`` arg > ``ROBOARM_LOG``
    env var > INFO.
    """
    global _CONFIGURED

    if level is None:
        level = os.environ.get("ROBOARM_LOG", "INFO")
    if isinstance(level, str):
        level = logging.getLevelName(level.upper())

    handler = RichHandler(
        rich_tracebacks=True,
        show_path=False,
        markup=True,
        log_time_format="[%X]",
    )

    root = logging.getLogger()
    # Reconfigure cleanly if called more than once (e.g. CLI then REPL).
    for existing in list(root.handlers):
        root.removeHandler(existing)
    root.addHandler(handler)
    root.setLevel(level)

    _CONFIGURED = True


def get_logger(name: str) -> logging.Logger:
    if not _CONFIGURED:
        configure_logging()
    return logging.getLogger(name)
