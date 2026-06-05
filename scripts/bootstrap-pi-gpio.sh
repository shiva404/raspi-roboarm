#!/usr/bin/env bash
# Install lgpio from Adafruit's pre-built wheel when PyPI would build from source.
# Run on the Pi if `poetry install` fails on lgpio with "swig: No such file".
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "aarch64" ]]; then
  echo "bootstrap-pi-gpio: skipped (not Linux aarch64)"
  exit 0
fi

PY_MAJOR_MINOR="$(poetry run python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
BASE="https://github.com/adafruit/lgpio-python-wheels/raw/main/wheels"

case "$PY_MAJOR_MINOR" in
  3.13) WHEEL="$BASE/lgpio-0.2.2.0-cp313-cp313-linux_aarch64.whl" ;;
  3.14) WHEEL="$BASE/lgpio-0.2.2.0-cp314-cp314-linux_aarch64.whl" ;;
  *)
    echo "bootstrap-pi-gpio: Python $PY_MAJOR_MINOR — PyPI wheel should work; skipping"
    exit 0
    ;;
esac

echo "Installing lgpio wheel for Python $PY_MAJOR_MINOR ..."
poetry run pip install --force-reinstall "$WHEEL"
poetry run pip install --force-reinstall rpi-lgpio
echo "OK — verify with: poetry run python -c \"import board; print('OK')\""
