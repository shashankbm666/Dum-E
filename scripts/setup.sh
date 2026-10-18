#!/usr/bin/env bash
set -e

echo "================================================================"
echo "   Dum-E 4-DOF Robotic Arm - Linux/macOS Setup Script"
echo "================================================================"

if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python 3 was not found! Please install python3 and python3-venv."
    exit 1
fi

echo "[1/3] Creating virtual environment (.venv)..."
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi

echo "[2/3] Activating virtual environment..."
source .venv/bin/activate

echo "[3/3] Installing dependencies..."
pip install --upgrade pip --quiet
pip install -r requirements.txt

echo "================================================================"
echo "   SETUP COMPLETE! To start the server, run: ./run.sh"
echo "================================================================"
