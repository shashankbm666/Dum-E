#!/usr/bin/env bash
if [ ! -f ".venv/bin/activate" ]; then
    echo "[ERROR] Virtual environment not found. Please run ./setup.sh first!"
    exit 1
fi

source .venv/bin/activate

# Open browser if on macOS or Linux with xdg-open
if command -v xdg-open &> /dev/null; then
    (sleep 2 && xdg-open http://localhost:8000/) &
elif command -v open &> /dev/null; then
    (sleep 2 && open http://localhost:8000/) &
fi

python3 run_server.py
