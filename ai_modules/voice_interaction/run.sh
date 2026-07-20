#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-7860}"

if [[ -f .env ]]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

if [[ -z "${OPENAI_API_KEY:-}" || "${OPENAI_API_KEY}" == "sk-your-openai-key-here" ]]; then
  echo "OPENAI_API_KEY is missing or still uses the placeholder value."
  echo "Edit .env and set a valid OpenAI API key before running."
  exit 1
fi

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-7860}"

uv run python bot.py -t webrtc --host "$HOST" --port "$PORT"
