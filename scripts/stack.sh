#!/usr/bin/env bash
# Deprecated wrapper — logic lives in entrypoint.sh
exec "$(dirname "$0")/../entrypoint.sh" "$@"
