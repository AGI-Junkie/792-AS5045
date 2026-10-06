#!/bin/bash
# SessionStart hook for Claude Code cloud sessions.
#
# Installs the .NET 10 SDK (the WPF host in Commissioning/ targets net10.0-windows) and restores
# its NuGet packages, so `dotnet build` works in the Linux container. WPF cannot *run* on Linux,
# so the host self-tests (`--self-test`, `--language-self-test`) and the MSVC-based firmware tests
# (tests/test_stage1.py) still need Windows; CI (.github/workflows/host-ci.yml) covers those.
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
DOTNET_DIR="${DOTNET_INSTALL_DIR:-$HOME/.dotnet}"
DOTNET_CHANNEL="10.0"

has_dotnet_10() {
  [ -x "$DOTNET_DIR/dotnet" ] && "$DOTNET_DIR/dotnet" --list-sdks 2>/dev/null | grep -q '^10\.'
}

if has_dotnet_10; then
  echo "[session-start] .NET 10 SDK already installed in $DOTNET_DIR"
else
  echo "[session-start] Installing .NET SDK $DOTNET_CHANNEL into $DOTNET_DIR"
  installer="$(mktemp -d)/dotnet-install.sh"
  curl -fsSL --retry 4 --retry-delay 2 --retry-all-errors -o "$installer" https://dot.net/v1/dotnet-install.sh
  head -n 1 "$installer" | grep -q '^#!' || { echo "[session-start] unexpected dotnet-install.sh content" >&2; exit 1; }
  bash "$installer" --channel "$DOTNET_CHANNEL" --install-dir "$DOTNET_DIR" --no-path
  rm -f "$installer"
fi

export DOTNET_ROOT="$DOTNET_DIR"
export PATH="$DOTNET_DIR:$PATH"
export DOTNET_CLI_TELEMETRY_OPTOUT=1
export DOTNET_NOLOGO=1
export EnableWindowsTargeting=true

# Persist for the session (the Bash tool sources this file).
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  {
    echo "export DOTNET_ROOT=\"$DOTNET_DIR\""
    echo "export PATH=\"$DOTNET_DIR:\$PATH\""
    echo "export DOTNET_CLI_TELEMETRY_OPTOUT=1"
    echo "export DOTNET_NOLOGO=1"
    # Lets `dotnet build` of the net10.0-windows project work on Linux (compile only, no run).
    echo "export EnableWindowsTargeting=true"
  } >> "$CLAUDE_ENV_FILE"
fi

# Restore NuGet packages (System.IO.Ports + the Windows desktop reference pack) so the container
# cache already holds them. Restore is idempotent.
dotnet restore "$PROJECT_DIR/Commissioning/Commissioning.csproj" --nologo -v quiet

echo "[session-start] done: $(dotnet --version)"
