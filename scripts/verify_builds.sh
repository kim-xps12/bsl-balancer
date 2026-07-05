#!/usr/bin/env bash
# scripts/verify_builds.sh — UDP telemetry Phase1 の hermetic ビルド検証 (計画書 §5)。
#
# 常に以下の順で実行する (ゲート1第2回指摘: 残留ファイルで片経路が未検証になる問題の
# 排除。ローカル状態に依存せず enabled/disabled 両経路を毎回検証する):
#   (0) platform / framework の解決済みバージョンを assert (不一致で fail。
#       計画書 §3.1 ゲート1第9回・第10回指摘: platform 更新時に Wi-Fi 安全ガードが
#       依存する first_connect one-shot 挙動の再確認を強制する)
#   (1) 既存 include/wifi_secrets.h を退避 → secrets なしビルド (no-op 経路)
#   (2) wifi_secrets.h.example からダミー生成 → 有効経路ビルド
#   (3) 退避した元ファイルを復元 (元々存在しなければダミーを削除するのみ)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PIO="${PIO:-$HOME/.platformio/penv/bin/pio}"
SECRETS="include/wifi_secrets.h"
EXAMPLE="include/wifi_secrets.h.example"
BACKUP_DIR="$(mktemp -d)"
BACKUP="$BACKUP_DIR/wifi_secrets.h.bak"

# 2026-07-05 実装時に `pio pkg list -e m5stack-core2` で確認した解決済みバージョン
# (platformio.ini のコメント参照)。
EXPECTED_PLATFORM="espressif32 @ 6.12.0"
EXPECTED_FRAMEWORK="framework-arduinoespressif32 @ 3.20017.241212"

had_secrets=0
restore_secrets() {
  if [ "$had_secrets" -eq 1 ]; then
    mv -f "$BACKUP" "$SECRETS"
    echo "[verify_builds] (3) restored original $SECRETS"
  else
    rm -f "$SECRETS"
    echo "[verify_builds] (3) removed dummy $SECRETS (no original existed)"
  fi
  rm -rf "$BACKUP_DIR"
}
trap restore_secrets EXIT

if [ -f "$SECRETS" ]; then
  had_secrets=1
  mv "$SECRETS" "$BACKUP"
fi

echo "[verify_builds] (0) platform/framework バージョンを確認"
PKG_LIST="$("$PIO" pkg list -e m5stack-core2)"
echo "$PKG_LIST"
if ! echo "$PKG_LIST" | grep -qF "$EXPECTED_PLATFORM"; then
  echo "[verify_builds] FAIL: platform version mismatch (expected: $EXPECTED_PLATFORM)" >&2
  exit 1
fi
if ! echo "$PKG_LIST" | grep -qF "$EXPECTED_FRAMEWORK"; then
  echo "[verify_builds] FAIL: framework version mismatch (expected: $EXPECTED_FRAMEWORK)" >&2
  exit 1
fi
echo "[verify_builds] OK: ${EXPECTED_PLATFORM} / ${EXPECTED_FRAMEWORK}"

echo "[verify_builds] (1) secrets なしビルド (no-op 経路)"
rm -f "$SECRETS"
"$PIO" run -e m5stack-core2

echo "[verify_builds] (2) example からダミー生成 → 有効経路ビルド"
cp "$EXAMPLE" "$SECRETS"
"$PIO" run -e m5stack-core2

echo "[verify_builds] OK: enabled/disabled 両経路のビルドに成功"
