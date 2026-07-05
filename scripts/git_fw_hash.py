#!/usr/bin/env python3
"""git_fw_hash.py — ビルド時に git 短縮ハッシュを -DBSL_FW_GIT として注入する
(UDP telemetry Phase1 計画書 §3.1: schema v1 の "fw" フィールド)。

platformio.ini の build_flags から `!python3 scripts/git_fw_hash.py` として
呼ばれ、標準出力に単一の -D フラグを印字する。git が使えない/リポジトリ外
などで失敗した場合は "unknown" にフォールバックし、ビルド自体は止めない
(telemetry_task.cpp 側にも同じフォールバック #ifndef BSL_FW_GIT を保険で持つ)。
"""
import subprocess


def git_short_hash() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short=8", "HEAD"],
            stderr=subprocess.DEVNULL,
        )
        h = out.decode().strip()
        return h if h else "unknown"
    except Exception:
        return "unknown"


def main() -> None:
    print('-DBSL_FW_GIT=\\"' + git_short_hash() + '\\"')


if __name__ == "__main__":
    main()
