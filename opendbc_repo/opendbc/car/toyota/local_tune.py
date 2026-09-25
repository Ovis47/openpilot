"""この端末だけの縦制御チューニング値を /data/long_tune.json から読む。

Params のキー追加は C++ の再ビルドが必要なため、音量設定と同じくファイルで持つ。
値は起動時（車種判定と CarController の生成時）に読むので、変更は次に車の電源を入れたときから効く。
"""
import json
import math
from dataclasses import dataclass

from opendbc.car.carlog import carlog

LONG_TUNE_PATH = "/data/long_tune.json"
# 誤入力で制御が極端になる事故を防ぐため、範囲外の値はこの範囲に丸める
ACTUATOR_DELAY_RANGE = (0.05, 0.5)
KI_SCALE_RANGE = (0.1, 1.5)


@dataclass(frozen=True)
class LongTune:
  actuator_delay: float | None = None  # None なら車種ごとの既定値を使う
  ki_scale: float = 1.0


def _clamped(name: str, value: object, bounds: tuple[float, float]) -> float | None:
  if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
    carlog.warning(f"long tune: invalid {name}={value!r}, ignored")
    return None
  clamped = min(max(float(value), bounds[0]), bounds[1])
  if clamped != value:
    carlog.warning(f"long tune: {name}={value} out of range, clamped to {clamped}")
  return clamped


def load_long_tune(path: str | None = None) -> LongTune:
  path = path or LONG_TUNE_PATH
  try:
    with open(path) as f:
      raw = json.load(f)
  except FileNotFoundError:
    return LongTune()
  except (OSError, ValueError) as e:
    # 壊れた設定では純正の値で走る方が安全なので、既定値に戻す
    carlog.warning(f"long tune: failed to load {path}: {e}")
    return LongTune()
  if not isinstance(raw, dict):
    carlog.warning(f"long tune: {path} must be an object")
    return LongTune()

  delay = _clamped("actuator_delay", raw["actuator_delay"], ACTUATOR_DELAY_RANGE) if "actuator_delay" in raw else None
  ki_scale = _clamped("ki_scale", raw["ki_scale"], KI_SCALE_RANGE) if "ki_scale" in raw else None
  tune = LongTune(actuator_delay=delay, ki_scale=1.0 if ki_scale is None else ki_scale)
  carlog.info(f"long tune loaded: {tune}")
  return tune
