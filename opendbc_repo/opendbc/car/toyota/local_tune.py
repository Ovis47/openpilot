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
# 先読みと勾配補正は純正より弱める方向だけ許す（強めると揺れを増やすため）
FUTURE_SCALE_RANGE = (0.0, 1.0)
PITCH_SCALE_RANGE = (0.0, 1.0)


@dataclass(frozen=True)
class LongTune:
  actuator_delay: float | None = None  # None なら車種ごとの既定値を使う
  ki_scale: float = 1.0
  future_scale: float = 1.0  # carcontroller が実加速度を先読みする時間の倍率
  pitch_scale: float = 1.0   # 勾配変化の補正量の倍率


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

  def scale(name: str, bounds: tuple[float, float]) -> float:
    value = _clamped(name, raw[name], bounds) if name in raw else None
    return 1.0 if value is None else value

  delay = _clamped("actuator_delay", raw["actuator_delay"], ACTUATOR_DELAY_RANGE) if "actuator_delay" in raw else None
  tune = LongTune(actuator_delay=delay, ki_scale=scale("ki_scale", KI_SCALE_RANGE),
                  future_scale=scale("future_scale", FUTURE_SCALE_RANGE), pitch_scale=scale("pitch_scale", PITCH_SCALE_RANGE))
  carlog.info(f"long tune loaded: {tune}")
  return tune
