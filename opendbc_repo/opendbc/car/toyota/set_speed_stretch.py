"""車側の設定速度を、openpilot の縦制御で使う目標速度に読み替える。

Prius（TSS-P）の車側の設定速度は 115km/h が上限で、120km/h 区間の高速では足りない。
クルコンのレバー操作は CAN に出ないため、openpilot 側で加算はできない。
そこで車側の 100〜115km/h を 100〜145km/h に引き伸ばす（レバー1回で 3km/h）。
上限の 145km/h は openpilot の V_CRUISE_MAX と同じ。これを超えても速度計画で切り捨てられるため。
"""
STRETCH_START_KPH = 100.
PCM_SET_SPEED_MAX_KPH = 115.
STRETCHED_MAX_KPH = 145.

_STRETCH_RATIO = (STRETCHED_MAX_KPH - STRETCH_START_KPH) / (PCM_SET_SPEED_MAX_KPH - STRETCH_START_KPH)


def stretch_set_speed_kph(pcm_set_speed_kph: float) -> float:
  if pcm_set_speed_kph <= STRETCH_START_KPH:
    return pcm_set_speed_kph
  return min(STRETCH_START_KPH + (pcm_set_speed_kph - STRETCH_START_KPH) * _STRETCH_RATIO, STRETCHED_MAX_KPH)
