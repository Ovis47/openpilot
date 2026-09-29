"""前の車が設定速度より速く走るとき、上限付きで設定速度を超えて追従する。

設定速度のままだと、前の車が加速するたびに離されて、そのたびにレバーで設定を上げる必要がある。
前の車の速度まで巡航の上限を一時的に引き上げ、前の車がいなくなったら設定速度へ緩やかに戻す。
/data/lead_speed_follow_disabled があれば無効（起動時に読む）。
"""
import os
from dataclasses import dataclass

from openpilot.common.constants import CV

DISABLE_FLAG_PATH = "/data/lead_speed_follow_disabled"

MIN_EGO_SPEED = 45. * CV.KPH_TO_MS
# 上限はメーター表示（画面の MAX と同じ基準）の値。制御用の速度へは表示との比で換算する
MAX_OVER_SET_KPH = 20.
ABSOLUTE_MAX_KPH = 145.
# 追従していると言える距離。これより遠い車につられて加速しないようにする
MAX_TIME_GAP = 3.0
MIN_MODEL_PROB = 0.5
# 前の車が抜けたとき、引き上げた上限を下げる速さ。急に下げると減速が強く出るため
RELEASE_RATE = 0.5  # m/s^2


@dataclass(frozen=True)
class LeadInput:
  status: bool
  radar: bool
  model_prob: float
  d_rel: float
  v_lead: float


def target_boost(v_ego: float, v_cruise: float, cluster_ratio: float, lead: LeadInput) -> float:
  """引き上げ後の上限 [m/s] を返す。条件を満たさないときは v_cruise のまま。"""
  if v_ego < MIN_EGO_SPEED or not lead.status:
    return v_cruise
  # カメラだけの前車は距離と速度の誤差が大きく、誤って加速すると危ないのでレーダーで捉えたときだけ
  if not lead.radar or lead.model_prob < MIN_MODEL_PROB:
    return v_cruise
  if lead.d_rel > v_ego * MAX_TIME_GAP:
    return v_cruise
  limit = min(v_cruise + MAX_OVER_SET_KPH * CV.KPH_TO_MS * cluster_ratio, ABSOLUTE_MAX_KPH * CV.KPH_TO_MS * cluster_ratio)
  return max(v_cruise, min(lead.v_lead, limit))


class LeadSpeedFollow:
  def __init__(self, dt: float, enabled: bool | None = None):
    self.dt = dt
    self.enabled = (not os.path.exists(DISABLE_FLAG_PATH)) if enabled is None else enabled
    self.v_limit = 0.
    self.v_cruise_prev = 0.

  def update(self, active: bool, v_ego: float, v_cruise: float, v_cruise_kph: float, v_cruise_cluster_kph: float,
             lead: LeadInput) -> float:
    # 設定を下げたときは運転者の意思なので、緩やかに戻さずすぐに従う
    set_lowered = v_cruise < self.v_cruise_prev
    self.v_cruise_prev = v_cruise
    if not self.enabled or not active or set_lowered:
      self.v_limit = v_cruise
      return v_cruise

    # 制御用の設定速度はメーター表示より数%低い。表示基準の上限をこの比で制御用へ換算する
    cluster_ratio = v_cruise_kph / v_cruise_cluster_kph if v_cruise_cluster_kph > 0 else 1.
    target = target_boost(v_ego, v_cruise, cluster_ratio, lead)
    if target >= self.v_limit:
      self.v_limit = target
    else:
      self.v_limit = max(target, self.v_limit - RELEASE_RATE * self.dt)
    return self.v_limit
