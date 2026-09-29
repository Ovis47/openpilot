import pytest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.lead_speed_follow import LeadInput, LeadSpeedFollow, RELEASE_RATE

DT = 0.05
# 実車の記録で、制御用の設定速度はメーター表示の約 0.948 倍
RATIO = 137.4 / 145.


def kph(v):
  return v * CV.KPH_TO_MS


def lead(v_lead_kph, d_rel=30., radar=True, prob=0.9, status=True):
  return LeadInput(status, radar, prob, d_rel, kph(v_lead_kph))


def step(lsf, set_cluster_kph, v_ego_kph, lead_in, active=True):
  set_kph = set_cluster_kph * RATIO
  return lsf.update(active, kph(v_ego_kph), kph(set_kph), set_kph, set_cluster_kph, lead_in) * CV.MS_TO_KPH / RATIO


class TestLeadSpeedFollow:
  def test_follows_faster_lead_up_to_plus_20(self):
    lsf = LeadSpeedFollow(DT, enabled=True)
    assert step(lsf, 100, 100, lead(110 * RATIO)) == pytest.approx(110, abs=0.1)
    assert step(lsf, 100, 100, lead(150 * RATIO)) == pytest.approx(120, abs=0.1)

  def test_absolute_max_145(self):
    lsf = LeadSpeedFollow(DT, enabled=True)
    assert step(lsf, 140, 130, lead(170 * RATIO)) == pytest.approx(145, abs=0.1)

  @pytest.mark.parametrize("lead_in,v_ego", [
    (lead(80, radar=False), 60),         # カメラだけの前車
    (lead(80, prob=0.3), 60),            # 前車らしさが低い
    (lead(80, d_rel=100.), 60),          # 追従していない遠い車
    (lead(80, status=False), 60),        # 前車なし
    (lead(80), 40),                      # 45km/h 未満
  ])
  def test_no_boost(self, lead_in, v_ego):
    lsf = LeadSpeedFollow(DT, enabled=True)
    assert step(lsf, 60, v_ego, lead_in) == pytest.approx(60, abs=0.1)

  def test_slower_lead_keeps_set_speed(self):
    lsf = LeadSpeedFollow(DT, enabled=True)
    assert step(lsf, 100, 90, lead(80)) == pytest.approx(100, abs=0.1)

  def test_release_is_gradual(self):
    lsf = LeadSpeedFollow(DT, enabled=True)
    step(lsf, 100, 110, lead(120 * RATIO))
    after = step(lsf, 100, 110, lead(0, status=False))
    expected_drop = RELEASE_RATE * DT * CV.MS_TO_KPH / RATIO
    assert after == pytest.approx(120 - expected_drop, abs=0.01)
    for _ in range(2000):
      after = step(lsf, 100, 110, lead(0, status=False))
    assert after == pytest.approx(100, abs=0.1)

  def test_lowering_set_speed_applies_immediately(self):
    lsf = LeadSpeedFollow(DT, enabled=True)
    step(lsf, 100, 110, lead(120 * RATIO))
    assert step(lsf, 90, 110, lead(0, status=False)) == pytest.approx(90, abs=0.1)

  def test_inactive_or_disabled(self):
    lsf = LeadSpeedFollow(DT, enabled=True)
    assert step(lsf, 100, 100, lead(120), active=False) == pytest.approx(100, abs=0.1)
    lsf = LeadSpeedFollow(DT, enabled=False)
    assert step(lsf, 100, 100, lead(120)) == pytest.approx(100, abs=0.1)
