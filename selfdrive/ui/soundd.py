import json
import math
import os
import numpy as np
import time
import wave


from cereal import car, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog

from openpilot.system import micd
from openpilot.system.hardware import HARDWARE

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1
ALERT_RAMP_TIME = 4 # seconds to ramp to max volume for warningImmediate
SELFDRIVE_STATE_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 24 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

VOLUME_BASE = 20
if HARDWARE.get_device_type() == "tizi":
  AMBIENT_DB = 30
  VOLUME_BASE = 10

AudibleAlert = car.CarControl.HUDControl.AudibleAlert


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("prompt.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("prompt.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("prompt_distracted.wav", None, MAX_VOLUME),

  AudibleAlert.warningSoft: ("warning_soft.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("warning_immediate.wav", None, MAX_VOLUME),
}
if HARDWARE.get_device_type() == "tizi":
  sound_list.update({
    AudibleAlert.engage: ("engage_tizi.wav", 1, MAX_VOLUME),
    AudibleAlert.disengage: ("disengage_tizi.wav", 1, MAX_VOLUME),
  })

# 警報ごとの音量倍率。Params のキー追加は C++ の再ビルドが必要なため /data のファイルで持つ
ALERT_VOLUME_PATH = "/data/alert_volume.json"
ALERT_VOLUME_MAX_SCALE = 2.0
ALERT_VOLUME_RELOAD_INTERVAL = 1.0  # seconds
# 設定ミスで安全に関わる警報が聞こえなくなる事故を防ぐため、種類ごとに倍率の下限を設ける
ALERT_VOLUME_MIN_SCALE: dict[int, float] = {
  AudibleAlert.engage: 0.0,
  AudibleAlert.disengage: 0.0,
  AudibleAlert.refuse: 0.0,
  AudibleAlert.prompt: 0.1,
  AudibleAlert.promptRepeat: 0.1,
  AudibleAlert.promptDistracted: 0.1,
  AudibleAlert.warningSoft: 0.1,
  AudibleAlert.warningImmediate: 0.1,
}


ALERT_BY_NAME: dict[str, int] = {name: alert for name, alert in AudibleAlert.schema.enumerants.items() if alert in ALERT_VOLUME_MIN_SCALE}
ALERT_NAME: dict[int, str] = {alert: name for name, alert in ALERT_BY_NAME.items()}


def clamp_alert_scale(alert: int, scale: float) -> float:
  return min(max(scale, ALERT_VOLUME_MIN_SCALE[alert]), ALERT_VOLUME_MAX_SCALE)


def parse_alert_volume(raw: dict) -> dict[int, float]:
  scales: dict[int, float] = {}
  for name, value in raw.items():
    if name not in ALERT_BY_NAME:
      cloudlog.warning(f"alert volume: unknown alert {name!r}, ignored")
      continue
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
      cloudlog.warning(f"alert volume: invalid value for {name!r}: {value!r}, ignored")
      continue
    scales[ALERT_BY_NAME[name]] = clamp_alert_scale(ALERT_BY_NAME[name], float(value))
  return scales


class AlertVolume:
  def __init__(self, path: str = ALERT_VOLUME_PATH):
    self.path = path
    self.scales: dict[int, float] = {}
    self.loaded_mtime: float | None = None
    self.last_check = 0.

  def scale(self, alert: int) -> float:
    return self.scales.get(alert, 1.0)

  def reload_if_changed(self, now: float) -> None:
    if now - self.last_check < ALERT_VOLUME_RELOAD_INTERVAL:
      return
    self.last_check = now
    try:
      mtime = os.path.getmtime(self.path)
    except FileNotFoundError:
      if self.loaded_mtime is not None:
        self.scales, self.loaded_mtime = {}, None
      return
    if mtime == self.loaded_mtime:
      return
    self.loaded_mtime = mtime
    try:
      with open(self.path) as f:
        raw = json.load(f)
      if not isinstance(raw, dict):
        raise ValueError("top level must be an object")
    except (OSError, ValueError) as e:
      # 壊れた設定で音が止まるのを避け、直前の倍率を使い続ける
      cloudlog.warning(f"alert volume: failed to load {self.path}: {e}")
      return
    self.scales = parse_alert_volume(raw)
    cloudlog.info(f"alert volume loaded: { {ALERT_NAME[a]: v for a, v in self.scales.items()} }")


def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time['selfdriveState']

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if sm['selfdriveState'].enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10:
      return True

  return False


class Soundd:
  def __init__(self):
    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.ramp_start_volume = MIN_VOLUME
    self.ramp_start_time = 0.

    self.selfdrive_timeout_alert = False
    self.alert_volume = AlertVolume()

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      with wave.open(BASEDIR + "/selfdrive/assets/sounds/" + filename, 'r') as wavefile:
        assert wavefile.getnchannels() == 1
        assert wavefile.getsampwidth() == 2
        assert wavefile.getframerate() == SAMPLE_RATE

        length = wavefile.getnframes()
        self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      num_loops = sound_list[self.current_alert][1]
      sound_data = self.loaded_sounds[self.current_alert]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write

    return ret * self.output_volume()

  def output_volume(self) -> float:
    return min(self.current_volume * self.alert_volume.scale(self.current_alert), MAX_VOLUME)

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    current_alert_played_once = self.current_alert == AudibleAlert.none or self.current_sound_frame > len(self.loaded_sounds[self.current_alert])
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
      if new_alert == AudibleAlert.warningImmediate:
        self.ramp_start_volume = self.current_volume
        self.ramp_start_time = time.monotonic()
      self.current_alert = new_alert
      self.current_sound_frame = 0

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      self.update_alert(new_alert)
    elif check_selfdrive_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      self.selfdrive_timeout_alert = True
    elif self.selfdrive_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.selfdrive_timeout_alert = False

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(VOLUME_BASE, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd

    sm = messaging.SubMaster(['selfdriveState', 'soundPressure'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)

        # Always update volume, even when alert is playing
        if sm.updated['soundPressure']:
          self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x))

        self.alert_volume.reload_if_changed(time.monotonic())
        self.get_audible_alert(sm)

        # Ramp up immediate warning sound over 4s
        if self.current_alert == AudibleAlert.warningImmediate:
          elapsed = time.monotonic() - self.ramp_start_time
          ramp_vol = float(np.interp(elapsed, [0, ALERT_RAMP_TIME], [self.ramp_start_volume, MAX_VOLUME]))
          self.current_volume = max(self.current_volume, ramp_vol)

        rk.keep_time()

        assert stream.active


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
