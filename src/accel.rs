use embedded_hal_async::{delay::DelayNs, i2c::*};
use micromath::vector::Vector3d;

use super::{Bmi323, Error, defs::*};

impl<I, D, W, E> Bmi323<I, D, W>
where
  I: I2c<SevenBitAddress, Error = E>,
  D: DelayNs,
{
  pub async fn get_accel_conf(&mut self) -> Result<AccelConfig, Error<E>> {
    self.read(Reg::AccConf).await
  }

  pub async fn set_accel_conf(&mut self, cfg: AccelConfig) -> Result<(), Error<E>> {
    self.write(Reg::AccConf, cfg).await?;
    self.wait_for(crate::Sensor::Accel).await
  }

  pub async fn get_raw_accel_data(&mut self) -> Result<Vector3d<i16>, Error<E>> {
    let xyz: crate::XYZ = self.read(Reg::AccDataX).await?;
    Ok(Vector3d { x: xyz.x, y: xyz.y, z: xyz.z })
  }

  /// Get the accelerometer data in g
  pub async fn get_accel_data(&mut self) -> Result<Vector3d<f32>, Error<E>> {
    let accel_data = self.get_raw_accel_data().await?;
    let range = self.get_accel_conf().await?.range.multiplier();

    Ok(Vector3d { x: accel_data.x as f32 * range, y: accel_data.y as f32 * range, z: accel_data.z as f32 * range })
  }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[packbits::pack(bytes = 2)]
pub struct AccelConfig {
  /// Output Data Rate (Hz). See `OutputDataRate` (≈0.78 Hz .. 6.4 kHz).
  #[bits(4)]
  pub odr: crate::OutputDataRate,
  /// Full‑scale range in g (±2/±4/±8/±16).
  #[bits(3)]
  pub range: AccelRange,
  /// Digital low‑pass cutoff relative to ODR (ODR/2 or ODR/4).
  #[bits(1)]
  pub bw: crate::Bandwidth,
  /// Number of samples averaged by on‑chip filter.
  #[bits(3)]
  pub avg: crate::AverageNum,
  #[skip(1)]
  #[bits(3)]
  pub mode: AccelPowerMode,
}

impl Default for AccelConfig {
  fn default() -> Self {
    AccelConfig {
      odr: crate::OutputDataRate::Hz1600,
      range: AccelRange::G2,
      bw: crate::Bandwidth::OdrHalf,
      avg: crate::AverageNum::No,
      mode: AccelPowerMode::Normal,
    }
  }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelRange {
  G2 = 0x00,
  G4 = 0x01,
  G8 = 0x02,
  G16 = 0x03,
}

impl AccelRange {
  pub(crate) fn multiplier(self) -> f32 {
    match self {
      AccelRange::G2 => 1. / 16384.,
      AccelRange::G4 => 1. / 8192.,
      AccelRange::G8 => 1. / 4096.,
      AccelRange::G16 => 1. / 2048.,
    }
  }
}

impl From<AccelRange> for u8 {
  fn from(value: AccelRange) -> Self {
    value as u8
  }
}

impl TryFrom<u8> for AccelRange {
  type Error = ();

  fn try_from(value: u8) -> Result<Self, Self::Error> {
    match value {
      0x00 => Ok(AccelRange::G2),
      0x01 => Ok(AccelRange::G4),
      0x02 => Ok(AccelRange::G8),
      0x03 => Ok(AccelRange::G16),
      _ => Err(()),
    }
  }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelPowerMode {
  Disable = 0x0,
  LowPower = 0x3,
  Normal = 0x4,
  HighPerf = 0x7,
}

impl From<AccelPowerMode> for u8 {
  fn from(value: AccelPowerMode) -> Self {
    value as u8
  }
}

impl TryFrom<u8> for AccelPowerMode {
  type Error = ();

  fn try_from(value: u8) -> Result<Self, Self::Error> {
    match value {
      0x0 => Ok(AccelPowerMode::Disable),
      0x3 => Ok(AccelPowerMode::LowPower),
      0x4 => Ok(AccelPowerMode::Normal),
      0x7 => Ok(AccelPowerMode::HighPerf),
      _ => Err(()),
    }
  }
}
