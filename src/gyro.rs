use embedded_hal_async::{delay::DelayNs, i2c::*};
use micromath::vector::Vector3d;

use super::{Bmi323, Error, defs::*};

impl<I, D, W, E> Bmi323<I, D, W>
where
  I: I2c<SevenBitAddress, Error = E>,
  D: DelayNs,
{
  pub async fn get_gyro_conf(&mut self) -> Result<GyroConfig, Error<E>> {
    self.read(Reg::GyrConf).await
  }

  pub async fn set_gyro_conf(&mut self, cfg: GyroConfig) -> Result<(), Error<E>> {
    self.write(Reg::GyrConf, cfg).await?;
    self.wait_for(crate::Sensor::Gyro).await
  }

  pub async fn get_raw_gyro_data(&mut self) -> Result<Vector3d<i16>, Error<E>> {
    let xyz: crate::XYZ = self.read(Reg::GyrDataX).await?;
    Ok(Vector3d { x: xyz.x, y: xyz.y, z: xyz.z })
  }

  /// Get the gyroscope data in degrees per second (dps)
  pub async fn get_gyro_data(&mut self) -> Result<Vector3d<f32>, Error<E>> {
    let accel_data = self.get_raw_gyro_data().await?;
    let range = self.get_gyro_conf().await?.range.multiplier();

    Ok(Vector3d { x: accel_data.x as f32 * range, y: accel_data.y as f32 * range, z: accel_data.z as f32 * range })
  }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[packbits::pack(bytes = 2)]
pub struct GyroConfig {
  /// Output Data Rate (Hz).
  #[bits(4)]
  pub odr: crate::OutputDataRate,
  /// Full‑scale range (degrees per second).
  #[bits(3)]
  pub range: GyroRange,
  #[bits(1)]
  pub bw: crate::Bandwidth,
  /// Number of samples averaged by on‑chip filter.
  #[bits(3)]
  pub avg: crate::AverageNum,
  #[skip(1)]
  #[bits(3)]
  pub mode: GyroPowerMode,
}

impl Default for GyroConfig {
  fn default() -> Self {
    GyroConfig {
      odr: crate::OutputDataRate::Hz100,
      range: GyroRange::DPS2000,
      bw: crate::Bandwidth::OdrQuarter,
      avg: crate::AverageNum::No,
      mode: GyroPowerMode::Normal,
    }
  }
}

/// Gyroscope measurement ranges
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroRange {
  /// ±125 degrees per second
  DPS125 = 0,
  /// ±250 degrees per second
  DPS250 = 1,
  /// ±500 degrees per second
  DPS500 = 2,
  /// ±1000 degrees per second
  DPS1000 = 3,
  /// ±2000 degrees per second
  DPS2000 = 4,
}

impl GyroRange {
  pub fn dps(&self) -> f32 {
    match self {
      GyroRange::DPS125 => 125.0,
      GyroRange::DPS250 => 250.0,
      GyroRange::DPS500 => 500.0,
      GyroRange::DPS1000 => 1000.0,
      GyroRange::DPS2000 => 2000.0,
    }
  }

  pub fn multiplier(&self) -> f32 {
    self.dps() / f32::from(i16::MAX)
  }
}

impl From<GyroRange> for u8 {
  fn from(value: GyroRange) -> Self {
    value as u8
  }
}

impl TryFrom<u8> for GyroRange {
  type Error = ();

  fn try_from(value: u8) -> Result<Self, Self::Error> {
    match value {
      0 => Ok(GyroRange::DPS125),
      1 => Ok(GyroRange::DPS250),
      2 => Ok(GyroRange::DPS500),
      3 => Ok(GyroRange::DPS1000),
      4 => Ok(GyroRange::DPS2000),
      _ => Err(()),
    }
  }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroPowerMode {
  /// Gyroscope disabled
  Disable = 0x00,
  /// Supend mode
  Suspend = 0x01,
  /// Low power mode
  LowPower = 0x03,
  /// Normal power mode
  Normal = 0x04,
  /// High perfomance mode
  HighPerf = 0x07,
}

impl From<GyroPowerMode> for u8 {
  fn from(value: GyroPowerMode) -> Self {
    value as u8
  }
}

impl TryFrom<u8> for GyroPowerMode {
  type Error = ();

  fn try_from(value: u8) -> Result<Self, Self::Error> {
    match value {
      0x00 => Ok(GyroPowerMode::Disable),
      0x01 => Ok(GyroPowerMode::Suspend),
      0x03 => Ok(GyroPowerMode::LowPower),
      0x04 => Ok(GyroPowerMode::Normal),
      0x07 => Ok(GyroPowerMode::HighPerf),
      _ => Err(()),
    }
  }
}
