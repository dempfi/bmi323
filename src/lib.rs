#![no_std]
//! Async, no-std BMI323 driver for I2C.
//!
//! Design goals:
//! - Small, typed helpers for common data paths and features
//! - Raw register/feature fields exposed where the datasheet uses non‑linear encodings
//!   (docs describe the raw→units mapping)
//! - Uses `embedded-hal-async` I2C traits

use embedded_hal_async::{delay::DelayNs, i2c::*};

pub mod accel;
pub mod alt;
pub mod calib;
mod defs;
#[cfg(feature = "events")]
mod events;
mod feature;
pub mod fifo;
pub mod gyro;
pub mod interrupt;
pub mod io;
pub mod offset;
pub(crate) mod rw;
pub mod selftest;
mod types;

use defs::*;
#[cfg(feature = "events")]
pub use events::*;
pub use feature::*;
pub use types::*;

/// Driver error type.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
  I2c(E),
  InvalidChipId(u8),
  InvalidMode,
  Init,
  Data,
}

/// BMI323 device instance (I2C + delay provider).
///
/// The third generic `W` is used only when the `events` feature is enabled. When
/// disabled, it is carried as a PhantomData to keep the type arity stable.
pub struct Bmi323<I, D: DelayNs, W = ()> {
  i2c: I,
  delay: D,
  #[cfg(feature = "events")]
  dequeue: heapless::Deque<Event, 16>,
  #[cfg(feature = "events")]
  int_pin: W,
  #[cfg(not(feature = "events"))]
  _wait: core::marker::PhantomData<W>,
}

// Constructor(s)
#[cfg(feature = "events")]
impl<I, D, W> Bmi323<I, D, W>
where
  I: I2c<SevenBitAddress>,
  D: DelayNs,
  W: embedded_hal_async::digital::Wait,
{
  pub fn new(i2c: I, delay: D, int_pin: W) -> Self {
    Self { i2c, delay, dequeue: heapless::Deque::new(), int_pin }
  }
}

#[cfg(not(feature = "events"))]
impl<I, D, W> Bmi323<I, D, W>
where
  I: I2c<SevenBitAddress>,
  D: DelayNs,
{
  pub fn new(i2c: I, delay: D) -> Self {
    Self { i2c, delay, _wait: core::marker::PhantomData }
  }
}

// Common functionality (independent of `events`)
impl<I, D, W, E> Bmi323<I, D, W>
where
  I: I2c<SevenBitAddress, Error = E>,
  D: DelayNs,
{
  /// Read `CHIP_ID` (0x00).
  pub async fn get_id(&mut self) -> Result<u8, Error<E>> {
    let r: ChipId = self.read(Reg::ChipId).await?;
    Ok(r.id)
  }

  pub async fn soft_reset(&mut self) -> Result<(), Error<E>> {
    self.write_u16(Reg::Cmd, Command::SoftReset.into()).await?;
    self.delay.delay_ms(SOFT_RESET_DELAY as u32).await;
    Ok(())
  }

  /// Read `ERR_REG` (raw bits per datasheet).
  pub async fn get_error(&mut self) -> Result<u16, Error<E>> {
    self.read_u16(Reg::ErrReg).await
  }

  /// Wait until the selected sensor sets its data-ready bit.
  async fn wait_for(&mut self, sensor: Sensor) -> Result<(), Error<E>> {
    let mut retries = 0;
    while !self.is_ready(sensor).await? {
      if retries > 20 {
        return Err(Error::Data);
      }
      self.delay.delay_ms(2).await;
      retries += 1;
    }

    Ok(())
  }

  /// Check data-ready status for accel/gyro/temp using `STATUS` bits.
  async fn is_ready(&mut self, sensor: Sensor) -> Result<bool, Error<E>> {
    let st: Status = self.read(Reg::Status).await?;
    Ok(match sensor {
      Sensor::Accel => st.drdy_acc,
      Sensor::Gyro => st.drdy_gyr,
      Sensor::Temp => st.drdy_temp,
    })
  }

  // -----------------
  // Misc. data reads
  // -----------------
  /// Read raw temperature register (device-specific units).
  pub async fn get_temperature_raw(&mut self) -> Result<i16, Error<E>> {
    // TempData is 16-bit; convert to i16 LE.
    let raw: u16 = self.read_u16(Reg::TempData).await?;
    Ok(raw as i16)
  }

  /// Read 24-bit sensor time counter (wraps at 2^24). Units per datasheet.
  pub async fn get_sensor_time(&mut self) -> Result<u32, Error<E>> {
    let mut b = [0u8; 3];
    // SensorTime0..2 are consecutive starting at SensorTime0.
    self.read_bytes(Reg::SensorTime0, &mut b).await?;
    Ok((b[2] as u32) << 16 | (b[1] as u32) << 8 | (b[0] as u32))
  }

  /// Read saturation flags for accel/gyro axes.
  pub async fn get_saturation_flags(&mut self) -> Result<SaturationFlags, Error<E>> {
    self.read(Reg::SatFlags).await
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[packbits::pack(bytes = 1)]
struct Status {
  #[skip(5)]
  pub drdy_temp: bool,
  pub drdy_gyr: bool,
  pub drdy_acc: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[packbits::pack(bytes = 1)]
struct ChipId {
  pub id: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[packbits::pack(bytes = 2)]
pub struct SaturationFlags {
  pub acc_x: bool,
  pub acc_y: bool,
  pub acc_z: bool,
  pub gyr_x: bool,
  pub gyr_y: bool,
  pub gyr_z: bool,
}
