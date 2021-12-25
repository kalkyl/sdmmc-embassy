//! embedded-sdmmc-rs - SDMMC Protocol
//!
//! Implements the SD/MMC protocol on some generic SPI interface.
//!
//! This is currently optimised for readability and debugability, not
//! performance.

use super::sdmmc_proto::*;
use super::{Block, BlockCount, BlockIdx};
use core::cell::RefCell;

const DEFAULT_DELAY_COUNT: u32 = 32_000;

/// Represents an inactive SD Card interface.
/// Built from an SPI peripheral and a Chip
/// Select pin. We need Chip Select to be separate so we can clock out some
/// bytes without Chip Select asserted (which puts the card into SPI mode).
pub struct SdMmcSpi<SPI, CS>
where
    SPI: embassy_traits::spi::FullDuplex<u8> + embassy_traits::spi::Spi<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embassy_traits::spi::Spi<u8>>::Error: core::fmt::Debug,
{
    spi: RefCell<SPI>,
    cs: RefCell<CS>,
    card_type: CardType,
    state: State,
}

/// The possible errors `SdMmcSpi` can generate.
#[derive(Debug, Copy, Clone)]
pub enum Error {
    /// We got an error from the SPI peripheral
    Transport,
    /// We failed to enable CRC checking on the SD card
    CantEnableCRC,
    /// We didn't get a response when reading data from the card
    TimeoutReadBuffer,
    /// We didn't get a response when waiting for the card to not be busy
    TimeoutWaitNotBusy,
    /// We didn't get a response when executing this command
    TimeoutCommand(u8),
    /// We didn't get a response when executing this application-specific command
    TimeoutACommand(u8),
    /// We got a bad response from Command 58
    Cmd58Error,
    /// We failed to read the Card Specific Data register
    RegisterReadError,
    /// We got a CRC mismatch (card gave us, we calculated)
    CrcError(u16, u16),
    /// Error reading from the card
    ReadError,
    /// Error writing to the card
    WriteError,
    /// Can't perform this operation with the card in this state
    BadState,
    /// Couldn't find the card
    CardNotFound,
    /// Couldn't set a GPIO pin
    GpioError,
}

/// The possible states `SdMmcSpi` can be in.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum State {
    /// Card is not initialised
    NoInit,
    /// Card is in an error state
    Error,
    /// Card is initialised and idle
    Idle,
}

/// The different types of card we support.
#[derive(Debug, Copy, Clone, PartialEq)]
enum CardType {
    SD1,
    SD2,
    SDHC,
}

/// A terrible hack for busy-waiting the CPU while we wait for the card to
/// sort itself out.
///
/// @TODO replace this!
struct Delay(u32);

impl Delay {
    fn new() -> Self {
        Delay(DEFAULT_DELAY_COUNT)
    }

    async fn delay<D>(&mut self, delay: &mut D, micros: u64, err: Error) -> Result<(), Error>
    where
        D: embassy_traits::delay::Delay,
    {
        if self.0 == 0 {
            Err(err)
        } else {
            delay.delay_us(micros).await;
            self.0 -= 1;
            Ok(())
        }
    }

    // fn delay(&mut self, err: Error) -> Result<(), Error> {
    //     if self.0 == 0 {
    //         Err(err)
    //     } else {
    //         let dummy_var: u32 = 0;
    //         for _ in 0..100 {
    //             unsafe { core::ptr::read_volatile(&dummy_var) };
    //         }
    //         self.0 -= 1;
    //         Ok(())
    //     }
    // }
}

/// Options for acquiring the card.
#[derive(Debug)]
pub struct AcquireOpts {
    /// Some cards don't support CRC mode. At least a 512MiB Transcend one.
    pub require_crc: bool,
}

impl Default for AcquireOpts {
    fn default() -> Self {
        AcquireOpts { require_crc: true }
    }
}

impl<'a, SPI, CS> SdMmcSpi<SPI, CS>
where
    SPI: embassy_traits::spi::FullDuplex<u8> + embassy_traits::spi::Spi<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embassy_traits::spi::Spi<u8>>::Error: core::fmt::Debug,
{
    /// Create a new SD/MMC controller using a raw SPI interface.
    pub fn new(spi: SPI, cs: CS) -> SdMmcSpi<SPI, CS> {
        SdMmcSpi {
            spi: RefCell::new(spi),
            cs: RefCell::new(cs),
            card_type: CardType::SD1,
            state: State::NoInit,
        }
    }

    fn cs_high(&self) -> Result<(), Error> {
        self.cs
            .borrow_mut()
            .set_high()
            .map_err(|_| Error::GpioError)
    }

    fn cs_low(&self) -> Result<(), Error> {
        self.cs.borrow_mut().set_low().map_err(|_| Error::GpioError)
    }

    /// Initializes the card into a known state
    pub async fn acquire<D>(self, delay: &mut D) -> Result<SdMmcSpi<SPI, CS>, Error>
    where
        D: embassy_traits::delay::Delay,
    {
        self.acquire_with_opts(delay, Default::default()).await
    }

    /// Initializes the card into a known state
    pub async fn acquire_with_opts<D>(
        mut self,
        delay: &mut D,
        options: AcquireOpts,
    ) -> Result<SdMmcSpi<SPI, CS>, Error>
    where
        D: embassy_traits::delay::Delay,
    {
        // Assume it hasn't worked
        self.state = State::Error;
        // Supply minimum of 74 clock cycles without CS asserted.
        self.cs_high()?;
        for _ in 0..10 {
            self.send(0xFF).await?;
        }
        // Assert CS
        self.cs_low()?;
        // Enter SPI mode
        let mut attempts = 32;
        while attempts > 0 {
            match self.card_command(delay, CMD0, 0).await {
                Err(Error::TimeoutCommand(0)) => {
                    // Try again?
                    attempts -= 1;
                }
                Err(e) => {
                    return Err(e);
                }
                Ok(R1_IDLE_STATE) => {
                    break;
                }
                Ok(_) => {
                    // Try again
                }
            }
        }
        if attempts == 0 {
            return Err(Error::CardNotFound);
        }
        // Enable CRC
        if self.card_command(delay, CMD59, 1).await? != R1_IDLE_STATE && options.require_crc {
            return Err(Error::CantEnableCRC);
        }
        // Check card version
        let mut timeout = Delay::new();
        loop {
            if self.card_command(delay, CMD8, 0x1AA).await? == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE)
            {
                self.card_type = CardType::SD1;
                break;
            }
            self.receive().await?;
            self.receive().await?;
            self.receive().await?;
            let status = self.receive().await?;
            if status == 0xAA {
                self.card_type = CardType::SD2;
                break;
            }
            timeout
                .delay(delay, 10, Error::TimeoutCommand(CMD8))
                .await?;
        }

        let arg = match self.card_type {
            CardType::SD1 => 0,
            CardType::SD2 | CardType::SDHC => 0x4000_0000,
        };

        let mut timeout = Delay::new();
        while self.card_acmd(delay, ACMD41, arg).await? != R1_READY_STATE {
            timeout
                .delay(delay, 10, Error::TimeoutACommand(ACMD41))
                .await?;
        }

        if self.card_type == CardType::SD2 {
            if self.card_command(delay, CMD58, 0).await? != 0 {
                return Err(Error::Cmd58Error);
            }
            if (self.receive().await? & 0xC0) == 0xC0 {
                self.card_type = CardType::SDHC;
            }
            // Discard other three bytes
            self.receive().await?;
            self.receive().await?;
            self.receive().await?;
        }
        self.state = State::Idle;
        // Ok(())

        // let result = f(self).await;
        self.cs_high()?;
        let _ = self.receive();
        Ok(self)
    }

    /// Perform an application-specific command.
    async fn card_acmd<D: embassy_traits::delay::Delay>(
        &self,
        delay: &mut D,
        command: u8,
        arg: u32,
    ) -> Result<u8, Error> {
        self.card_command(delay, CMD55, 0).await?;
        self.card_command(delay, command, arg).await
    }

    /// Perform a command.
    async fn card_command<D: embassy_traits::delay::Delay>(
        &self,
        delay: &mut D,
        command: u8,
        arg: u32,
    ) -> Result<u8, Error> {
        self.wait_not_busy(delay).await?;
        let mut buf = [
            0x40 | command,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        for b in buf.iter() {
            self.send(*b).await?;
        }

        // skip stuff byte for stop read
        if command == CMD12 {
            let _result = self.receive().await?;
        }

        for _ in 0..512 {
            let result = self.receive().await?;
            if (result & 0x80) == ERROR_OK {
                return Ok(result);
            }
        }

        Err(Error::TimeoutCommand(command))
    }

    /// Receive a byte from the SD card by clocking in an 0xFF byte.
    async fn receive(&self) -> Result<u8, Error> {
        self.transfer(0xFF).await
    }

    /// Send a byte from the SD card.
    async fn send(&self, out: u8) -> Result<(), Error> {
        let _ = self.transfer(out).await?;
        Ok(())
    }

    /// Send one byte and receive one byte.
    async fn transfer(&self, out: u8) -> Result<u8, Error> {
        let mut spi = self.spi.borrow_mut();
        let mut buf = [0_u8];
        spi.read_write(&mut buf, &[out])
            .await
            .map_err(|_e| Error::Transport)?;
        Ok(buf[0])
    }

    /// Spin until the card returns 0xFF, or we spin too many times and
    /// timeout.
    async fn wait_not_busy<D>(&self, delay: &mut D) -> Result<(), Error>
    where
        D: embassy_traits::delay::Delay,
    {
        let mut timeout = Delay::new();
        loop {
            let s = self.receive().await?;
            if s == 0xFF {
                break;
            }
            timeout.delay(delay, 10, Error::TimeoutWaitNotBusy).await?;
        }
        Ok(())
    }

    /// Get a temporary borrow on the underlying SPI device. Useful if you
    /// need to re-clock the SPI.
    pub fn spi(&mut self) -> core::cell::RefMut<SPI> {
        self.spi.borrow_mut()
    }

    /// Mark the card as unused.
    /// This should be kept infallible, because Drop is unable to fail.
    /// See https://github.com/rust-lang/rfcs/issues/814
    // If there is any need to flush data, it should be implemented here.
    fn deinit(&mut self) {
        self.state = State::NoInit;
    }

    /// Return the usable size of this SD card in bytes.
    pub async fn card_size_bytes<D: embassy_traits::delay::Delay>(
        &self,
        delay: &mut D,
    ) -> Result<u64, Error> {
        self.cs_low()?;
        let csd = self.read_csd(delay).await?;
        let result = match csd {
            Csd::V1(ref contents) => Ok(contents.card_capacity_bytes()),
            Csd::V2(ref contents) => Ok(contents.card_capacity_bytes()),
        };
        self.cs_high()?;
        result
    }

    /// Erase some blocks on the card.
    pub fn erase(&mut self, _first_block: BlockIdx, _last_block: BlockIdx) -> Result<(), Error> {
        unimplemented!();
    }

    /// Can this card erase single blocks?
    pub async fn erase_single_block_enabled<D: embassy_traits::delay::Delay>(
        &self,
        delay: &mut D,
    ) -> Result<bool, Error> {
        self.cs_low()?;
        let csd = self.read_csd(delay).await?;
        let result = match csd {
            Csd::V1(ref contents) => Ok(contents.erase_single_block_enabled()),
            Csd::V2(ref contents) => Ok(contents.erase_single_block_enabled()),
        };
        self.cs_high()?;
        result
    }

    /// Read the 'card specific data' block.
    async fn read_csd<D: embassy_traits::delay::Delay>(&self, delay: &mut D) -> Result<Csd, Error> {
        match self.card_type {
            CardType::SD1 => {
                let mut csd = CsdV1::new();
                if self.card_command(delay, CMD9, 0).await? != 0 {
                    return Err(Error::RegisterReadError);
                }
                self.read_data(delay, &mut csd.data).await?;
                Ok(Csd::V1(csd))
            }
            CardType::SD2 | CardType::SDHC => {
                let mut csd = CsdV2::new();
                if self.card_command(delay, CMD9, 0).await? != 0 {
                    return Err(Error::RegisterReadError);
                }
                self.read_data(delay, &mut csd.data).await?;
                Ok(Csd::V2(csd))
            }
        }
    }

    /// Read an arbitrary number of bytes from the card. Always fills the
    /// given buffer, so make sure it's the right size.
    async fn read_data<D>(&self, delay: &mut D, buffer: &mut [u8]) -> Result<(), Error>
    where
        D: embassy_traits::delay::Delay,
    {
        // Get first non-FF byte.
        let mut timeout = Delay::new();
        let status = loop {
            let s = self.receive().await?;
            if s != 0xFF {
                break s;
            }
            timeout.delay(delay, 10, Error::TimeoutReadBuffer).await?;
        };
        if status != DATA_START_BLOCK {
            return Err(Error::ReadError);
        }

        for b in buffer.iter_mut() {
            *b = self.receive().await?;
        }

        let mut crc = u16::from(self.receive().await?);
        crc <<= 8;
        crc |= u16::from(self.receive().await?);

        let calc_crc = crc16(buffer);
        if crc != calc_crc {
            return Err(Error::CrcError(crc, calc_crc));
        }

        Ok(())
    }

    /// Write an arbitrary number of bytes to the card.
    async fn write_data(&self, token: u8, buffer: &[u8]) -> Result<(), Error> {
        let calc_crc = crc16(buffer);
        self.send(token).await?;
        for &b in buffer.iter() {
            self.send(b).await?;
        }
        self.send((calc_crc >> 8) as u8).await?;
        self.send(calc_crc as u8).await?;
        let status = self.receive().await?;
        if (status & DATA_RES_MASK) != DATA_RES_ACCEPTED {
            Err(Error::WriteError)
        } else {
            Ok(())
        }
    }

    /// Read one or more blocks, starting at the given block index.
    pub async fn read<D: embassy_traits::delay::Delay>(
        &self,
        delay: &mut D,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
        _reason: &str,
    ) -> Result<(), Error> {
        let start_idx = match self.card_type {
            CardType::SD1 | CardType::SD2 => start_block_idx.0 * 512,
            CardType::SDHC => start_block_idx.0,
        };
        self.cs_low()?;

        if blocks.len() == 1 {
            // Start a single-block read
            self.card_command(delay, CMD17, start_idx).await?;
            self.read_data(delay, &mut blocks[0].contents).await?;
        } else {
            // Start a multi-block read
            self.card_command(delay, CMD18, start_idx).await?;
            for block in blocks.iter_mut() {
                self.read_data(delay, &mut block.contents).await?;
            }
            // Stop the read
            self.card_command(delay, CMD12, 0).await?;
        }
        self.cs_high()?;
        Ok(())
    }

    /// Write one or more blocks, starting at the given block index.
    pub async fn write<D: embassy_traits::delay::Delay>(
        &self,
        delay: &mut D,
        blocks: &[Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), Error> {
        let start_idx = match self.card_type {
            CardType::SD1 | CardType::SD2 => start_block_idx.0 * 512,
            CardType::SDHC => start_block_idx.0,
        };
        self.cs_low()?;

        if blocks.len() == 1 {
            // Start a single-block write
            self.card_command(delay, CMD24, start_idx).await?;
            self.write_data(DATA_START_BLOCK, &blocks[0].contents)
                .await?;
            self.wait_not_busy(delay).await?;
            if self.card_command(delay, CMD13, 0).await? != 0x00 {
                return Err(Error::WriteError);
            }
            if self.receive().await? != 0x00 {
                return Err(Error::WriteError);
            }
        } else {
            // Start a multi-block write
            self.card_command(delay, CMD25, start_idx).await?;
            for block in blocks.iter() {
                self.wait_not_busy(delay).await?;
                self.write_data(WRITE_MULTIPLE_TOKEN, &block.contents)
                    .await?;
            }
            // Stop the write
            self.wait_not_busy(delay).await?;
            self.send(STOP_TRAN_TOKEN).await?;
        }

        self.cs_high()?;
        Ok(())
    }

    /// Determine how many blocks this device can hold.
    async fn num_blocks<D: embassy_traits::delay::Delay>(
        &self,
        delay: &mut D,
    ) -> Result<BlockCount, Error> {
        let num_bytes = self.card_size_bytes(delay).await?;
        let num_blocks = (num_bytes / 512) as u32;
        Ok(BlockCount(num_blocks))
    }

    fn drop(&mut self) {
        self.deinit()
    }
}
