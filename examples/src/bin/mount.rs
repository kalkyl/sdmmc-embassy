// $ DEFMT_LOG=info cargo rb mount
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use nrf_embassy as _; // global logger + panicking-behavior + memory layout

use defmt::*;
use embassy::executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{interrupt, spim, Peripherals};
use sdmmc_embassy::{TimeSource, Timestamp, VolumeIdx};

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M2;
    let irq = interrupt::take!(SPIM3);
    let sdmmc_cs = Output::new(p.P0_26, Level::High, OutputDrive::Standard);
    let spim = spim::Spim::new(p.SPI3, irq, p.P0_08, p.P0_06, p.P0_04, config);

    let mut sdmmc_spi = sdmmc_embassy::SdMmcSpi::new(spim, sdmmc_cs);
    sdmmc_spi.init().await.unwrap();

    let mut sdmmc_controller = sdmmc_embassy::Controller::new(sdmmc_spi, SdMmcClock);

    match sdmmc_controller.device().card_size_bytes().await {
        Ok(size) => info!("Card size {}", size),
        _ => error!("Error getting card size"),
    }

    match sdmmc_controller.get_volume(VolumeIdx(0)).await {
        Ok(volume) => {
            let root_dir = sdmmc_controller.open_root_dir(&volume).unwrap();
            info!("Listing root directory:");
            sdmmc_controller
                .iterate_dir(&volume, &root_dir, |x| {
                    info!("{}", core::str::from_utf8(x.name.base_name()).unwrap());
                })
                .await
                .unwrap();
            info!("End of listing");
        }
        _ => error!("Error listing directory"),
    }

    loop {}
}

pub struct SdMmcClock;

impl TimeSource for SdMmcClock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
