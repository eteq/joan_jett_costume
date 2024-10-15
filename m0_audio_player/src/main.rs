#![no_std]
#![no_main]

use nb;
use cortex_m::asm;

use embedded_sdmmc;
use embedded_sdmmc::{VolumeIdx, VolumeManager, SdCard};
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_persist;

use bsp::hal;
use bsp::pac;
use feather_m0::{self as bsp, ehal::digital::StatefulOutputPin, hal::{embedded_io::Write, gpio::{Output, PushPull}}};

use bsp::{entry, periph_alias, pin_alias};
use hal::delay::Delay;
use hal::prelude::*;
use pac::{CorePeripherals, Peripherals};

use numtoa::NumToA;

const VOL_IDX: usize = 0;
const N_SONGS: usize = 6;
const SD_CARD_KHZ: u32 = 4000;

#[entry]
fn main() -> ! {

    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = hal::clock::GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut pm = peripherals.PM;
    let pins = bsp::Pins::new(peripherals.PORT);

    // setup status led
    let mut status_led: bsp::RedLed = pin_alias!(pins.red_led).into();
    status_led.set_low().unwrap(); // in case it was stuck on for some reason
    let mut delay = Delay::new(core.SYST, &mut clocks);

    // Setup UART peripheral and pins
    let uart_sercom = periph_alias!(peripherals.uart_sercom);
    let uart_rx_pin = pin_alias!(pins.uart_rx);
    let uart_tx_pin = pin_alias!(pins.uart_tx);
    let mut uart = bsp::uart(
        &mut clocks,
        9600.Hz(),
        uart_sercom,
        &mut pm,
        uart_rx_pin,
        uart_tx_pin,
    );

    // write any panic message - do this first before any other setup
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        uart.write_all("last start yielded the following panic:\n\r".as_bytes()).expect("Could not write panic to uart!!");
        uart.write_all(msg).expect("Could not write panic to uart!!");
        uart.write_all("\n\r".as_bytes()).expect("Could not write panic to uart!!");

        status_led.set_high().unwrap();
        asm::wfi(); // just print the message, means a reset is needed
    }


    // setup SD card in SPI mode

    // start by making sure a card is present
    let sd_cd: bsp::SdCd = pins.sd_cd.into();
    if sd_cd.is_low().unwrap() {
        uart.write_all("No SD card is present. Waiting for one to appear...\r\n".as_bytes()).expect("Could not write to uart!!");

        let mut i = 0;
        while sd_cd.is_low().unwrap() {
            blink_led(&mut status_led, &mut delay, 3, 100);
            status_led.set_high().unwrap();
            delay.delay_ms(1500u32);
            i += 1;
            if i > (5*60000)/1800 {
                panic!("Waited 5 min with no SD card, giving up and panicking.")
            }
        }
        uart.write_all("SD card inserted, continuing with startup.\r\n".as_bytes()).expect("Could not write to uart!!");
        status_led.set_low().unwrap();
        delay.delay_ms(250_u32); // just in case the card needs time to warm up or something
    } else {
        uart.write_all("SD card present, continuing with startup.\r\n".as_bytes()).expect("Could not write to uart!!");
    }

    // now set up SPI in low-speed mode
    let spi_sercom = periph_alias!(peripherals.spi_sercom);
    let mut sd_spi = bsp::spi_master(
        &mut clocks,
        400_u32.kHz(),
        spi_sercom,
        &mut pm,
        pins.sclk,
        pins.mosi,
        pins.miso,
    );
    let mut sd_cs: bsp::SdCs = pins.sd_cs.into();
    // do the weird SD init thing where you have to send >74 clocks with CS high/de-asserted
    sd_cs.set_high().unwrap();
    for _ in 0..10 { nb::block!(sd_spi.send(0xAA)).unwrap(); } // 10*8 bits > 74 clocks

    //speed up the SPI
    sd_spi.reconfigure(|c| c.set_baud(SD_CARD_KHZ.kHz()));

    // now start up the SD controller
    let sdmmc_spi = ExclusiveDevice::new_no_delay(sd_spi, sd_cs).expect("Failed to create SpiDevice");
    let card = SdCard::new(sdmmc_spi, delay);
    let mut volume_mgr: VolumeManager<_, _, 1, N_SONGS, 1> = VolumeManager::new_with_limits(card, FakeClock {}, 0);
    let mut sdvolume = volume_mgr
        .open_volume(VolumeIdx(VOL_IDX))
        .expect("Failed to open volume");

    // now search the root directory for songs that match the JJ#.WAV format
    let mut root_dir = sdvolume.open_root_dir().expect("Failed to open root dir");
    let mut song_filenames: [Option<embedded_sdmmc::ShortFileName> ; N_SONGS] = Default::default();
    root_dir.iterate_dir(|entry| {
        for i in 0..N_SONGS {
            let start = entry.name.base_name()[0..2] == *b"JJ";
            let end = entry.name.extension() == b"WAV";
            let buf = &mut [0u8; 1];
            NumToA::numtoa(i, 10, buf);
            let num = entry.name.base_name()[2..3] == *buf;
            if start && end && num {
                song_filenames[i] = Some(entry.name.clone());
                break;
            }
        }
    }).expect("Failed to iterate root dir");

    // Completed setup! entering mainloop
    uart.write_all("started!\n\r".as_bytes()).expect("Could not write start to uart!!");

    let stolencore = unsafe { CorePeripherals::steal() };
    let mut stolendelay = Delay::new(stolencore.SYST, &mut clocks);

    loop {
        blink_led(&mut status_led, &mut stolendelay, 5, 100);
        panic!("ahhh");
    }
}

fn blink_led<T: hal::gpio::PinId>(led: &mut hal::gpio::Pin<T, Output<PushPull>>, delay: &mut Delay,
                                  ntimes: u8, blinkms: u16) {
    let started_high = led.is_set_high().unwrap();
    if  started_high { led.set_low().expect("led setting failed!"); }
    for _ in 0..ntimes {
        delay.delay_ms(blinkms);
        led.set_high().expect("led setting failed!");
        delay.delay_ms(blinkms);
        led.set_low().expect("led setting failed!");
    }
    if started_high { led.set_high().expect("led setting failed!"); }
}


struct FakeClock;

impl embedded_sdmmc::TimeSource for FakeClock {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}