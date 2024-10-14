#![no_std]
#![no_main]

//use core::fmt::Debug;

use cortex_m::asm;

use embedded_sdmmc;
use panic_persist;

use bsp::hal;
use bsp::pac;
use feather_m0::{self as bsp, ehal::digital::StatefulOutputPin, hal::{embedded_io::Write, gpio::{Output, PushPull}}};

use bsp::{entry, periph_alias, pin_alias};
use hal::clock::{ClockGenId, ClockSource, GenericClockController};
use hal::delay::Delay;
use hal::prelude::*;
use pac::{CorePeripherals, Peripherals};

//use hal::embedded_io::*;

const VOL_IDX: usize = 0;
const N_SONGS: usize = 1;

#[entry]
fn main() -> ! {

    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
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

    // we need the internal 32k running at 1024 Hz for the RTC which is needed by the sd controller
    let timer_clock = clocks.configure_gclk_divider_and_source(ClockGenId::GCLK3, 32, ClockSource::XOSC32K, true).unwrap();
    let rtc_clock = clocks.rtc(&timer_clock).unwrap();
    let timer = hal::rtc::Rtc::clock_mode(peripherals.RTC, rtc_clock.freq(), &mut pm);

    // now set up SPI in low-speed mode
    let spi_sercom = periph_alias!(peripherals.spi_sercom);
    let spi = bsp::spi_master(
        &mut clocks,
        400_u32.kHz(),
        spi_sercom,
        &mut pm,
        pins.sclk,
        pins.mosi,
        pins.miso,
    );
    let mut sd_cs: bsp::SdCs = pins.sd_cs.into();
    sd_cs.set_high().unwrap();


    let mut sdcontroller = embedded_sdmmc::Controller::new(embedded_sdmmc::SdMmcSpi::new(spi, sd_cs), timer);
    match sdcontroller.device().init() {
        Ok(_) => {
            // start up the sd card

            // first speed it up to a reasonable speed
            sdcontroller
                .device()
                .spi()
                .reconfigure(|c| c.set_baud(4.MHz()));

            // now dump some info

            let sdsize = sdcontroller.device().card_size_bytes().unwrap();
            uart.write_fmt(format_args!("Sd card size is {sdsize}\r\n")).expect("Could not write to uart!!");

            let mut sdvol =  match sdcontroller.get_volume(embedded_sdmmc::VolumeIdx(VOL_IDX)) {
                Ok(v) => v,
                Err(_) => panic!("could not get sd volume")
            };
            let root_dir = sdcontroller.open_root_dir(&sdvol).unwrap();

            uart.write_all("starting read of first file\r\n".as_bytes()).expect("Could not write to uart!!");
            let mut buf = [0u8; 1024];
            let mut file0 = match sdcontroller.open_file_in_dir(&mut sdvol, &root_dir, "JJ0.WAV", embedded_sdmmc::Mode::ReadOnly) {
                Ok(f) => f,
                Err(_) => panic!("could not open first file")
            };
            let mut nread = 0;  
            while !file0.eof() { 
                nread += sdcontroller.read(&sdvol, &mut file0, &mut buf).unwrap();
            }

            uart.write_fmt(format_args!("finished read of first file, {nread} bytes\r\n")).expect("Could not write to uart!!");


            uart.write_all("Sd card startup completed.".as_bytes()).expect("Could not write to uart!!");
        }
        Err(e) => {
            uart.write_fmt(format_args!("Sd card init error: {e:?}")).expect("Could not write to uart!!");
            panic!("SD card startup failed");
        }
    }



    // Completed setup! entering mainloop
    uart.write_all("started!\n\r".as_bytes()).expect("Could not write start to uart!!");

    loop {
        blink_led(&mut status_led, &mut delay, 5, 100);
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