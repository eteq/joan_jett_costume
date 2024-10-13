#![no_std]
#![no_main]

use cortex_m::asm;

use panic_persist;

use bsp::hal;
use bsp::pac;
use feather_m0::{self as bsp, ehal::digital::StatefulOutputPin, hal::{embedded_io::Write, gpio::{Output, PushPull}}};

use bsp::{entry, periph_alias, pin_alias};
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use pac::{CorePeripherals, Peripherals};

//use hal::embedded_io::*;

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

    // setup  status led
    let mut status_led: bsp::RedLed = pin_alias!(pins.red_led).into();
    status_led.set_low().unwrap(); // in case it was stuck on for some reason
    let mut delay = Delay::new(core.SYST, &mut clocks);

    // Take peripheral and pins
    let uart_sercom = periph_alias!(peripherals.uart_sercom);
    let uart_rx_pin = pin_alias!(pins.uart_rx);
    let uart_tx_pin = pin_alias!(pins.uart_tx);

    // Setup UART peripheral
    let mut uart = bsp::uart(
        &mut clocks,
        9600.Hz(),
        uart_sercom,
        &mut pm,
        uart_rx_pin,
        uart_tx_pin,
    );

    //let (mut uart_rx, mut uart_tx) = uart.split();
    // write any panic message - do this first before any other setup
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        uart.write_all(msg).expect("Could not write to uart!!");

        status_led.set_high().unwrap();
        asm::wfi(); // just print the message, means a reset is needed
    }



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