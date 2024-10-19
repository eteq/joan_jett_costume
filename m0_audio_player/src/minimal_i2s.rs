#![no_std]
#![no_main]

use panic_halt as _;

use bsp::{hal, pac, entry, pin_alias};
use feather_m0 as bsp;

use hal::prelude::*;

// good register values for i2s
// reg CTRLA : 0b10110
// reg SERCTRL0 : 0b1000000000100010110010101
// reg CLKCTRL0 : 0b100000000000000000000010100101
// reg SERCTRL1 : 0b0
// reg CLKCTRL1 : 0b0
// reg SYNCBUSY : 0b100000000

#[entry]
fn main() -> ! {

    let mut peripherals = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut clocks = hal::clock::GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let _gclk0 = clocks.gclk0();
    let pm = peripherals.PM;
    let pins = bsp::Pins::new(peripherals.PORT);

    let mut status_led: bsp::RedLed = pin_alias!(pins.red_led).into();
    status_led.set_low().unwrap(); // in case it was stuck on for some reason
    let mut delay = hal::delay::Delay::new(core.SYST, &mut clocks);
    status_led.set_high().unwrap(); // in case it was stuck on for some reason

    let _i2s_fs = pins.d0.into_alternate::<hal::gpio::G>();
    let _i2s_sck = pins.d1.into_alternate::<hal::gpio::G>();
    let _i2s_sd = pins.d12.into_alternate::<hal::gpio::G>();

    // turn on i2s bus clock
    pm.apbcmask.modify(|_, w| {w.i2s_().set_bit()});

    // first set the sample rate and relevant parameters to work out the clock frequency

    let sample_rate: u32 = 22_050;
    let bits_per_sample: u32 = 16;
    let nchannels: u32 = 1;
    let fsck = sample_rate * bits_per_sample * nchannels;
    let div: u16 = (48_000_000 / fsck).try_into().expect("audio clock divisor doesnt fit in 16 bits");


    // configure a clock generator for the needed frequency and connect it to the i2s clocks
    let i2s_gclk = clocks.configure_gclk_divider_and_source(
                hal::clock::ClockGenId::GCLK3,
                div, 
                hal::clock::ClockSource::DFLL48M, 
                true).expect("the gclk for i2s is already configured!");
    let _i2s_clock0 = clocks.i2s0(&i2s_gclk);
    let _i2s_clock1 = clocks.i2s1(&i2s_gclk);  // we aren't neccessarly using this but just in case...

    status_led.set_high().unwrap();

    // now configure the i2s registers and enable the clock but not the serializer
    // first do a reset just in case
    peripherals.I2S.ctrla.write(|w| w.swrst().set_bit());
    // wait for reset to complete
    while peripherals.I2S.syncbusy.read().swrst().bit_is_set() {}
    while peripherals.I2S.ctrla.read().swrst().bit_is_set() {}

    // set up the clock unit
    unsafe {
    peripherals.I2S.clkctrl[0].write(|w| {
        w.slotsize()._16()
         .fswidth().half()
         .fsoutinv().set_bit()
         .nbslots().bits(1)
         .bitdelay().set_bit()  // not entirely sure about this one...
    });}
    // and the serializer
    peripherals.I2S.serctrl[0].write(|w| {
        w.mono().set_bit()
            //.datasize()._16()
            .datasize()._16c()
            .clksel().clk0()
            .txsame().set_bit()
            //.txdefault().hiz()
            .txdefault().one()
            .sermode().tx()
            .extend().msbit()
            .slotadj().left()
    });
    // wait for sync to complete
    while peripherals.I2S.syncbusy.read().bits() != 0 {}

    // and set up the i2s interrupt for txrdy0
    peripherals.I2S.intflag.write(|w| w.txrdy0().set_bit()); // it really should be cleared but you never know...
    peripherals.I2S.intenset.write(|w| w.txrdy0().set_bit());
    unsafe {
        core.NVIC.set_priority(pac::interrupt::I2S, 2);
        pac::NVIC::unmask(pac::interrupt::I2S);
    }

    // now we enable the peripheral and wait on sync but don't start the clock or serializer - wait on that for the song start
    peripherals.I2S.ctrla.modify(|_,w| w.enable().set_bit());
    while peripherals.I2S.syncbusy.read().bits() != 0 {}

    // send off the i2s peripheral to the global 
    // cortex_m::interrupt::free(|cs| {
    //     I2S_PERIPHERAL.borrow(cs).replace(Some(peripherals.I2S));
    // });

    let i2s = peripherals.I2S;

    // add the first sample to the i2s buffer, all 0s
    i2s.data[0].write(|w| unsafe { w.data().bits(0) });
    
    // enable the i2s clock0, ser0, and wait for sync in between

    i2s.ctrla.modify(|_,w| { w.cken0().set_bit() });
    while i2s.syncbusy.read().cken0().bit_is_set() {}
    i2s.ctrla.modify(|_,w| { w.seren0().set_bit() });
    while i2s.syncbusy.read().seren0().bit_is_set() {}

    loop {
        if i2s.intflag.read().txrdy0().bit_is_set() {
            // write a repeating quasi-toggle pattern to the i2s data register
            i2s.data[0].write(|w| unsafe { w.data().bits(0xafafafaf) });
        }
    }
}