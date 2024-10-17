#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

use embedded_sdmmc::{self, BlockDevice, ShortFileName, TimeSource, VolumeIdx, VolumeManager, SdCard, File};
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_persist;

use bsp::{hal, pac, entry, periph_alias, pin_alias};

use feather_m0::{self as bsp, ehal::digital::StatefulOutputPin, hal::{embedded_io::Write, gpio, gpio::{Output, PushPull}}};

use hal::delay::Delay;
use hal::prelude::*;
use hal::sercom::uart;
use pac::{CorePeripherals, Peripherals, interrupt};

use nb;

use numtoa::NumToA;
use arraydeque::ArrayDeque;

const SD_CARD_KHZ: u32 = 4000;
const VOL_IDX: usize = 0;
const N_SONGS: usize = 6;

const BUFFER_CAPACITY: usize = 512;
static DATA_BUFFER: Mutex<RefCell<Option<ArrayDeque<i16, BUFFER_CAPACITY>>>> = Mutex::new(RefCell::new(None));
static I2S_PERIPHERAL: Mutex<RefCell<Option<pac::I2S>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {

    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = hal::clock::GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let gclk0 = clocks.gclk0();
    let mut pm = peripherals.PM;
    let pins = bsp::Pins::new(peripherals.PORT);

    // setup status led
    let mut status_led: bsp::RedLed = pin_alias!(pins.red_led).into();
    status_led.set_low().unwrap(); // in case it was stuck on for some reason
    let mut delay = Delay::new(core.SYST, &mut clocks);

    // Setup UART peripheral and pins
    // this uses the labeled pins... but we can't do that because it conflicts with I2S pins.  So use an alternative pin pair
    // let mut uart = bsp::uart(
    //     &mut clocks,
    //     9600.Hz(),
    //     periph_alias!(peripherals.uart_sercom),
    //     &mut pm,
    //     pin_alias!(pins.uart_rx),
    //     pin_alias!(pins.uart_tx),
    // );
    let uart_clock = clocks.sercom1_core(&gclk0).unwrap();
    let uart_sercom = peripherals.SERCOM1;
    let uart_pads = uart::Pads::default()
                    .rx(pins.d11.into_alternate::<gpio::C>())
                    .tx(pins.d10.into_alternate::<gpio::C>());
    let mut uart = uart::Config::new(&pm, uart_sercom, uart_pads, uart_clock.freq())
        .baud(57600.Hz(), uart::BaudMode::Fractional(uart::Oversampling::Bits16))
        .enable();

    // write any panic message - do this first before any other setup
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        uart.write_all("last start yielded the following panic:\n\r".as_bytes()).expect("Could not write panic to uart!!");
        uart.write_all(msg).expect("Could not write panic to uart!!");
        uart.write_all("\n\r".as_bytes()).expect("Could not write panic to uart!!");

        
        // slow blink forever, means a reset is needed
        loop {
            status_led.set_high().unwrap();
            delay.delay_ms(750u32);
            status_led.set_low().unwrap();
            delay.delay_ms(750u32);

        }
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
    let mut volume_mgr: VolumeManager<_, _, N_SONGS, N_SONGS, 1> = VolumeManager::new_with_limits(card, FakeClock {}, 0);
    let mut sdvolume = volume_mgr
        .open_volume(VolumeIdx(VOL_IDX))
        .expect("Failed to open volume");

    // now search the root directory for songs that match the JJ#.WAV format
    let mut song_filenames: [Option<ShortFileName> ; N_SONGS] = Default::default();
    
    let mut root_dir = sdvolume.open_root_dir().expect("Failed to open root dir");
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

    // note: it would be better if we could actually open the files above but mutability creates all kinds of issues that won't be fixed until embedded_sdmmc > 0.8    

    // set up the song trigger pins
    let mut song_trigger_pins: [hal::gpio::DynPin ; N_SONGS] = [pins.a0.into(), pins.a1.into(), pins.a2.into(), pins.a3.into(), pins.a4.into(), pins.a5.into()];
    for pin in song_trigger_pins.as_mut() { pin.into_pull_up_input(); }
    

    //----------START I2S STARTUP-----------
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

    uart.write_fmt(format_args!("setting div as {} for rate {} which yields a true sample rate of {}\r\n", 
                                div, sample_rate, 48_000_000 as f32 / div as f32/ (bits_per_sample * nchannels) as f32))
        .expect("Could not write to uart!!");

    // configure a clock generator for the needed frequency and connect it to the i2s clocks
    let i2s_gclk = clocks.configure_gclk_divider_and_source(
                hal::clock::ClockGenId::GCLK3,
                div, 
                hal::clock::ClockSource::DFLL48M, 
                true).expect("the gclk for i2s is already configured!");
    let _i2s_clock0 = clocks.i2s0(&i2s_gclk);
    let _i2s_clock1 = clocks.i2s1(&i2s_gclk);

    // now configure the i2s registers and enable the clock but not the serializer
    // first do a reset just in  case
    peripherals.I2S.ctrla.write(|w| w.swrst().set_bit());
    // wait for reset to complete
    while peripherals.I2S.syncbusy.read().swrst().bit_is_set() {}
    while peripherals.I2S.ctrla.read().swrst().bit_is_set() {}

    // set up the clock unit
    peripherals.I2S.clkctrl[0].write(|w| {
        w.bitdelay().set_bit()  // not entirely sure about this one...
            .slotsize()._16()
    });
    // and the serializer
    peripherals.I2S.serctrl[0].write(|w| {
        w.mono().set_bit()
            .datasize()._16()
            .clksel().clk0()
            .txsame().set_bit()
            .txdefault().hiz()
            .sermode().tx()
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

    // send off the i2s peripheral to the global and set up the data buffer while we're at it, since we need these there once I2S is  enabled
    cortex_m::interrupt::free(|cs| {
        I2S_PERIPHERAL.borrow(cs).replace(Some(peripherals.I2S));
        DATA_BUFFER.borrow(cs).replace(Some(ArrayDeque::new()));
    });

    // now we do not enable it because we only do that when we need to play sound
    
    //----------END I2S STARTUP-----------


    // need to get a second delay because the main one was taken by the SD card controller
    // this is probably safe since it's set up exactly the same way as above as long as we don't try to do multithreading. But it's still a bit sketchy.
    let mut stolendelay = Delay::new(unsafe { CorePeripherals::steal() }.SYST, &mut clocks);

    // let rtc_clock = clocks.rtc(&gclk0).unwrap();
    // let rtc = hal::rtc::Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut pm);
    // rtc.count32();
    

    // Completed setup! entering mainloop
    uart.write_all("started!\n\r".as_bytes()).expect("Could not write start to uart!!");

    blink_led(&mut status_led, &mut stolendelay, 5, 100);
    status_led.set_low().expect("led setting failed!");

    loop {
        for i in 0..song_trigger_pins.len() {
            if song_trigger_pins[i].is_low().expect("Failed to read song trigger pin") {
                // do a debounce check
                stolendelay.delay_ms(5u8);
                if song_trigger_pins[i].is_low().expect("Failed to read song trigger pin") {
                    match song_filenames[i].clone() {
                        Some(songfn) => {
                            uart.write_fmt(format_args!("Playing song for trigger {}\r\n", i)).expect("Could not write to uart!!");
                            let mut songfile = root_dir.open_file_in_dir(songfn, embedded_sdmmc::Mode::ReadOnly).expect("Failed to open song file");
                            status_led.set_high().expect("led setting failed!");
                            let completed = play_song(&mut songfile, &song_trigger_pins[i], &mut stolendelay);
                            status_led.set_low().expect("led setting failed!");
                            songfile.close().expect("Failed to close song file");
                            uart.write_fmt(format_args!("Song for trigger {} ended as completed={}\r\n", i, completed)).expect("Could not write to uart!!");
                        },
                        None => {
                            uart.write_fmt(format_args!("No song found for trigger {}\r\n", i)).expect("Could not write to uart!!");
                        }
                    }
                }
            }
        }
    stolendelay.delay_ms(5u8);  // to kep the CPU mostly idleish when not playing music
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

static mut SONG_COUNTER: usize = 0;

fn play_song<D:BlockDevice,T:TimeSource>(file: &mut File<D,T,N_SONGS, N_SONGS, 1>,
                                         trigger_pin: &hal::gpio::DynPin, 
                                         delay: &mut Delay) -> bool{
    unsafe { SONG_COUNTER += 1; } 
    
    let freq = validate_wav_file(file).unwrap();  // this advances to the start of the data chunk
    if freq != 22_050 { panic!("Only 22.05kHz sample rate is currently supported!"); }

    // First read out a single sample to initialize
    let samplebuf = &mut [0u8; 2];
    if file.read(samplebuf).expect("Failed to read from file") < 2 {
         panic!("Failed to read first sample from file"); 
    }
    let data0 = i16::from_le_bytes(*samplebuf);
    
    // Then fill the buffer
    cortex_m::interrupt::free(|cs| {
        let mut data_buffer = DATA_BUFFER.borrow(cs).borrow_mut();
        let data_buffer = data_buffer.as_mut().expect("Data buffer not initialized");

        data_buffer.clear(); // in case it has data from the previous song still

        let local_buffer = &mut [0u8; 2*BUFFER_CAPACITY];
        let nread = file.read(local_buffer).expect("Failed to read from file");

        for i in 0..(nread/2) {
            let sample = i16::from_le_bytes(local_buffer[2*i..2*i+2].try_into().unwrap());
            data_buffer.push_back(sample).expect("Data buffer overflow!!!");
        }
    });

    // now prep and start up the I2S peripheral

    start_i2s(data0 as u16 as u32);

    let mut completed = false;

    // playing loop.  Runs until the trigger pin goes high
    let mut buffer_idx = 0;
    let mut nreadm1 = None;
    let read_buffer = &mut [0u8; 2*BUFFER_CAPACITY];
    loop {
        
        match nreadm1 {
            None => {
                if file.is_eof() {
                    // we've finished the song, quit the playing loop
                    // TODO: figure out how to wait until the data buffer is drained before quitting
                    completed = true;
                    break; 
                }
                // read into the buffer.  This is by far the slowest operation so it's where the main data buffer is most likely to get drained
                nreadm1 = Some(file.read(read_buffer).expect("Failed to read from file") - 1);
                buffer_idx = 0;
            },
            Some(nreadm1val) => {
                if buffer_idx >= nreadm1val { // this is why we need the -1 above - catches the unlikely but possible case of an odd number of bytes read
                    nreadm1 = None;
                }  else {
                    // write to the buffer only one at a time to give plenty of  time for the data interrupt to trigger
                    cortex_m::interrupt::free(|cs| {
                        let mut data_buffer = DATA_BUFFER.borrow(cs).borrow_mut();
                        let data_buffer = data_buffer.as_mut().expect("Data buffer not initialized");
                        if !data_buffer.is_full() {
                            let sample = i16::from_le_bytes(read_buffer[buffer_idx..buffer_idx+2].try_into().unwrap());
                            data_buffer.push_back(sample).expect("Data buffer overflow!!!");
                            buffer_idx += 2;
                        }
                    });
                }
            }
        }
        

        // high means stop, because the trigger pin is active low
        if trigger_pin.is_high().expect("Failed to read song trigger pin") {
            // debounce check
            delay.delay_ms(5u8);
            if trigger_pin.is_high().expect("Failed to read song trigger pin") {
                break; // interrupt the song, breaking out of the loop
            }
        }
    }
    let underrun = stop_i2s();
    if underrun { panic!("I2S underrun detected while playing this song"); }
    completed
}

fn start_i2s(data0: u32) {
    cortex_m::interrupt::free(|cs| {
        let mut i2s = I2S_PERIPHERAL.borrow(cs).borrow_mut();
        let i2s = i2s.as_mut().expect("I2S peripheral variable not initialized");

        // reset the interrupts flags just in case they're still set someohow from the past
        i2s.intflag.write(|w| w.txur0().set_bit().txrdy0().set_bit());

        // add the first sample to the i2s buffer
        i2s.data[0].write(|w| unsafe { w.data().bits(data0) });

        // enable the i2s clock0, ser0, and wait for sync, then enable the peripheral
        i2s.ctrla.write(|w| {
            w.cken0().set_bit()
            .seren0().set_bit()
            .enable().set_bit()
            .cken1().clear_bit()
            .seren1().clear_bit()
            .swrst().clear_bit()
        });
        unsafe { if SONG_COUNTER > 1 { panic!("here!"); } }
        while i2s.syncbusy.read().bits() != 0 {}

    unsafe { if SONG_COUNTER > 1 { panic!("huzzah!"); } }
    });
}

fn stop_i2s() -> bool {
    let mut underrun = false;
    cortex_m::interrupt::free(|cs| {
        let mut i2s = I2S_PERIPHERAL.borrow(cs).borrow_mut();
        let i2s = i2s.as_mut().expect("I2S peripheral variable not initialized");

        i2s.ctrla.write(|w| w.enable().clear_bit());
        while i2s.syncbusy.read().enable().bit_is_set() {}

        underrun = i2s.intflag.read().txur0().bit_is_set();
        // reset the interrupts just in case too once it's been disabled
        i2s.intflag.write(|w| w.txur0().set_bit().txrdy0().set_bit());
    });
    underrun
}

fn validate_wav_file<D:BlockDevice,T:TimeSource>(file: &mut embedded_sdmmc::File<D,T,N_SONGS, N_SONGS, 1>) -> Result<u32, WavError>{
    file.seek_from_start(0).expect("failed to seek to start of file");

    let buf4 = &mut [0u8; 4];
    let buf2 = &mut [0u8; 2];

    file.read(buf4).expect("failed to read from file");
    if buf4 != b"RIFF" { return Err(WavError::InvalidWavError); }

    file.read(buf4).expect("failed to read from file");
    file.read(buf4).expect("failed to read from file");
    if buf4 != b"WAVE" { return Err(WavError::InvalidWavError); }
    file.read(buf4).expect("failed to read from file");
    if buf4 != b"fmt " { return Err(WavError::InvalidWavError); }
    file.read(buf4).expect("failed to read from file");
    if u32::from_le_bytes(*buf4) != 16 { return Err(WavError::InvalidWavError); }

    file.read(buf2).expect("failed to read from file");
    let audio_format = u16::from_le_bytes(*buf2);
    file.read(buf2).expect("failed to read from file");
    let nchan = u16::from_le_bytes(*buf2);
    file.read(buf4).expect("failed to read from file");
    let freq = u32::from_le_bytes(*buf4);
    file.read(buf4).expect("failed to read from file");
    let _bytepersec = u32::from_le_bytes(*buf4);
    file.read(buf2).expect("failed to read from file");
    let _byteperblock = u16::from_le_bytes(*buf2);
    file.read(buf2).expect("failed to read from file");
    let bitspersample = u16::from_le_bytes(*buf2);

    file.read(buf4).expect("failed to read from file");
    if buf4 != b"data" { return Err(WavError::InvalidWavError); }

    if audio_format != 1 { return Err(WavError::IncorrectWavFormatError); }
    if nchan != 1 { return Err(WavError::IncorrectWavFormatError); }
    if bitspersample != 16 { return Err(WavError::IncorrectWavFormatError); }

    Ok(freq)
}

#[derive(Debug, Clone)]
enum WavError {
    InvalidWavError,
    IncorrectWavFormatError,
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


#[interrupt]
fn I2S() {
    // this interrupt is only triggered on TXRDY0
    cortex_m::interrupt::free(|cs| {
        let mut i2s = I2S_PERIPHERAL.borrow(cs).borrow_mut();
        let mut data_buffer = DATA_BUFFER.borrow(cs).borrow_mut();
        let i2s = i2s.as_mut().expect("I2S peripheral variable not initialized");
        let data_buffer = data_buffer.as_mut().expect("Data buffer not initialized");
        let data = data_buffer.pop_front().expect("Data buffer underflow!!!");  // could instead just let it ride to skip...

        // write the next sample to the i2s peripheral
        i2s.data[0].write(|w| unsafe { w.data().bits(data as u16 as u32) });
        // note the interrupt is cleared by writing the data register so we don't have to do it explicitly
    });
}