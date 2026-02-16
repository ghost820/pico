//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{
    self as bsp,
    hal::{
        adc::AdcPin,
        dma::{self, DMAExt as _},
        Adc,
    },
};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

const DSP_BUFFER_SIZE: usize = 1024;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut led_pin = pins.led.into_push_pull_output();

    let adc_fs_hz = 44100;
    let adc_clk_hz = clocks.adc_clock.freq().to_Hz();
    let adc_div = (adc_clk_hz / adc_fs_hz - 1) as u16;
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
    let mut adc_fifo = adc
        .build_fifo()
        .clock_divider(adc_div, 0)
        .set_channel(&mut adc_pin)
        .enable_dma()
        .start_paused();

    let dma_buf_a = singleton!(: [u16; DSP_BUFFER_SIZE] = [0; DSP_BUFFER_SIZE]).unwrap();
    let dma_buf_b = singleton!(: [u16; DSP_BUFFER_SIZE] = [0; DSP_BUFFER_SIZE]).unwrap();
    let dma = pac.DMA.split(&mut pac.RESETS);
    let dma_transfer =
        dma::double_buffer::Config::new((dma.ch0, dma.ch1), adc_fifo.dma_read_target(), dma_buf_a)
            .start();
    let mut dma_transfer = dma_transfer.write_next(dma_buf_b);

    adc_fifo.resume();

    loop {
        let (filled, next) = dma_transfer.wait();

        dsp(filled);

        dma_transfer = next.write_next(filled);
    }
}

fn dsp(samples: &[u16; DSP_BUFFER_SIZE]) {
    for &s in samples {
        // s & 0x0fff
    }
}

// End of file
