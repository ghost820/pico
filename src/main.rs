//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{
    self as bsp,
    hal::{
        fugit::RateExtU32 as _,
        gpio::{
            bank0::{Gpio0, Gpio1},
            FunctionPio0, FunctionUart, Pin, PullDown, PullUp,
        },
        pac::interrupt,
        pio::{PIOBuilder, PIOExt as _},
        uart::{self, DataBits, StopBits, UartConfig, UartPeripheral},
    },
    pac::{Interrupt, NVIC, UART0},
};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use heapless::spsc::{self, Queue};
use pio_proc::pio_asm;

type Uart0Tx = Pin<Gpio0, FunctionUart, PullDown>;
type Uart0Rx = Pin<Gpio1, FunctionUart, PullUp>;
type Uart0Reader = uart::Reader<UART0, (Uart0Tx, Uart0Rx)>;
static mut UART_RX: Option<Uart0Reader> = None;
static mut UART_RX_BUFFER: Queue<u8, 128> = Queue::new();
static mut UART_RX_BUFFER_PROD: Option<spsc::Producer<'static, u8>> = None;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
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

    let uart_tx_pin = pins.gpio0.into_function::<FunctionUart>();
    let uart_tx_pin_num = uart_tx_pin.id().num;
    let uart_rx_pin = pins
        .gpio1
        .into_pull_up_input()
        .into_function::<FunctionUart>();
    let mut uart = UartPeripheral::new(pac.UART0, (uart_tx_pin, uart_rx_pin), &mut pac.RESETS)
        .enable(
            UartConfig::new(2400_u32.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart.enable_rx_interrupt();
    let (uart_rx, _uart_tx) = uart.split();
    let (uart_rx_prod, mut uart_rx_cons) = unsafe { UART_RX_BUFFER.split() };
    unsafe {
        UART_RX = Some(uart_rx);
        UART_RX_BUFFER_PROD = Some(uart_rx_prod);
    }

    let ir_tx_pin = pins.gpio2.into_function::<FunctionPio0>();
    let ir_tx_pin_num = ir_tx_pin.id().num;

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let program = pio_asm!(
        "set pindirs, 1",
        "loop:",
        "    jmp pin idle",
        "    set pins, 1 [31]",
        "    set pins, 0 [31]",
        "    jmp loop",
        "idle:",
        "    set pins, 0",
        "    jmp loop",
    );
    let installed = pio0.install(&program.program).unwrap();
    let (sm, _rx, _tx) = PIOBuilder::from_installed_program(installed)
        .set_pins(ir_tx_pin_num, 1)
        .jmp_pin(uart_tx_pin_num)
        .clock_divisor_fixed_point(51, 102)
        .build(sm0);

    let _sm = sm.start();

    unsafe {
        core.NVIC.set_priority(Interrupt::UART0_IRQ, 2);
        NVIC::unmask(Interrupt::UART0_IRQ);
    }

    loop {
        while let Some(_b) = uart_rx_cons.dequeue() {
            info!("Received byte: {}", _b as char);
        }
    }
}

#[interrupt]
fn UART0_IRQ() {
    let mut buf = [0u8; 32];

    let (rx, prod) = unsafe {
        match (UART_RX.as_ref(), UART_RX_BUFFER_PROD.as_mut()) {
            (Some(rx), Some(prod)) => (rx, prod),
            _ => return,
        }
    };

    loop {
        match rx.read_raw(&mut buf) {
            Ok(n) => {
                for &b in &buf[..n] {
                    let _ = prod.enqueue(b);
                }
            }
            Err(nb::Error::WouldBlock) => break,
            Err(nb::Error::Other(e)) => {
                for &b in e.discarded {
                    let _ = prod.enqueue(b);
                }
            }
        }
    }
}

// End of file
