//! Pinouts:
//! * GPIO 0 - UART TX (out of the RP2040)
//! * GPIO 1 - UART RX (in to the RP2040)
//! * GPIO 16 - WS2812 Data

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use core::str::from_utf8;
use cortex_m::prelude::_embedded_hal_serial_Read;
//use cortex_m::interrupt::Mutex;
use embedded_hal::{digital::v2::OutputPin, serial::Write as UartWrite};
use fugit::RateExtU32;
//use hal::gpio::FunctionUart;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::pio::PIOExt;
use rp_pico::hal::prelude::*;
use rp_pico::hal::timer::Timer;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use pac::interrupt;

use critical_section::Mutex;
use hal::gpio::pin::bank0::{Gpio0, Gpio1};
use hal::uart::{DataBits, StopBits, UartConfig};
use heapless::spsc::Queue;

type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

struct UartQueue {
    mutex_cell_queue: Mutex<RefCell<Queue<u8, 64>>>,
    interrupt: pac::Interrupt,
}

const STRIP_LEN: usize = 8;

static GLOBAL_UART: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));
static UART_TX_QUEUE: UartQueue = UartQueue {
    mutex_cell_queue: Mutex::new(RefCell::new(Queue::new())),
    interrupt: hal::pac::Interrupt::UART0_IRQ,
};
static UART_RX_QUEUE: UartQueue = UartQueue {
    mutex_cell_queue: Mutex::new(RefCell::new(Queue::new())),
    interrupt: hal::pac::Interrupt::UART0_IRQ,
};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );

    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let mut frame_delay =
        cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    unsafe {
        // Enable UART interrupt in the NVIC (Nested Vectored Interrupt Controller)
        pac::NVIC::unmask(hal::pac::Interrupt::UART0_IRQ);
    }

    uart.enable_rx_interrupt();

    // Raise interrupt when TX FIFO has space
    uart.enable_tx_interrupt();

    critical_section::with(|cs| {
        GLOBAL_UART.borrow(cs).replace(Some(uart));
    });

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    // Import sine function
    let sin = hal::rom_data::float_funcs::fsin::ptr();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instanciate a Ws2812 LED strip:
    let mut ws = Ws2812::new(
        // Use pin 21 on the Raspberry Pi Pico (which is GPIO16 of the rp2040 chip)
        // for the LED data output:
        pins.gpio16.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut t = 0.0;

    let strip_brightness = 128u8; // Limit brightness to 128/256 (50%)

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 1.0;
    let mut led_toggle = 0;

    let mut hue: f32 = 215.0;

    loop {
        write!(&UART_TX_QUEUE, " ").unwrap();
        if UART_RX_QUEUE.peek_byte().is_some() {
            let recv_char = UART_RX_QUEUE.read_byte().unwrap() as char;

            if matches!(recv_char, 'A'..='Z' | 'a'..='z' | '0'..='9') {
                hue += 5.0;
                if hue > 360.0 {
                    hue -= 360.0
                };

                writeln!(&UART_TX_QUEUE, "Hue is now {hue}").unwrap();
            }
        }

        for (i, led) in leds.iter_mut().enumerate() {
            let sin_11 = sin(t * 2.0 * core::f32::consts::PI);
            // Bring -1..1 sine range to 0..1 range:
            let sin_01 = (sin_11 + 1.0) * 0.5;

            //hue = 215_f32;
            let sat = 1.0;
            let val = if i % 2 == led_toggle { 0.0 } else { sin_01 };

            let rgb = hsv2rgb_u8(hue, sat, val);
            *led = rgb.into();
        }

        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        // Wait before processing next frame
        frame_delay.delay_ms(16); // 16ms = approx 60 FPS

        t += (16_f32 / 1000_f32) * animation_speed;
        while t > 1.0 {
            t -= 1.0;

            if led_toggle == 1 {
                led_toggle = 0;
            } else {
                led_toggle = 1;
            }
        }
    }
}

impl UartQueue {
    fn read_byte(&self) -> Option<u8> {
        critical_section::with(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let mut queue = cell_queue.borrow_mut();
            queue.dequeue()
        })
    }

    fn peek_byte(&self) -> Option<u8> {
        critical_section::with(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let queue = cell_queue.borrow_mut();
            queue.peek().cloned()
        })
    }

    fn write_bytes_blocking(&self, data: &[u8]) {
        // Go through all the bytes we need to write.
        for byte in data.iter() {
            // Keep trying until there is space in the queue. But release the
            // mutex between each attempt, otherwise the IRQ will never run
            // and we will never have space!
            let mut written = false;
            while !written {
                // Grab the mutex, by turning interrupts off. NOTE: This
                // doesn't work if you are using Core 1 as we only turn
                // interrupts off on one core.
                critical_section::with(|cs| {
                    // Grab the mutex contents.
                    let cell_queue = self.mutex_cell_queue.borrow(cs);
                    // Grab mutable access to the queue. This can't fail
                    // because there are no interrupts running.
                    let mut queue = cell_queue.borrow_mut();
                    // Try and put the byte in the queue.
                    if queue.enqueue(*byte).is_ok() {
                        // It worked! We must have had space.
                        if !pac::NVIC::is_enabled(self.interrupt) {
                            unsafe {
                                // Now enable the UART interrupt in the *Nested
                                // Vectored Interrupt Controller*, which is part
                                // of the Cortex-M0+ core. If the FIFO has space,
                                // the interrupt will run as soon as we're out of
                                // the closure.
                                pac::NVIC::unmask(self.interrupt);
                                // We also have to kick the IRQ in case the FIFO
                                // was already below the threshold level.
                                pac::NVIC::pend(self.interrupt);
                            }
                        }
                        written = true;
                    }
                });
            }
        }
    }
}

impl core::fmt::Write for &UartQueue {
    fn write_str(&mut self, data: &str) -> core::fmt::Result {
        self.write_bytes_blocking(data.as_bytes());
        Ok(())
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}

#[interrupt]
fn UART0_IRQ() {
    // This variable is special. It gets mangled by the `#[interrupt]` macro
    // into something that we can access without the `unsafe` keyword. It can
    // do this because this function cannot be called re-entrantly. We know
    // this because the function's 'real' name is unknown, and hence it cannot
    // be called from the main thread. We also know that the NVIC will not
    // re-entrantly call an interrupt.
    static mut UART: Option<hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>> =
        None;

    // This is one-time lazy initialisation. We steal the variable given to us
    // via `GLOBAL_UART`.
    if UART.is_none() {
        critical_section::with(|cs| {
            *UART = GLOBAL_UART.borrow(cs).take();
        });
    }

    // Check if we have a UART to work with
    if let Some(uart) = UART {
        // Check if we have data to transmit
        while let Some(byte) = UART_TX_QUEUE.peek_byte() {
            if uart.write(byte).is_ok() {
                // The UART took it, so pop it off the queue.
                let _ = UART_TX_QUEUE.read_byte();
            } else {
                break;
            }
        }

        // Mask UART interrupt flag
        if UART_TX_QUEUE.peek_byte().is_none() {
            pac::NVIC::mask(hal::pac::Interrupt::UART0_IRQ);
        }

        // Check if we have data to receive
        while let Ok(byte) = uart.read() {
            write!(&UART_RX_QUEUE, "{}", from_utf8(&[byte]).unwrap()).unwrap();
        }
    }

    // Set an event to ensure the main thread always wakes up, even if it's in
    // the process of going to sleep.
    cortex_m::asm::sev();
}
