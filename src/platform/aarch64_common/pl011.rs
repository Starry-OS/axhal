//! PL011 UART.

use arm_pl011::pl011::Pl011Uart;
use memory_addr::PhysAddr;
use spinlock::SpinNoIrq;

use crate::mem::phys_to_virt;

const UART_BASE: PhysAddr = PhysAddr::from(axconfig::UART_PADDR);

#[cfg(feature = "irq")]
const BUFFER_SIZE: usize = 128;

#[cfg(feature = "irq")]
pub struct RxRingBuffer {
    buffer: [u8; BUFFER_SIZE],
    head: usize,
    tail: usize,
    empty: bool,
}

#[cfg(feature = "irq")]
impl RxRingBuffer {
    const fn new() -> Self {
        RxRingBuffer {
            buffer: [0_u8; BUFFER_SIZE],
            head: 0_usize,
            tail: 0_usize,
            empty: true,
        }
    }

    pub fn push(&mut self, n: u8) {
        if self.tail != self.head || self.empty {
            self.buffer[self.tail] = n;
            self.tail = (self.tail + 1) % BUFFER_SIZE;
            self.empty = false;
        }
    }

    pub fn pop(&mut self) -> Option<u8> {
        if self.empty {
            None
        } else {
            let ret = self.buffer[self.head];
            self.head = (self.head + 1) % BUFFER_SIZE;
            if self.head == self.tail {
                self.empty = true;
            }
            Some(ret)
        }
    }
}

pub struct Uart {
    pub inner: SpinNoIrq<Pl011Uart>,
    #[cfg(feature = "irq")]
    pub buffer: SpinNoIrq<RxRingBuffer>,
}

// static UART: SpinNoIrq<Pl011Uart> =
//     SpinNoIrq::new(Pl011Uart::new(phys_to_virt(UART_BASE).as_mut_ptr()));

pub static UART: Uart = Uart {
    inner: SpinNoIrq::new(Pl011Uart::new(phys_to_virt(UART_BASE).as_mut_ptr())),
    #[cfg(feature = "irq")]
    buffer: SpinNoIrq::new(RxRingBuffer::new()),
};

/// Writes a byte to the console.
pub fn putchar(c: u8) {
    let mut uart = UART.inner.lock();
    match c {
        b'\n' => {
            uart.putchar(b'\r');
            uart.putchar(b'\n');
        }
        c => uart.putchar(c),
    }
}

/// Reads a byte from the console, or returns [`None`] if no input is available.
pub fn getchar() -> Option<u8> {
    cfg_if::cfg_if! {
        if #[cfg(feature = "irq")] {
            UART.buffer.lock().pop()
        }else{
            UART.inner.lock().getchar()
        }
    }
}

/// Initialize the UART
pub fn init_early() {
    unsafe {
        crate::platform::aarch64_common::mem::idmap_device(UART_BASE.as_usize());
    }
    UART.inner.lock().init();
}

