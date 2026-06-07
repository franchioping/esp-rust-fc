#![no_std]
#![no_main]

use esp_println as _;

use esp_backtrace as _;
use esp_hal::{
    main,
    system::{CpuControl, Stack},
    time::Instant,
};

use core::ptr::addr_of_mut;
use defmt::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    println!("Init!");

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    static mut APP_CORE_STACK: Stack<{ 1024 * 16 }> = Stack::new();
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            println!("second core started");
            loop {
                println!("Bing on 2nd core!");

                let now = Instant::now();
                while now.elapsed().as_millis() < 5000 {}
            }
        })
        .unwrap();

    loop {
        println!("Bing!");

        let now = Instant::now();
        while now.elapsed().as_millis() < 5000 {}
    }
}
