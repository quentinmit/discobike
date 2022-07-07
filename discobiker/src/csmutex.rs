#![no_std]
use embassy::blocking_mutex::CriticalSectionMutex;

use core::cell;

use critical_section;

use shared_bus;

use shared_bus::BusMutex;

pub struct CriticalSectionBusMutex<T> {
    csm: CriticalSectionMutex<cell::RefCell<T>>,
}

impl<T> BusMutex for CriticalSectionBusMutex<T> {
    type Bus = T;

    fn create(v: T) -> Self {
        Self { csm: CriticalSectionMutex::new(cell::RefCell::new(v)) }
    }

    fn lock<R, F: FnOnce(&mut Self::Bus) -> R>(&self, f: F) -> R {
        critical_section::with(|cs| {
            let c = self.csm.borrow(cs);
            f(&mut c.borrow_mut())
        })
    }
}
