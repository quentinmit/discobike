use atomic_traits::{Atomic, NumOps};
use core::sync::atomic::Ordering;
use num_traits::One;

#[derive(Debug)]
pub struct CounterGuard<'a, T>(&'a T)
where
    T: Atomic + NumOps + Default,
    <T as Atomic>::Type: One;

pub trait Counter
where
    Self: Sized + Atomic + NumOps + Default,
    <Self as Atomic>::Type: One,
{
    fn count_raii<'a>(&'a self) -> CounterGuard<'a, Self>;
}

impl<T> Counter for T
where
    T: Atomic + NumOps + Default,
    <T as Atomic>::Type: One,
{
    fn count_raii<'a>(&'a self) -> CounterGuard<'a, T> {
        self.fetch_add(<T as Atomic>::Type::one(), Ordering::Relaxed);
        CounterGuard(self)
    }
}

impl<'a, T> Drop for CounterGuard<'a, T>
where
    T: Atomic + NumOps + Default,
    <T as Atomic>::Type: One,
{
    #[inline]
    fn drop(&mut self) {
        self.0
            .fetch_sub(<T as Atomic>::Type::one(), Ordering::Relaxed);
    }
}
