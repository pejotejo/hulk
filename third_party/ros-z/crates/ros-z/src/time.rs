use std::{
    fmt,
    future::Future,
    ops::{Add, Sub},
    pin::Pin,
    sync::Arc,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use parking_lot::Mutex;
use ros_z_protocol::TypeHash;
use serde::{Deserialize, Serialize};
use tokio::sync::Notify;

use crate::{
    FieldTypeInfo, MessageTypeInfo,
    dynamic::{FieldSchema, FieldType, MessageSchema, MessageSchemaTypeDescription},
};

/// A clock-relative instant used throughout ros-z.
///
/// `ZTime` is intentionally generic: it represents an instant on some clock's
/// timeline and only becomes wallclock time when interpreted through a
/// wallclock-backed [`ZClock`] or converted with [`ZTime::from_wallclock`] /
/// [`ZTime::to_wallclock`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ZTime {
    since_origin: Duration,
}

impl MessageTypeInfo for ZTime {
    fn type_name() -> &'static str {
        "ros_z::msg::dds_::ZTime_"
    }

    fn type_hash() -> TypeHash {
        let rihs_string = Self::message_schema()
            .unwrap()
            .compute_type_hash()
            .expect("failed to compute type hash")
            .to_rihs_string();
        TypeHash::from_rihs_string(&rihs_string).expect("failed to compute type hash")
    }

    fn message_schema() -> Option<std::sync::Arc<crate::dynamic::MessageSchema>> {
        Some(Arc::new(crate::dynamic::MessageSchema {
            type_name: "ros_z/msg/ZTime".to_string(),
            package: "ros_z".to_string(),
            name: "ZTime".to_string(),
            fields: vec![FieldSchema::new("duration", Duration::field_type())],
            type_hash: None,
        }))
    }
}

impl FieldTypeInfo for Duration {
    fn field_type() -> crate::dynamic::FieldType {
        FieldType::Message(
            MessageSchema::builder("std/msg/Duration")
                .field("secs", FieldType::Uint64)
                .field("nanos", FieldType::Uint32)
                .build()
                .expect("failed to build schema for Duration"),
        )
    }
}

impl fmt::Debug for ZTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ZTime")
            .field("secs", &self.since_origin.as_secs())
            .field("nanos", &self.since_origin.subsec_nanos())
            .finish()
    }
}

impl ZTime {
    pub fn zero() -> Self {
        Self {
            since_origin: Duration::ZERO,
        }
    }

    /// Convert a wallclock timestamp into a `ZTime` instant.
    pub fn from_wallclock(time: SystemTime) -> Self {
        let since_origin = time.duration_since(UNIX_EPOCH).unwrap_or(Duration::ZERO);
        Self { since_origin }
    }

    #[deprecated(note = "use ZTime::from_wallclock instead")]
    pub fn from_system_time(time: SystemTime) -> Self {
        Self::from_wallclock(time)
    }

    /// Construct a `ZTime` from a raw nanosecond count on the active timeline.
    pub fn from_nanos(nanos: i64) -> Self {
        let nanos = u64::try_from(nanos).unwrap_or_default();
        Self {
            since_origin: Duration::from_nanos(nanos),
        }
    }

    #[deprecated(note = "use ZTime::from_nanos instead")]
    pub fn from_unix_nanos(nanos: i64) -> Self {
        Self::from_nanos(nanos)
    }

    /// Interpret this instant as wallclock time.
    pub fn to_wallclock(self) -> SystemTime {
        UNIX_EPOCH + self.since_origin
    }

    #[deprecated(note = "use ZTime::to_wallclock instead")]
    pub fn to_system_time(self) -> SystemTime {
        self.to_wallclock()
    }

    /// Return the raw nanosecond position of this instant on its timeline.
    pub fn as_nanos(self) -> i64 {
        self.since_origin.as_nanos().min(i64::MAX as u128) as i64
    }

    #[deprecated(note = "use ZTime::as_nanos instead")]
    pub fn as_unix_nanos(self) -> i64 {
        self.as_nanos()
    }

    pub fn saturating_add(self, duration: Duration) -> Self {
        Self {
            since_origin: self.since_origin.saturating_add(duration),
        }
    }

    pub fn saturating_sub(self, duration: Duration) -> Self {
        Self {
            since_origin: self.since_origin.saturating_sub(duration),
        }
    }

    pub fn duration_since(self, earlier: ZTime) -> Duration {
        self.since_origin.saturating_sub(earlier.since_origin)
    }
}

impl From<SystemTime> for ZTime {
    fn from(value: SystemTime) -> Self {
        Self::from_wallclock(value)
    }
}

impl Add<Duration> for ZTime {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        self.saturating_add(rhs)
    }
}

impl Sub<Duration> for ZTime {
    type Output = Self;

    fn sub(self, rhs: Duration) -> Self::Output {
        self.saturating_sub(rhs)
    }
}

impl Default for ZTime {
    fn default() -> Self {
        Self::zero()
    }
}

#[derive(Debug)]
pub enum ClockError {
    NotLogical,
    TimeWentBackwards,
}

impl fmt::Display for ClockError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ClockError::NotLogical => write!(f, "clock is not logical"),
            ClockError::TimeWentBackwards => write!(f, "logical time cannot move backwards"),
        }
    }
}

impl std::error::Error for ClockError {}

#[derive(Clone)]
pub struct ZClock {
    inner: Arc<ClockInner>,
}

enum ClockInner {
    Wallclock,
    Logical(LogicalClockState),
}

struct LogicalClockState {
    now: Mutex<ZTime>,
    notify: Notify,
}

impl fmt::Debug for ZClock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let kind = match self.inner.as_ref() {
            ClockInner::Wallclock => "Wallclock",
            ClockInner::Logical(_) => "Logical",
        };

        f.debug_struct("ZClock")
            .field("kind", &kind)
            .finish_non_exhaustive()
    }
}

impl Default for ZClock {
    fn default() -> Self {
        Self::wallclock()
    }
}

impl ZClock {
    pub fn wallclock() -> Self {
        Self {
            inner: Arc::new(ClockInner::Wallclock),
        }
    }

    #[deprecated(note = "use ZClock::wallclock instead")]
    pub fn system() -> Self {
        Self::wallclock()
    }

    pub fn logical(start: ZTime) -> Self {
        Self {
            inner: Arc::new(ClockInner::Logical(LogicalClockState {
                now: Mutex::new(start),
                notify: Notify::new(),
            })),
        }
    }

    #[deprecated(note = "use ZClock::logical instead")]
    pub fn simulated(start: ZTime) -> Self {
        Self::logical(start)
    }

    pub fn now(&self) -> ZTime {
        match self.inner.as_ref() {
            ClockInner::Wallclock => ZTime::from_wallclock(SystemTime::now()),
            ClockInner::Logical(state) => *state.now.lock(),
        }
    }

    pub fn set_time(&self, time: ZTime) -> Result<(), ClockError> {
        match self.inner.as_ref() {
            ClockInner::Wallclock => Err(ClockError::NotLogical),
            ClockInner::Logical(state) => {
                let mut current = state.now.lock();
                if time < *current {
                    return Err(ClockError::TimeWentBackwards);
                }
                *current = time;
                state.notify.notify_waiters();
                Ok(())
            }
        }
    }

    pub fn advance(&self, delta: Duration) -> Result<ZTime, ClockError> {
        match self.inner.as_ref() {
            ClockInner::Wallclock => Err(ClockError::NotLogical),
            ClockInner::Logical(state) => {
                let mut current = state.now.lock();
                *current = current.saturating_add(delta);
                let now = *current;
                state.notify.notify_waiters();
                Ok(now)
            }
        }
    }

    pub fn sleep_until(&self, deadline: ZTime) -> ZSleep {
        match self.inner.as_ref() {
            ClockInner::Wallclock => {
                let now = SystemTime::now();
                let deadline = deadline.to_wallclock();
                let duration = deadline.duration_since(now).unwrap_or(Duration::ZERO);
                ZSleep(Box::pin(tokio::time::sleep(duration)))
            }
            ClockInner::Logical(_) => {
                let clock = self.clone();
                ZSleep(Box::pin(async move {
                    loop {
                        // Obtain and *enable* the Notified future before checking the
                        // condition.  `enable()` registers this task as a waiter
                        // immediately, so a concurrent `notify_waiters()` call that
                        // fires between the condition check and the first `.await` poll
                        // is not lost.
                        let notified = match clock.inner.as_ref() {
                            ClockInner::Wallclock => unreachable!(),
                            ClockInner::Logical(state) => state.notify.notified(),
                        };
                        tokio::pin!(notified);
                        notified.as_mut().enable();

                        if clock.now() >= deadline {
                            break;
                        }
                        notified.await;
                    }
                }))
            }
        }
    }

    pub fn sleep(&self, duration: impl Into<Duration>) -> ZSleep {
        let deadline = self.now().saturating_add(duration.into());
        self.sleep_until(deadline)
    }

    pub fn interval(&self, period: impl Into<Duration>) -> ZInterval {
        let period = period.into();
        ZInterval {
            clock: self.clone(),
            period,
            next_deadline: self.now().saturating_add(period),
        }
    }

    /// Create a reusable timer tied to this clock.
    ///
    /// Unlike [`ZInterval`], a [`ZTimer`] exposes convenience methods to inspect
    /// and reset its cadence, making it a better fit for long-lived robotics
    /// tasks that need explicit periodic scheduling.
    pub fn timer(&self, period: impl Into<Duration>) -> ZTimer {
        ZTimer::new(self.clone(), period)
    }
}

pub struct ZSleep(Pin<Box<dyn Future<Output = ()> + Send>>);

impl Future for ZSleep {
    type Output = ();

    fn poll(
        mut self: Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        self.0.as_mut().poll(cx)
    }
}

pub struct ZInterval {
    clock: ZClock,
    period: Duration,
    next_deadline: ZTime,
}

impl ZInterval {
    pub async fn tick(&mut self) -> ZTime {
        self.clock.sleep_until(self.next_deadline).await;
        let fired_at = self.next_deadline;
        self.next_deadline = self.next_deadline.saturating_add(self.period);
        fired_at
    }
}

#[derive(Debug, Clone)]
pub struct ZTimer {
    clock: ZClock,
    period: Duration,
    start: ZTime,
}

impl ZTimer {
    pub fn new(clock: ZClock, period: impl Into<Duration>) -> Self {
        let period = period.into();
        Self {
            start: clock.now(),
            clock,
            period,
        }
    }

    pub fn period(&self) -> Duration {
        self.period
    }

    pub fn deadline(&self) -> ZTime {
        self.start.saturating_add(self.period)
    }

    pub fn reset(&mut self) {
        self.start = self.clock.now();
    }

    pub fn set_period(&mut self, period: impl Into<Duration>) {
        self.period = period.into();
    }

    pub async fn tick(&mut self) -> ZTime {
        let deadline = self.deadline();
        self.clock.sleep_until(deadline).await;
        let fired_at = deadline;
        self.start = fired_at;
        fired_at
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wallclock_is_default() {
        let clock = ZClock::default();
        assert!(matches!(
            clock.set_time(ZTime::zero()),
            Err(ClockError::NotLogical)
        ));
    }

    #[tokio::test]
    async fn logical_clock_can_advance_manually() {
        let clock = ZClock::logical(ZTime::zero());
        let mut interval = clock.interval(Duration::from_secs(1));

        let waiter = tokio::spawn(async move { interval.tick().await });
        tokio::task::yield_now().await;
        assert!(!waiter.is_finished());

        clock.advance(Duration::from_secs(1)).unwrap();
        let tick = waiter.await.unwrap();
        assert_eq!(tick, ZTime::from_nanos(1_000_000_000));
    }

    #[tokio::test]
    async fn logical_sleep_follows_logical_time() {
        let clock = ZClock::logical(ZTime::zero());
        let sleep = clock.sleep(Duration::from_millis(10));

        let waiter = tokio::spawn(sleep);
        tokio::task::yield_now().await;
        assert!(!waiter.is_finished());

        clock.advance(Duration::from_millis(10)).unwrap();
        waiter.await.unwrap();
    }

    #[tokio::test]
    async fn logical_ztimer_follows_logical_time() {
        let clock = ZClock::logical(ZTime::zero());
        let mut timer = clock.timer(Duration::from_millis(10));

        let waiter = tokio::spawn(async move { timer.tick().await });
        tokio::task::yield_now().await;
        assert!(!waiter.is_finished());

        clock.advance(Duration::from_millis(10)).unwrap();
        let tick = waiter.await.unwrap();
        assert_eq!(tick, ZTime::from_nanos(10_000_000));
    }

    #[test]
    fn ztimer_reset_uses_current_clock_time() {
        let clock = ZClock::logical(ZTime::zero());
        let mut timer = clock.timer(Duration::from_secs(2));
        assert_eq!(timer.deadline(), ZTime::from_nanos(2_000_000_000));

        clock.advance(Duration::from_secs(5)).unwrap();
        timer.reset();

        assert_eq!(timer.deadline(), ZTime::from_nanos(7_000_000_000));
    }

    #[test]
    fn ztimer_set_period_before_first_tick_preserves_creation_anchor() {
        let clock = ZClock::logical(ZTime::zero());
        let mut timer = clock.timer(Duration::from_secs(2));

        timer.set_period(Duration::from_secs(5));

        assert_eq!(timer.deadline(), ZTime::from_nanos(5_000_000_000));
    }

    #[tokio::test]
    async fn ztimer_set_period_after_tick_preserves_last_fire_phase() {
        let clock = ZClock::logical(ZTime::zero());
        let mut timer = clock.timer(Duration::from_secs(2));

        let waiter = tokio::spawn(async move {
            let first_tick = timer.tick().await;
            timer.set_period(Duration::from_secs(5));
            (first_tick, timer.deadline())
        });

        tokio::task::yield_now().await;
        clock.advance(Duration::from_secs(2)).unwrap();

        let (first_tick, next_deadline) = waiter.await.unwrap();
        assert_eq!(first_tick, ZTime::from_nanos(2_000_000_000));
        assert_eq!(next_deadline, ZTime::from_nanos(7_000_000_000));
    }

    #[tokio::test]
    async fn logical_sleep_no_lost_wakeup_when_advance_before_poll() {
        // Regression test: advance the clock past the deadline *before* the sleep
        // future is ever polled.  Without the enable() fix this would hang forever
        // because notify_waiters() fires before the future registers as a waiter.
        let clock = ZClock::logical(ZTime::zero());
        let sleep = clock.sleep(Duration::from_millis(10));
        // Advance BEFORE yielding — the future has not been polled yet.
        clock.advance(Duration::from_millis(10)).unwrap();
        tokio::time::timeout(std::time::Duration::from_secs(1), sleep)
            .await
            .expect("sleep should resolve without hanging");
    }

    // --- Duration ---

    #[test]
    fn duration_from_secs_and_as_std() {
        let d = Duration::from_secs(3);
        assert_eq!(d.as_std(), Duration::from_secs(3));
    }

    #[test]
    fn duration_default_is_zero() {
        assert_eq!(Duration::default().as_std(), Duration::ZERO);
    }

    #[test]
    fn duration_roundtrip_from_std() {
        let std_d = Duration::from_millis(500);
        let zd = Duration::from(std_d);
        let back: Duration = zd.into();
        assert_eq!(back, std_d);
    }

    // --- ZTime ---

    #[test]
    fn ztime_zero_and_default() {
        assert_eq!(ZTime::zero(), ZTime::default());
        assert_eq!(ZTime::zero().as_nanos(), 0);
    }

    #[test]
    fn ztime_from_nanos_negative_clamps_to_zero() {
        assert_eq!(ZTime::from_nanos(-1).as_nanos(), 0);
    }

    #[test]
    fn ztime_from_wallclock_roundtrip() {
        let t = ZTime::from_nanos(1_000_000_000);
        let sys = t.to_wallclock();
        let back = ZTime::from_wallclock(sys);
        assert_eq!(back, t);
    }

    #[test]
    fn ztime_saturating_add_sub() {
        let t = ZTime::from_nanos(5_000_000_000);
        let d = Duration::from_secs(2);
        assert_eq!(t.saturating_add(d).as_nanos(), 7_000_000_000);
        assert_eq!(t.saturating_sub(d).as_nanos(), 3_000_000_000);
        // sub below zero saturates
        assert_eq!(ZTime::zero().saturating_sub(d).as_nanos(), 0);
    }

    #[test]
    fn ztime_duration_since() {
        let a = ZTime::from_nanos(5_000_000_000);
        let b = ZTime::from_nanos(3_000_000_000);
        assert_eq!(a.duration_since(b).as_std(), Duration::from_secs(2));
        // saturates to zero when earlier > self
        assert_eq!(b.duration_since(a).as_std(), Duration::ZERO);
    }

    #[test]
    fn ztime_debug_format() {
        let t = ZTime::from_nanos(1_500_000_000);
        let s = format!("{:?}", t);
        assert!(s.contains("secs"));
        assert!(s.contains("nanos"));
    }

    // --- ZClock constructors ---

    #[test]
    fn zclock_wallclock_constructor() {
        let c = ZClock::wallclock();
        assert!(matches!(
            c.set_time(ZTime::zero()),
            Err(ClockError::NotLogical)
        ));
    }

    #[test]
    fn zclock_debug_format() {
        let s = format!("{:?}", ZClock::wallclock());
        assert!(s.contains("ZClock"));
    }

    #[test]
    #[allow(deprecated)]
    fn legacy_time_aliases_still_work() {
        let t = ZTime::from_system_time(SystemTime::UNIX_EPOCH + Duration::from_secs(1));
        assert_eq!(t.as_unix_nanos(), 1_000_000_000);
        assert_eq!(
            t.to_system_time(),
            SystemTime::UNIX_EPOCH + Duration::from_secs(1)
        );
        assert_eq!(ZTime::from_unix_nanos(5).as_nanos(), 5);

        assert!(matches!(
            ZClock::system().set_time(ZTime::zero()),
            Err(ClockError::NotLogical)
        ));
        assert_eq!(ZClock::simulated(ZTime::zero()).now(), ZTime::zero());
    }

    #[test]
    fn wallclock_now_is_nonzero() {
        let t = ZClock::wallclock().now();
        assert!(t.as_nanos() > 0);
    }

    // --- set_time ---

    #[test]
    fn set_time_advances_logical_clock() {
        let clock = ZClock::logical(ZTime::zero());
        let t = ZTime::from_nanos(1_000_000_000);
        clock.set_time(t).unwrap();
        assert_eq!(clock.now(), t);
    }

    #[test]
    fn set_time_rejects_backwards() {
        let clock = ZClock::logical(ZTime::from_nanos(1_000_000_000));
        let err = clock.set_time(ZTime::zero()).unwrap_err();
        assert!(matches!(err, ClockError::TimeWentBackwards));
        assert!(!err.to_string().is_empty());
    }

    #[test]
    fn set_time_on_wallclock_errors() {
        let err = ZClock::wallclock().set_time(ZTime::zero()).unwrap_err();
        assert!(matches!(err, ClockError::NotLogical));
        assert!(!err.to_string().is_empty());
    }

    #[test]
    fn advance_on_wallclock_errors() {
        let err = ZClock::wallclock()
            .advance(Duration::from_secs(1))
            .unwrap_err();
        assert!(matches!(err, ClockError::NotLogical));
    }

    // --- wallclock sleep (just verify it doesn't block) ---

    #[tokio::test]
    async fn wallclock_sleep_zero_completes() {
        ZClock::wallclock().sleep(Duration::default()).await;
    }
}
