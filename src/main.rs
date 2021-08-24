#![no_std]
#![no_main]

use panic_semihosting as _;

use core::cell::RefCell;
use cortex_m::interrupt::{free as interrupt_free, Mutex};
use cortex_m_rt::entry;
use mill::{
    rotary_encoder::RotaryEncoder,
    screen::{Frame, Screen, ScreenConfig},
    stepper_motor::{Duration, Mode, StepperMotor, StepperMotorConfig},
    Mill, MillConfig,
};
use stm32f1xx_hal::{
    delay::Delay,
    device::interrupt,
    gpio::{
        gpioa::{PA1, PA10, PA11, PA12, PA2, PA8, PA9},
        gpiob::{PB0, PB1, PB12, PB13, PB14, PB15, PB6, PB7},
        Edge, ExtiPin, Input, Output, PullDown, PushPull,
    },
    pac::{CorePeripherals, Interrupt, Peripherals, NVIC},
    prelude::*,
    rtc::Rtc,
};

const MOTOR_MODE: Mode = Mode::QuarterStep;
const MM_STEPS: u32 = 200 * 4;
const STEPS_PER_LOOP: u32 = 1;
const SIGNAL_DELAY: u8 = 1;

static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));
static DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static MILL: Mutex<
    RefCell<
        Option<
            Mill<
                PB0<Input<PullDown>>,
                PB1<Input<PullDown>>,
                PA1<Input<PullDown>>,
                PA2<Input<PullDown>>,
                PB6<Output<PushPull>>,
                PB7<Output<PushPull>>,
                PA10<Output<PushPull>>,
                PA11<Output<PushPull>>,
                PA12<Output<PushPull>>,
                u8,
                PB12<Output<PushPull>>,
                PB13<Output<PushPull>>,
                PB14<Output<PushPull>>,
                PB15<Output<PushPull>>,
                PA8<Output<PushPull>>,
                PA9<Output<PushPull>>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

// If `COUNTDOWN` is greater than 0, main loop will decrease it instead of
// rotating the motor. It is set by encoder interrupt, so there is a delay
// between user input and motor rotation.
static COUNTDOWN: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

// Number of ticks required to start rotating motor after user input.
const COUNTDOWN_VALUE: u32 = 1000;

#[entry]
fn main() -> ! {
    let core_peripherals = CorePeripherals::take().unwrap();
    let peripherals = Peripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
    let mut pwr = peripherals.PWR;
    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(core_peripherals.SYST, clocks);

    let mut sia = gpiob.pb0.into_pull_down_input(&mut gpiob.crl);
    sia.make_interrupt_source(&mut afio);
    sia.trigger_on_edge(&peripherals.EXTI, Edge::FALLING);
    sia.enable_interrupt(&peripherals.EXTI);

    let mut home_switch = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);
    home_switch.make_interrupt_source(&mut afio);
    home_switch.trigger_on_edge(&peripherals.EXTI, Edge::FALLING);
    home_switch.enable_interrupt(&peripherals.EXTI);

    let mut limit_switch = gpioa.pa2.into_pull_down_input(&mut gpioa.crl);
    limit_switch.make_interrupt_source(&mut afio);
    limit_switch.trigger_on_edge(&peripherals.EXTI, Edge::RISING);
    limit_switch.enable_interrupt(&peripherals.EXTI);

    let mut backup_domain = rcc.bkp.constrain(peripherals.BKP, &mut rcc.apb1, &mut pwr);

    let rtc = Rtc::rtc(peripherals.RTC, &mut backup_domain);

    unsafe {
        NVIC::unmask(Interrupt::EXTI0);
        NVIC::unmask(Interrupt::EXTI1);
        NVIC::unmask(Interrupt::EXTI2);
    };

    let mut screen = Screen::new(
        ScreenConfig {
            d7: gpioa.pa9.into_push_pull_output(&mut gpioa.crh),
            d6: gpioa.pa8.into_push_pull_output(&mut gpioa.crh),
            d5: gpiob.pb15.into_push_pull_output(&mut gpiob.crh),
            d4: gpiob.pb14.into_push_pull_output(&mut gpiob.crh),
            en: gpiob.pb13.into_push_pull_output(&mut gpiob.crh),
            rs: gpiob.pb12.into_push_pull_output(&mut gpiob.crh),
        },
        &mut delay,
    )
    .ok()
    .unwrap();

    screen.update(Frame::Welcome, &mut delay).ok().unwrap();
    delay.delay_ms(1000u16);

    let mill = Mill::new(
        MillConfig {
            encoder: RotaryEncoder::new(sia, gpiob.pb1.into_pull_down_input(&mut gpiob.crl)),

            screen,

            motor: StepperMotor::new(StepperMotorConfig {
                dir: gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
                step: gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
                m2: gpioa.pa12.into_push_pull_output(&mut gpioa.crh),
                m1: gpioa.pa11.into_push_pull_output(&mut gpioa.crh),
                enable: gpioa.pa10.into_push_pull_output(&mut gpioa.crh),

                mode: MOTOR_MODE,
                signal_delay: Duration::Ms(SIGNAL_DELAY),
            })
            .ok()
            .unwrap(),

            limit_switch,
            home_switch,

            max_height: 48 * MM_STEPS,
            motor_steps_per_tick: STEPS_PER_LOOP,
            motor_steps_per_mm: MM_STEPS,
        },
        &mut delay,
    )
    .ok()
    .unwrap();

    interrupt_free(|cs| {
        MILL.borrow(cs).replace(Some(mill));
        DELAY.borrow(cs).replace(Some(delay));
        RTC.borrow(cs).replace(Some(rtc));
    });

    loop {
        interrupt_free(|cs| {
            let mut countdown = COUNTDOWN.borrow(cs).borrow_mut();
            if *countdown > 0 {
                *countdown -= 1;
                return;
            }

            let mut option = MILL.borrow(cs).borrow_mut();
            let mut delay = DELAY.borrow(cs).borrow_mut();
            let rtc = RTC.borrow(cs).borrow_mut();
            if let (Some(mill), Some(delay), Some(rtc)) =
                (option.as_mut(), delay.as_mut(), rtc.as_ref())
            {
                mill.tick(delay, rtc).ok().unwrap();
            }
        });
    }
}

#[interrupt]
fn EXTI0() {
    interrupt_free(|cs| {
        let mut countdown = COUNTDOWN.borrow(cs).borrow_mut();
        *countdown = COUNTDOWN_VALUE;

        let mut mill = MILL.borrow(cs).borrow_mut();
        let mut delay = DELAY.borrow(cs).borrow_mut();
        let mut rtc = RTC.borrow(cs).borrow_mut();
        if let (Some(mill), Some(delay), Some(rtc)) = (mill.as_mut(), delay.as_mut(), rtc.as_mut())
        {
            if !mill.encoder.sia.check_interrupt() {
                return;
            }

            mill.handle_sia_interrupt(delay, rtc).ok().unwrap();
            mill.encoder.sia.clear_interrupt_pending_bit();
        }
    });
}

#[interrupt]
fn EXTI1() {
    interrupt_free(|cs| {
        let mut mill = MILL.borrow(cs).borrow_mut();
        let mut delay = DELAY.borrow(cs).borrow_mut();
        if let (Some(mill), Some(delay)) = (mill.as_mut(), delay.as_mut()) {
            if !mill.home_switch.check_interrupt() {
                return;
            }

            mill.handle_home_switch_interrupt(delay).ok().unwrap();
            mill.home_switch.clear_interrupt_pending_bit();
        }
    })
}

#[interrupt]
fn EXTI2() {
    interrupt_free(|cs| {
        let mut mill = MILL.borrow(cs).borrow_mut();
        let mut delay = DELAY.borrow(cs).borrow_mut();
        if let (Some(mill), Some(delay)) = (mill.as_mut(), delay.as_mut()) {
            if !mill.limit_switch.check_interrupt() {
                return;
            }

            mill.handle_limit_switch_interrupt(delay).ok().unwrap();
            mill.limit_switch.clear_interrupt_pending_bit();
        }
    });
}
