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
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{
        gpioa::{PA1, PA10, PA11, PA12, PA2, PA8, PA9},
        gpiob::{PB0, PB1, PB12, PB13, PB14, PB15, PB6, PB7},
        Edge, ExtiPin, Input, Output, PullDown, PushPull,
    },
    interrupt,
    pac::{CorePeripherals, Interrupt, Peripherals, NVIC},
    prelude::*,
    rtc::Rtc,
};

// If you change this, you should propably change `MM_STEPS` too.
const MOTOR_MODE: Mode = Mode::FullStep;

// How many steps is one milimeter.
const MM_STEPS: u32 = 200;

// Main loop doesn't rotate motor to the specified height at once, as rotation
// executes inside `interrupt_free` block, so it would block any user
// interaction until motor stops rotating and it would be frustrating. So
// it rotates motor just a bit and exists `interrupt_free` block so user can
// interrupt motor rotation. This variable defines how many steps motor rotates
// per `interrupt_free` block.
const STEPS_PER_LOOP: u32 = 1;

// Interval between signals send to the stepper motor driver.
const SIGNAL_DELAY: u8 = 1;

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
static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut peripherals = Peripherals::take().unwrap();

    let rcc = peripherals.RCC.constrain();
    let mut syscfg = peripherals.SYSCFG.constrain();
    let gpioa = peripherals.GPIOA.split();
    let gpiob = peripherals.GPIOB.split();
    let clocks = rcc.cfgr.freeze();
    let mut delay = Delay::new(core_peripherals.SYST, clocks);

    let mut sia = gpiob.pb0.into_pull_down_input();
    sia.make_interrupt_source(&mut syscfg);
    sia.trigger_on_edge(&mut peripherals.EXTI, Edge::FALLING);
    sia.enable_interrupt(&mut peripherals.EXTI);

    let mut home_switch = gpioa.pa1.into_pull_down_input();
    home_switch.make_interrupt_source(&mut syscfg);
    home_switch.trigger_on_edge(&mut peripherals.EXTI, Edge::FALLING);
    home_switch.enable_interrupt(&mut peripherals.EXTI);

    let mut limit_switch = gpioa.pa2.into_pull_down_input();
    limit_switch.make_interrupt_source(&mut syscfg);
    limit_switch.trigger_on_edge(&mut peripherals.EXTI, Edge::RISING);
    limit_switch.enable_interrupt(&mut peripherals.EXTI);

    unsafe {
        NVIC::unmask(Interrupt::EXTI0);
        NVIC::unmask(Interrupt::EXTI1);
        NVIC::unmask(Interrupt::EXTI2);
    };

    let rtc = Rtc::new(peripherals.RTC, 255, 127, false, &mut peripherals.PWR);

    let mut screen = Screen::new(
        ScreenConfig {
            d7: gpioa.pa9.into_push_pull_output(),
            d6: gpioa.pa8.into_push_pull_output(),
            d5: gpiob.pb15.into_push_pull_output(),
            d4: gpiob.pb14.into_push_pull_output(),
            en: gpiob.pb13.into_push_pull_output(),
            rs: gpiob.pb12.into_push_pull_output(),
        },
        &mut delay,
    )
    .ok()
    .unwrap();

    screen.update(Frame::Welcome, &mut delay).ok().unwrap();
    delay.delay_ms(5000u16);

    let mill = Mill::new(
        MillConfig {
            encoder: RotaryEncoder::new(sia, gpiob.pb1.into_pull_down_input()),

            screen,

            motor: StepperMotor::new(StepperMotorConfig {
                dir: gpiob.pb7.into_push_pull_output(),
                step: gpiob.pb6.into_push_pull_output(),
                m2: gpioa.pa12.into_push_pull_output(),
                m1: gpioa.pa11.into_push_pull_output(),
                enable: gpioa.pa10.into_push_pull_output(),

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
            let mut option = MILL.borrow(cs).borrow_mut();
            let mut delay = DELAY.borrow(cs).borrow_mut();
            let mut rtc = RTC.borrow(cs).borrow_mut();
            if let (Some(mill), Some(delay), Some(rtc)) =
                (option.as_mut(), delay.as_mut(), rtc.as_mut())
            {
                mill.tick(delay, rtc).ok().unwrap();
            }
        });
    }
}

#[interrupt]
fn EXTI0() {
    interrupt_free(|cs| {
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
