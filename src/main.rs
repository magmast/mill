#![no_std]
#![no_main]

use panic_semihosting as _;

use core::cell::RefCell;
use cortex_m::interrupt::{free as interrupt_free, Mutex};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use mill::{
    stepper_motor::{Duration, Mode, StepperMotorConfig},
    Mill,
};
use stm32f1xx_hal::{
    delay::Delay,
    device::interrupt,
    gpio::{
        gpioa::{PA2, PA3, PA8},
        gpiob::{PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB5, PB6, PB7, PB8, PB9},
        Edge, ExtiPin, Input, Output, PullDown, PullUp, PushPull,
    },
    pac::{CorePeripherals, Interrupt, Peripherals, NVIC},
    prelude::*,
};

const MOTOR_MODE: Mode = Mode::FullStep;
const MM_STEPS: u32 = 200;
const STEPS_PER_LOOP: u32 = 1;
const SIGNAL_DELAY: u8 = 1;

static DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static WALCARKA: Mutex<
    RefCell<
        Option<
            Mill<
                PB0<Input<PullDown>>,
                PB1<Input<PullDown>>,
                PA2<Input<PullUp>>,
                PA3<Input<PullUp>>,
                PB7<Output<PushPull>>,
                PB9<Output<PushPull>>,
                PB8<Output<PushPull>>,
                PB6<Output<PushPull>>,
                PB5<Output<PushPull>>,
                PB10<Output<PushPull>>,
                u8,
                PB11<Output<PushPull>>,
                PB12<Output<PushPull>>,
                PA8<Output<PushPull>>,
                PB15<Output<PushPull>>,
                PB14<Output<PushPull>>,
                PB13<Output<PushPull>>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let core_peripherals = CorePeripherals::take().unwrap();
    let peripherals = Peripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(core_peripherals.SYST, clocks);

    let mut sia = gpiob.pb0.into_pull_down_input(&mut gpiob.crl);
    sia.make_interrupt_source(&mut afio);
    sia.trigger_on_edge(&peripherals.EXTI, Edge::RISING);
    sia.enable_interrupt(&peripherals.EXTI);

    unsafe {
        NVIC::unmask(Interrupt::EXTI0);
    };

    let walcarka = Mill::new(WalcarkaConfig {
        limit_switch: gpioa.pa2.into_pull_up_input(&mut gpioa.crl),
        sib: gpiob.pb1.into_pull_down_input(&mut gpiob.crl),
        home_switch: gpioa.pa3.into_pull_up_input(&mut gpioa.crl),
        motor_config: StepperMotorConfig {
            step: gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
            dir: gpiob.pb9.into_push_pull_output(&mut gpiob.crh),
            enable: gpiob.pb8.into_push_pull_output(&mut gpiob.crh),
            m1: gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
            m2: gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
            m3: gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
            mode: MOTOR_MODE,
            signal_delay: Duration::Ms(SIGNAL_DELAY),
        },
        motor_steps_per_mm: MM_STEPS,
        motor_steps_per_tick: STEPS_PER_LOOP,
        max_height: 45,
        screen_config: ScreenConfig {
            rs: gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
            en: gpiob.pb12.into_push_pull_output(&mut gpiob.crh),
            d4: gpioa.pa8.into_push_pull_output(&mut gpioa.crh),
            d5: gpiob.pb15.into_push_pull_output(&mut gpiob.crh),
            d6: gpiob.pb14.into_push_pull_output(&mut gpiob.crh),
            d7: gpiob.pb13.into_push_pull_output(&mut gpiob.crh),
            delay: &mut delay,
        },
    })
    .ok()
    .unwrap();

    interrupt_free(|cs| {
        WALCARKA.borrow(cs).replace(Some(walcarka));
        DELAY.borrow(cs).replace(Some(delay));
    });

    loop {
        interrupt_free(|cs| {
            let mut option = WALCARKA.borrow(cs).borrow_mut();
            let mut delay = DELAY.borrow(cs).borrow_mut();
            if let (Some(walcarka), Some(delay)) = (option.as_mut(), delay.as_mut()) {
                walcarka.tick(delay).ok().unwrap();
            }
        });
    }
}

#[interrupt]
fn EXTI0() {
    interrupt_free(|cs| {
        hprintln!("Handling EXTI0").unwrap();

        let mut option = WALCARKA.borrow(cs).borrow_mut();
        let mut delay = DELAY.borrow(cs).borrow_mut();
        if let (Some(walcarka), Some(delay)) = (option.as_mut(), delay.as_mut()) {
            walcarka.handle_sia_interrupt(delay).ok().unwrap();
            walcarka.sia.clear_interrupt_pending_bit();
        }
    });
}
