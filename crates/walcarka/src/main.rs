#![no_std]
#![no_main]

mod stepper_motor;

use panic_halt as _;

use arrayvec::ArrayString;
use core::{cell::RefCell, fmt::Write};
use cortex_m::interrupt::{free as interrupt_free, Mutex};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use hd44780_driver::{bus::FourBitBus, Cursor, CursorBlink, Display, DisplayMode, HD44780};
use stepper_motor::{Mode, StepperMotor, StepperMotorConfig};
use stm32f1xx_hal::{
    delay::Delay,
    device::interrupt,
    gpio::{
        gpioa::PA8,
        gpiob::{PB0, PB1, PB11, PB12, PB13, PB14, PB15},
        Edge, ExtiPin, Input, Output, PullUp, PushPull,
    },
    pac::{CorePeripherals, Interrupt, Peripherals, NVIC},
    prelude::*,
};

type LCD = HD44780<
    FourBitBus<
        PB11<Output<PushPull>>,
        PB12<Output<PushPull>>,
        PA8<Output<PushPull>>,
        PB15<Output<PushPull>>,
        PB14<Output<PushPull>>,
        PB13<Output<PushPull>>,
    >,
>;

static SIA: Mutex<RefCell<Option<PB0<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static SIB: Mutex<RefCell<Option<PB1<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static DISPLAY: Mutex<RefCell<Option<LCD>>> = Mutex::new(RefCell::new(None));
static DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static TARGET_HEIGHT: Mutex<RefCell<i32>> = Mutex::new(RefCell::new(4000));

#[entry]
fn main() -> ! {
    let core_peripherals = CorePeripherals::take().unwrap();
    let peripherals = Peripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);

    let mut sia = gpiob.pb0.into_pull_up_input(&mut gpiob.crl);
    sia.make_interrupt_source(&mut afio);
    sia.trigger_on_edge(&peripherals.EXTI, Edge::FALLING);
    sia.enable_interrupt(&peripherals.EXTI);

    let sib = gpiob.pb1.into_pull_up_input(&mut gpiob.crl);

    unsafe { NVIC::unmask(Interrupt::EXTI0) };

    let mut motor = StepperMotor::new(StepperMotorConfig {
        step: gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
        dir: gpiob.pb9.into_push_pull_output(&mut gpiob.crh),
        enable: gpiob.pb8.into_push_pull_output(&mut gpiob.crh),
        m1: gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
        m2: gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        m3: gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
        mode: Mode::FullStep,
    })
    .ok()
    .unwrap();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(core_peripherals.SYST, clocks);

    let mut lcd = HD44780::new_4bit(
        gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
        gpiob.pb12.into_push_pull_output(&mut gpiob.crh),
        gpioa.pa8.into_push_pull_output(&mut gpioa.crh),
        gpiob.pb15.into_push_pull_output(&mut gpiob.crh),
        gpiob.pb14.into_push_pull_output(&mut gpiob.crh),
        gpiob.pb13.into_push_pull_output(&mut gpiob.crh),
        &mut delay,
    )
    .ok()
    .unwrap();

    lcd.set_display_mode(
        DisplayMode {
            cursor_blink: CursorBlink::Off,
            cursor_visibility: Cursor::Invisible,
            display: Display::On,
        },
        &mut delay,
    )
    .ok()
    .unwrap();

    interrupt_free(|cs| {
        DISPLAY.borrow(cs).replace(Some(lcd));
        DELAY.borrow(cs).replace(Some(delay));
        SIA.borrow(cs).replace(Some(sia));
        SIB.borrow(cs).replace(Some(sib));
    });

    let mut current_height = 4000;
    write_height_to_lcd(current_height);

    loop {
        interrupt_free(|cs| {
            let target_height = *TARGET_HEIGHT.borrow(cs).borrow();

            let mut delay = DELAY.borrow(cs).borrow_mut();
            let delay = delay.as_mut().unwrap();

            if current_height < target_height {
                motor.rotate_clockwise(1, delay).ok().unwrap();
                current_height += 1;
            } else if current_height > target_height {
                motor.rotate_counter_clockwise(1, delay).ok().unwrap();
                current_height -= 1;
            }
        });
    }
}

#[interrupt]
fn EXTI0() {
    handle_encoder_exti(false);
}

#[interrupt]
fn EXTI1() {
    handle_encoder_exti(true);
}

fn handle_encoder_exti(check_sia: bool) {
    interrupt_free(|cs| {
        let mut sia = SIA.borrow(cs).borrow_mut();
        let sia = sia.as_mut().unwrap();

        let mut sib = SIB.borrow(cs).borrow_mut();
        let sib = sib.as_mut().unwrap();

        let mut target_height = TARGET_HEIGHT.borrow(cs).borrow_mut();

        let state = if check_sia {
            sia.is_high().unwrap()
        } else {
            sib.is_high().unwrap()
        };

        if state {
            *target_height += 200;
        } else {
            *target_height -= 200;
        }

        write_height_to_lcd(*target_height);

        sia.clear_interrupt_pending_bit();
    });
}

fn write_height_to_lcd(height: i32) {
    interrupt_free(|cs| {
        let mut lcd = DISPLAY.borrow(cs).borrow_mut();
        let lcd = lcd.as_mut().unwrap();

        let mut delay = DELAY.borrow(cs).borrow_mut();
        let delay = delay.as_mut().unwrap();

        let mut string = ArrayString::<[_; 32]>::new();
        write!(string, "{}", height / 200).unwrap();
        lcd.clear(delay).ok().unwrap();
        lcd.write_str(string.as_str(), delay).ok().unwrap();
    });
}
