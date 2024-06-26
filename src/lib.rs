#![no_std]

pub mod rotary_encoder;
pub mod screen;
pub mod stepper_motor;

use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::{InputPin, OutputPin},
};
use rotary_encoder::{RotaryEncoder, Rotation};
use rtcc::Rtcc;
use screen::{Frame, Screen, ScreenUpdateError};
use stepper_motor::StepperMotor;

pub struct Mill<SIA, SIB, HOM, LIM, STP, DIR, MEN, M1, M2, DUR, RS, SEN, D4, D5, D6, D7>
where
    SIA: InputPin,
    SIB: InputPin,
    HOM: InputPin,
    LIM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    MEN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    DUR: Copy,
    RS: OutputPin,
    SEN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    pub encoder: RotaryEncoder<SIA, SIB>,
    motor: StepperMotor<STP, DIR, MEN, M1, M2, DUR>,
    screen: Screen<RS, SEN, D4, D5, D6, D7>,
    pub limit_switch: LIM,
    pub home_switch: HOM,

    target_height: u32,
    current_height: Option<u32>,

    motor_steps_per_tick: u32,
    motor_steps_per_mm: u32,
    max_height: u32,
}

impl<SIA, SIB, HOM, LIM, STP, DIR, MEN, M1, M2, DUR, RS, SEN, D4, D5, D6, D7>
    Mill<SIA, SIB, HOM, LIM, STP, DIR, MEN, M1, M2, DUR, RS, SEN, D4, D5, D6, D7>
where
    SIA: InputPin,
    SIB: InputPin,
    HOM: InputPin,
    LIM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    MEN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    DUR: Copy,
    RS: OutputPin,
    SEN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    pub fn new(
        config: MillConfig<SIA, SIB, HOM, LIM, STP, DIR, MEN, M1, M2, DUR, RS, SEN, D4, D5, D6, D7>,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<Self, Error<SIA, SIB, LIM, STP, DIR, MEN, M1, M2>> {
        let MillConfig {
            encoder,
            motor,
            screen,
            limit_switch,
            home_switch,

            max_height,
            motor_steps_per_mm,
            motor_steps_per_tick,
            ..
        } = config;

        let mut mill = Self {
            encoder,
            motor,
            screen,
            limit_switch,
            home_switch,

            current_height: None,
            target_height: 0,

            max_height,
            motor_steps_per_mm,
            motor_steps_per_tick,
        };

        mill.screen.update(Frame::Calibrating, delay)?;

        Ok(mill)
    }

    pub fn tick(
        &mut self,
        delay: &mut (impl DelayMs<DUR> + DelayUs<DUR> + DelayMs<u8> + DelayUs<u16>),
        rtc: &mut impl Rtcc,
    ) -> Result<(), Error<SIA, SIB, LIM, STP, DIR, MEN, M1, M2>> {
        if let Some(current_height) = self.current_height {
            if rtc
                .get_seconds()
                .map(|seconds| seconds < 1)
                .unwrap_or(false)
            {
                return Ok(());
            }

            if current_height > self.target_height {
                self.motor
                    .rotate_counter_clockwise(self.motor_steps_per_tick, delay)?;
                self.current_height
                    .replace(current_height - self.motor_steps_per_tick);
            } else if current_height < self.target_height {
                self.motor
                    .rotate_clockwise(self.motor_steps_per_tick, delay)?;
                self.current_height
                    .replace(current_height + self.motor_steps_per_tick);
            }
        } else {
            self.motor
                .rotate_counter_clockwise(self.motor_steps_per_tick, delay)?;
            if self
                .limit_switch
                .is_low()
                .map_err(|err| Error::LimitSwitch(err))?
            {
                self.current_height.replace(0);
                self.update_screen(delay)?;
            }
        }

        Ok(())
    }

    pub fn handle_sia_interrupt(
        &mut self,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
        rtc: &mut impl Rtcc,
    ) -> Result<(), Error<SIA, SIB, LIM, STP, DIR, MEN, M1, M2>> {
        match self.encoder.update()? {
            Rotation::Clockwise => {
                self.target_height += self.motor_steps_per_mm;
                if self.target_height > self.max_height {
                    self.target_height = self.max_height;
                }
            }
            Rotation::CounterClockwise => {
                self.target_height = self
                    .target_height
                    .checked_sub(self.motor_steps_per_mm)
                    .unwrap_or(0)
            }
            _ => {}
        }
        rtc.set_seconds(0);
        self.update_screen(delay)
    }

    pub fn handle_home_switch_interrupt(
        &mut self,
        delay: &mut (impl DelayUs<u16> + DelayMs<u8>),
    ) -> Result<(), Error<SIA, SIB, LIM, STP, DIR, MEN, M1, M2>> {
        self.current_height = None;
        self.update_screen(delay)
    }

    pub fn handle_limit_switch_interrupt(
        &mut self,
        delay: &mut (impl DelayUs<u16> + DelayMs<u8> + DelayUs<DUR> + DelayMs<DUR>),
    ) -> Result<(), Error<SIA, SIB, LIM, STP, DIR, MEN, M1, M2>> {
        self.current_height = Some(self.motor_steps_per_mm);
        self.target_height = self.motor_steps_per_mm;
        self.motor
            .rotate_clockwise(self.motor_steps_per_mm, delay)?;
        self.update_screen(delay)
    }

    fn update_screen(
        &mut self,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<(), Error<SIA, SIB, LIM, STP, DIR, MEN, M1, M2>> {
        if let Some(_) = self.current_height {
            self.screen.update(
                Frame::Height(self.target_height / self.motor_steps_per_mm),
                delay,
            )?;
        } else {
            self.screen.update(Frame::Calibrating, delay)?;
        }

        Ok(())
    }
}

pub struct MillConfig<SIA, SIB, HOM, LIM, STP, DIR, MEN, M1, M2, DUR, RS, SEN, D4, D5, D6, D7>
where
    SIA: InputPin,
    SIB: InputPin,
    HOM: InputPin,
    LIM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    MEN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    DUR: Copy,
    RS: OutputPin,
    SEN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    pub encoder: RotaryEncoder<SIA, SIB>,
    pub screen: Screen<RS, SEN, D4, D5, D6, D7>,
    pub motor: StepperMotor<STP, DIR, MEN, M1, M2, DUR>,
    pub home_switch: HOM,
    pub limit_switch: LIM,

    pub max_height: u32,
    pub motor_steps_per_tick: u32,
    pub motor_steps_per_mm: u32,
}

pub enum Error<SIA, SIB, LIM, STP, DIR, EN, M1, M2>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
{
    Encoder(rotary_encoder::Error<SIA, SIB>),
    LimitSwitch(LIM::Error),
    Motor(stepper_motor::Error<STP, DIR, EN, M1, M2>),
    ScreenUpdate(ScreenUpdateError),
    Sia(SIA::Error),
}

impl<SIA, SIB, LIM, STP, DIR, EN, M1, M2> From<rotary_encoder::Error<SIA, SIB>>
    for Error<SIA, SIB, LIM, STP, DIR, EN, M1, M2>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
{
    fn from(err: rotary_encoder::Error<SIA, SIB>) -> Self {
        Self::Encoder(err)
    }
}

impl<SIA, SIB, LIM, STP, DIR, EN, M1, M2> From<ScreenUpdateError>
    for Error<SIA, SIB, LIM, STP, DIR, EN, M1, M2>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
{
    fn from(err: ScreenUpdateError) -> Self {
        Self::ScreenUpdate(err)
    }
}

impl<SIA, SIB, LIM, STP, DIR, EN, M1, M2> From<stepper_motor::Error<STP, DIR, EN, M1, M2>>
    for Error<SIA, SIB, LIM, STP, DIR, EN, M1, M2>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
{
    fn from(err: stepper_motor::Error<STP, DIR, EN, M1, M2>) -> Self {
        Self::Motor(err)
    }
}
