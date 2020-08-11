#![no_std]

pub mod rotary_encoder;
pub mod screen;
pub mod stepper_motor;

use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::{InputPin, OutputPin},
};
use rotary_encoder::{RotaryEncoder, Rotation};
use screen::{Frame, Screen, ScreenUpdateError};
use stepper_motor::StepperMotor;

pub struct Mill<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3, DUR, RS, SEN, D4, D5, D6, D7>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    HOM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    MEN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
    DUR: Copy,
    RS: OutputPin,
    SEN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    encoder: RotaryEncoder<SIA, SIB>,
    motor: StepperMotor<STP, DIR, MEN, M1, M2, M3, DUR>,
    screen: Screen<RS, SEN, D4, D5, D6, D7>,
    limit_switch: LIM,
    home_switch: HOM,

    target_height: u32,
    current_height: Option<u32>,

    motor_steps_per_tick: u32,
    motor_steps_per_mm: u32,
    max_height: u32,
}

impl<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3, DUR, RS, SEN, D4, D5, D6, D7>
    Mill<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3, DUR, RS, SEN, D4, D5, D6, D7>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    HOM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    MEN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
    DUR: Copy,
    RS: OutputPin,
    SEN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    pub fn new<DEL: DelayMs<u8> + DelayUs<u16>>(
        config: MillConfig<
            SIA,
            SIB,
            LIM,
            HOM,
            STP,
            DIR,
            MEN,
            M1,
            M2,
            M3,
            DUR,
            RS,
            SEN,
            D4,
            D5,
            D6,
            D7,
        >,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<Self, Error<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3>> {
        let MillConfig {
            encoder,
            motor,
            screen,
            home_switch,
            limit_switch,

            max_height,
            motor_steps_per_mm,
            motor_steps_per_tick,
            ..
        } = config;

        let mut mill = Self {
            encoder,
            motor,
            screen,
            home_switch,
            limit_switch,

            current_height: None,
            target_height: 20,

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
    ) -> Result<(), Error<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3>> {
        if let Some(current_height) = self.current_height {
            if current_height > self.target_height {
                self.motor
                    .rotate_counter_clockwise(self.motor_steps_per_tick, delay)?;
                self.current_height
                    .replace(current_height + self.motor_steps_per_tick);
            } else if current_height < self.target_height {
                self.motor
                    .rotate_clockwise(self.motor_steps_per_tick, delay)?;
                self.current_height
                    .replace(current_height - self.motor_steps_per_tick);
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

    pub fn handle_sia_rising(
        &mut self,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<(), Error<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3>> {
        self.encoder.update()?;
        Ok(())
    }

    pub fn handle_sia_falling(
        &mut self,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<(), Error<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3>> {
        match self.encoder.update()? {
            Rotation::Clockwise => self.target_height += self.motor_steps_per_mm,
            Rotation::CounterClockwise => self.target_height -= self.motor_steps_per_mm,
            _ => {}
        }
        self.update_screen(delay)
    }

    pub fn handle_home_switch_interrupt(
        &mut self,
        delay: &mut (impl DelayUs<u16> + DelayMs<u8>),
    ) -> Result<(), Error<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3>> {
        self.current_height = None;
        self.update_screen(delay)
    }

    fn update_screen(
        &mut self,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<(), Error<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3>> {
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

pub struct MillConfig<SIA, SIB, LIM, HOM, STP, DIR, MEN, M1, M2, M3, DUR, RS, SEN, D4, D5, D6, D7>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    HOM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    MEN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
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
    pub motor: StepperMotor<STP, DIR, MEN, M1, M2, M3, DUR>,
    pub limit_switch: LIM,
    pub home_switch: HOM,

    pub max_height: u32,
    pub motor_steps_per_tick: u32,
    pub motor_steps_per_mm: u32,
}

pub enum Error<SIA, SIB, LIM, HOM, STP, DIR, EN, M1, M2, M3>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    HOM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    Encoder(rotary_encoder::Error<SIA, SIB>),
    LimitSwitch(LIM::Error),
    HomeSwitch(HOM::Error),
    Motor(stepper_motor::Error<STP, DIR, EN, M1, M2, M3>),
    ScreenUpdate(ScreenUpdateError),
}

impl<SIA, SIB, LIM, HOM, STP, DIR, EN, M1, M2, M3> From<rotary_encoder::Error<SIA, SIB>>
    for Error<SIA, SIB, LIM, HOM, STP, DIR, EN, M1, M2, M3>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    HOM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    fn from(err: rotary_encoder::Error<SIA, SIB>) -> Self {
        Self::Encoder(err)
    }
}

impl<SIA, SIB, LIM, HOM, STP, DIR, EN, M1, M2, M3> From<ScreenUpdateError>
    for Error<SIA, SIB, LIM, HOM, STP, DIR, EN, M1, M2, M3>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    HOM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    fn from(err: ScreenUpdateError) -> Self {
        Self::ScreenUpdate(err)
    }
}

impl<SIA, SIB, LIM, HOM, STP, DIR, EN, M1, M2, M3>
    From<stepper_motor::Error<STP, DIR, EN, M1, M2, M3>>
    for Error<SIA, SIB, LIM, HOM, STP, DIR, EN, M1, M2, M3>
where
    SIA: InputPin,
    SIB: InputPin,
    LIM: InputPin,
    HOM: InputPin,
    STP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    fn from(err: stepper_motor::Error<STP, DIR, EN, M1, M2, M3>) -> Self {
        Self::Motor(err)
    }
}
