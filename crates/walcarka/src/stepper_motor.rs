use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin};

#[derive(Debug)]
pub struct StepperMotor<S, D, E, M1, M2, M3>
where
    S: OutputPin,
    D: OutputPin,
    E: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    step: S,
    dir: D,
    enable: E,
    m1: M1,
    m2: M2,
    m3: M3,
}

impl<S, D, E, M1, M2, M3> StepperMotor<S, D, E, M1, M2, M3>
where
    S: OutputPin,
    D: OutputPin,
    E: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    pub fn new(config: StepperMotorConfig<S, D, E, M1, M2, M3>) -> Result<Self, Error<S, D, E, M1, M2, M3>> {
        let mut motor = Self {
            step: config.step,
            dir: config.dir,
            enable: config.enable,
            m1: config.m1,
            m2: config.m2,
            m3: config.m3,
        };

        motor.set_mode(config.mode)?;

        Ok(motor)
    }

    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<S, D, E, M1, M2, M3>> {
        match mode {
            Mode::FullStep => {
                self.m1.set_low().map_err(|err| Error::M1(err))?;
                self.m2.set_low().map_err(|err| Error::M2(err))?;
                self.m3.set_low().map_err(|err| Error::M3(err))
            }
            Mode::HalfStep => {
                self.m1.set_high().map_err(|err| Error::M1(err))?;
                self.m2.set_low().map_err(|err| Error::M2(err))?;
                self.m3.set_low().map_err(|err| Error::M3(err))
            }
            Mode::QuarterStep => {
                self.m1.set_low().map_err(|err| Error::M1(err))?;
                self.m2.set_high().map_err(|err| Error::M2(err))?;
                self.m3.set_low().map_err(|err| Error::M3(err))
            }
            Mode::EighthStep => {
                self.m1.set_high().map_err(|err| Error::M1(err))?;
                self.m2.set_high().map_err(|err| Error::M2(err))?;
                self.m3.set_low().map_err(|err| Error::M3(err))
            }
            Mode::SixteenthStep => {
                self.m1.set_high().map_err(|err| Error::M1(err))?;
                self.m2.set_high().map_err(|err| Error::M2(err))?;
                self.m3.set_high().map_err(|err| Error::M3(err))
            }
        }
    }

    pub fn rotate_clockwise(&mut self, steps: u32, delay: &mut impl DelayMs<u8>) -> Result<(), Error<S, D, E, M1, M2, M3>> {
        self.dir.set_high().map_err(|err| Error::Dir(err))?;
        self.rotate(steps, delay)
    }

    pub fn rotate_counter_clockwise(&mut self, steps: u32, delay: &mut impl DelayMs<u8>) -> Result<(), Error<S, D, E, M1, M2, M3>> {
        self.dir.set_low().map_err(|err| Error::Dir(err))?;
        self.rotate(steps, delay)
    }

    fn rotate(&mut self, steps: u32, delay: &mut impl DelayMs<u8>) -> Result<(), Error<S, D, E, M1, M2, M3>> {
        self.enable.set_low().map_err(|err| Error::Enable(err))?;

        for _ in 0..steps {
            self.step.set_high().map_err(|err| Error::Step(err))?;
            delay.delay_ms(1);
            self.step.set_low().map_err(|err| Error::Step(err))?;
            delay.delay_ms(1);
        }

        self.enable.set_high().map_err(|err| Error::Enable(err))
    }
}

#[derive(Debug)]
pub struct StepperMotorConfig<S, D, E, M1, M2, M3>
where
    S: OutputPin,
    D: OutputPin,
    E: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    pub step: S,
    pub dir: D,
    pub enable: E,
    pub m1: M1,
    pub m2: M2,
    pub m3: M3,
    pub mode: Mode,
}

#[derive(Debug, Copy, Clone)]
pub enum Mode {
    FullStep,
    HalfStep,
    QuarterStep,
    EighthStep,
    SixteenthStep,
}

#[derive(Debug)]
pub enum Error<S, D, E, M1, M2, M3>
where
    S: OutputPin,
    D: OutputPin,
    E: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    M3: OutputPin,
{
    Step(S::Error),
    Dir(D::Error),
    Enable(E::Error),
    M1(M1::Error),
    M2(M2::Error),
    M3(M3::Error),
}
