use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::OutputPin,
};

#[derive(Debug)]
pub struct StepperMotor<S, DIR, EN, M1, M2, DUR>
where
    S: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    DUR: Copy,
{
    step: S,
    dir: DIR,
    enable: EN,
    m1: M1,
    m2: M2,

    signal_delay: Duration<DUR>,
    is_enabled: bool,
}

impl<STEP, DIR, EN, M1, M2, DUR> StepperMotor<STEP, DIR, EN, M1, M2, DUR>
where
    STEP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
    DUR: Copy,
{
    pub fn new(
        config: StepperMotorConfig<STEP, DIR, EN, M1, M2, DUR>,
    ) -> Result<Self, Error<STEP, DIR, EN, M1, M2>> {
        let mut motor = Self {
            step: config.step,
            dir: config.dir,
            enable: config.enable,
            m1: config.m1,
            m2: config.m2,

            signal_delay: config.signal_delay,
            is_enabled: false,
        };

        motor.set_mode(config.mode)?;
        motor.disable()?;

        Ok(motor)
    }

    pub fn set_mode(&mut self, mode: Mode) -> Result<&mut Self, Error<STEP, DIR, EN, M1, M2>> {
        match mode {
            Mode::FullStep => self.write_mode(false, false),
            Mode::HalfStep => self.write_mode(true, false),
            Mode::QuarterStep => self.write_mode(false, true),
        }
    }

    fn write_mode(
        &mut self,
        m1_state: bool,
        m2_state: bool,
    ) -> Result<&mut Self, Error<STEP, DIR, EN, M1, M2>> {
        if m1_state {
            self.m1.set_high().map_err(|err| Error::M1(err))?;
        } else {
            self.m1.set_low().map_err(|err| Error::M1(err))?;
        }

        if m2_state {
            self.m2.set_high().map_err(|err| Error::M2(err))?;
        } else {
            self.m2.set_low().map_err(|err| Error::M2(err))?;
        }

        Ok(self)
    }

    pub fn rotate_clockwise(
        &mut self,
        steps: u32,
        delay: &mut (impl DelayMs<DUR> + DelayUs<DUR>),
    ) -> Result<(), Error<STEP, DIR, EN, M1, M2>> {
        self.dir.set_high().map_err(|err| Error::Dir(err))?;
        self.rotate(steps, delay)
    }

    pub fn rotate_counter_clockwise(
        &mut self,
        steps: u32,
        delay: &mut (impl DelayMs<DUR> + DelayUs<DUR>),
    ) -> Result<(), Error<STEP, DIR, EN, M1, M2>> {
        self.dir.set_low().map_err(|err| Error::Dir(err))?;
        self.rotate(steps, delay)
    }

    pub fn enable(&mut self) -> Result<(), Error<STEP, DIR, EN, M1, M2>> {
        let result = self.enable.set_low();
        if result.is_ok() {
            self.is_enabled = true;
        }
        result.map_err(|err| Error::Enable(err))
    }

    pub fn is_enabled(&self) -> bool {
        self.is_enabled
    }

    pub fn disable(&mut self) -> Result<(), Error<STEP, DIR, EN, M1, M2>> {
        let result = self.enable.set_high();
        if result.is_ok() {
            self.is_enabled = false;
        }
        result.map_err(|err| Error::Enable(err))
    }

    fn rotate(
        &mut self,
        steps: u32,
        delay: &mut (impl DelayMs<DUR> + DelayUs<DUR>),
    ) -> Result<(), Error<STEP, DIR, EN, M1, M2>> {
        let was_enabled = self.is_enabled;

        if !was_enabled {
            self.enable()?;
        }

        for _ in 0..steps {
            self.step.set_high().map_err(|err| Error::Step(err))?;
            self.signal_delay.delay(delay);
            self.step.set_low().map_err(|err| Error::Step(err))?;
            self.signal_delay.delay(delay);
        }

        if !was_enabled {
            self.disable()?;
        }

        Ok(())
    }
}

#[derive(Debug)]
pub struct StepperMotorConfig<S, D, E, M1, M2, DUR>
where
    S: OutputPin,
    D: OutputPin,
    E: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
{
    pub step: S,
    pub dir: D,
    pub enable: E,
    pub m1: M1,
    pub m2: M2,
    pub mode: Mode,
    pub signal_delay: Duration<DUR>,
}

#[derive(Debug, Copy, Clone)]
pub enum Duration<T> {
    Us(T),
    Ms(T),
}

impl<T: Copy> Duration<T> {
    pub fn delay(self, delay: &mut (impl DelayMs<T> + DelayUs<T>)) {
        match self {
            Duration::Ms(dur) => delay.delay_ms(dur),
            Duration::Us(dur) => delay.delay_us(dur),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Mode {
    FullStep,
    HalfStep,
    QuarterStep,
}

#[derive(Debug)]
pub enum Error<S, D, E, M1, M2>
where
    S: OutputPin,
    D: OutputPin,
    E: OutputPin,
    M1: OutputPin,
    M2: OutputPin,
{
    Step(S::Error),
    Dir(D::Error),
    Enable(E::Error),
    M1(M1::Error),
    M2(M2::Error),
}
