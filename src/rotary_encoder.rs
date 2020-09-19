use embedded_hal::digital::v2::InputPin;

pub struct RotaryEncoder<SIA: InputPin, SIB: InputPin> {
    pub sia: SIA,
    sib: SIB,
}

impl<SIA: InputPin, SIB: InputPin> RotaryEncoder<SIA, SIB> {
    pub fn new(sia: SIA, sib: SIB) -> Self {
        Self { sia, sib }
    }

    pub fn update(&mut self) -> Result<Rotation, Error<SIA, SIB>> {
        let sib = self.sib.is_high().map_err(|err| Error::Sib(err))?;

        if sib {
            Ok(Rotation::Clockwise)
        } else {
            Ok(Rotation::CounterClockwise)
        }
    }
}

#[derive(PartialEq, Copy, Clone)]
pub enum Rotation {
    None,
    Clockwise,
    CounterClockwise,
}

pub enum Error<SIA: InputPin, SIB: InputPin> {
    Sia(SIA::Error),
    Sib(SIB::Error),
}
