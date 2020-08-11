use embedded_hal::digital::v2::InputPin;

pub struct RotaryEncoder<SIA: InputPin, SIB: InputPin> {
    pub sia: SIA,
    pub sib: SIB,
    current: Rotation,
}

impl<SIA: InputPin, SIB: InputPin> RotaryEncoder<SIA, SIB> {
    pub fn new(sia: SIA, sib: SIB) -> Self {
        Self {
            sia,
            sib,
            current: Rotation::None,
        }
    }

    pub fn update(&mut self) -> Result<Rotation, Error<SIA, SIB>> {
        let sia = self.sia.is_high().map_err(|err| Error::Sia(err))?;
        let sib = self.sib.is_high().map_err(|err| Error::Sib(err))?;

        if sia && self.current == Rotation::None {
            self.current = if sib {
                Rotation::CounterClockwise
            } else {
                Rotation::Clockwise
            };
            Ok(Rotation::None)
        } else if !sia {
            let rotation = self.current;
            self.current = Rotation::None;
            Ok(rotation)
        } else {
            Ok(Rotation::None)
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
