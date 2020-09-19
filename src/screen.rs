use arrayvec::ArrayString;
use core::fmt;
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::OutputPin,
};
use fmt::Write;
use hd44780_driver::{
    bus::FourBitBus, error::Error as HD44780Error, Cursor, CursorBlink, Display, DisplayMode,
    HD44780,
};

pub struct Screen<RS, EN, D4, D5, D6, D7>
where
    RS: OutputPin,
    EN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    hd44780: HD44780<FourBitBus<RS, EN, D4, D5, D6, D7>>,
}

impl<RS, EN, D4, D5, D6, D7> Screen<RS, EN, D4, D5, D6, D7>
where
    RS: OutputPin,
    EN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    pub fn new(
        config: ScreenConfig<RS, EN, D4, D5, D6, D7>,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<Self, HD44780Error> {
        let ScreenConfig {
            rs,
            en,
            d4,
            d5,
            d6,
            d7,
            ..
        } = config;

        let mut hd44780 = HD44780::new_4bit(rs, en, d4, d5, d6, d7, delay)?;

        hd44780.set_display_mode(
            DisplayMode {
                cursor_blink: CursorBlink::Off,
                cursor_visibility: Cursor::Invisible,
                display: Display::On,
            },
            delay,
        )?;

        Ok(Self { hd44780 })
    }

    pub fn update(
        &mut self,
        frame: Frame,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<(), ScreenUpdateError> {
        self.hd44780.clear(delay)?;

        match frame {
            Frame::Height(height) => {
                self.hd44780.write_str("Obecna wysokosc:", delay)?;
                self.write_height_line(height, delay)?;
                Ok(())
            }
            Frame::Calibrating => {
                self.hd44780.write_str("Kalibracja...", delay)?;
                Ok(())
            }
        }
    }

    fn write_height_line(
        &mut self,
        height: u32,
        delay: &mut (impl DelayMs<u8> + DelayUs<u16>),
    ) -> Result<(), ScreenUpdateError> {
        let content = height_to_string(height)?;
        self.hd44780.set_cursor_pos(40 + 5, delay)?;
        self.hd44780.write_str(&content, delay)?;
        Ok(())
    }
}

fn height_to_string(height: u32) -> Result<ArrayString<[u8; 5]>, fmt::Error> {
    let mut content = ArrayString::<[_; 5]>::new();
    write!(content, "{:02}mm", height)?;

    Ok(content)
}

pub enum ScreenUpdateError {
    Fmt(fmt::Error),
    HD44870(HD44780Error),
}

impl From<fmt::Error> for ScreenUpdateError {
    fn from(err: fmt::Error) -> Self {
        Self::Fmt(err)
    }
}

impl From<HD44780Error> for ScreenUpdateError {
    fn from(err: HD44780Error) -> Self {
        Self::HD44870(err)
    }
}

pub struct ScreenConfig<RS, EN, D4, D5, D6, D7>
where
    RS: OutputPin,
    EN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
{
    pub rs: RS,
    pub en: EN,
    pub d4: D4,
    pub d5: D5,
    pub d6: D6,
    pub d7: D7,
}

pub enum Frame {
    Height(u32),
    Calibrating,
}
