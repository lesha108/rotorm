// управление реле вкл

use bitflags::bitflags;

bitflags! {
    // состояние реле
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct SlaveRelay: u8 {
        const RELAY_PTZ = 0x01;
        const RELAY_LNA = 0x02;
        const RELAY_ALL = Self::RELAY_PTZ.bits() | Self::RELAY_LNA.bits();
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Relays(SlaveRelay);

impl Relays {
    pub const fn new() -> Self {
        Relays(SlaveRelay::empty())
    }

    pub fn ptz_on(&mut self) {
        self.0 |= SlaveRelay::RELAY_PTZ;
    }

    pub fn lna_on(&mut self) {
        self.0 |= SlaveRelay::RELAY_LNA;
    }

    pub fn all_on(&mut self) {
        self.0 |= SlaveRelay::RELAY_ALL;
    }

    pub fn ptz_off(&mut self) {
        self.0 &= !SlaveRelay::RELAY_PTZ;
    }

    pub fn lna_off(&mut self) {
        self.0 &= !SlaveRelay::RELAY_LNA;
    }

    pub fn all_off(&mut self) {
        self.0 = SlaveRelay::empty();
    }

    pub fn is_ptz_on(&mut self) -> bool {
        if (self.0 & SlaveRelay::RELAY_PTZ) != SlaveRelay::empty() {
            true
        } else {
            false
        }
    }

    pub fn is_lna_on(&mut self) -> bool {
        if (self.0 & SlaveRelay::RELAY_LNA) != SlaveRelay::empty() {
            true
        } else {
            false
        }
    }

}

impl From<&Relays> for u8 {
    fn from(val: &Relays) -> u8 {
        val.0.bits()
    }
}

impl TryFrom<u8> for Relays {
    type Error = ();

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        {
            let masked = val & 0b00000011;
            match masked {
                0x00 => Ok(Relays(SlaveRelay::empty())),
                0x01 => Ok(Relays(SlaveRelay::RELAY_PTZ)),
                0x02 => Ok(Relays(SlaveRelay::RELAY_LNA)),
                0x03 => Ok(Relays(SlaveRelay::RELAY_ALL)),
                _ => Err(()),
            }
        }
    }
}
