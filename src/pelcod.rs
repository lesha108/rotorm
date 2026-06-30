// Реализация подмножества протокола pelco-d достаточного для работы поворотки

use bitflags::bitflags;

// длина сообщения команды
pub const COMMAND_SIZE: usize = 7;
pub const SYNC_BYTE: u8 = 0xFF;

bitflags! {
    // Bitflag for generating the "command2" word of the message.
    pub struct Command2: u8 {
        const DOWN = 0x10;
        const UP = 0x08;
        const LEFT = 0x04;
        const RIGHT = 0x02;
        const QUERY_PAN = 0x51;
        const QUERY_TILT = 0x53;
        const SET_PAN = 0x4B;
        const SET_TILT = 0x4D;
    }
}

bitflags! {
    // position response
    pub struct QResponse: u8 {
        const QUERY_PAN_RESPONSE = 0x59;
        const QUERY_TILT_RESPONSE = 0x5B;
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Command([u8; COMMAND_SIZE]);

impl Command {
    fn new(address: u8, cmd2: Command2, data1: u8, data2: u8) -> Command {
        let mut msg = Command([SYNC_BYTE, address, 0x00, cmd2.bits(), data1, data2, 0]);
        msg.fill_checksum();
        msg
    }

    // заполнение байта контрольной суммы команды
    fn fill_checksum(&mut self) {
        self.0[COMMAND_SIZE - 1] = checksum(&self.0[1..COMMAND_SIZE - 1]);
    }

    pub fn pan_left(addr: u8, speed: &PelcoSpeed) -> Command {
        let mut cmd = Command::new(addr, Command2::LEFT, speed.into(), 0);
        cmd
    }

    pub fn pan_right(addr: u8, speed: &PelcoSpeed) -> Command {
        let mut cmd = Command::new(addr, Command2::RIGHT, speed.into(), 0);
        cmd
    }

    pub fn pan_stop(addr: u8) -> Command {
        let mut cmd = Command::new(addr, Command2::empty(), 0x12, 0);
        cmd
    }

    pub fn tilt_up(addr: u8, speed: &PelcoSpeed) -> Command {
        let mut cmd = Command::new(addr, Command2::UP, 0, speed.into());
        cmd
    }

    pub fn tilt_down(addr: u8, speed: &PelcoSpeed) -> Command {
        let mut cmd = Command::new(addr, Command2::DOWN, 0, speed.into());
        cmd
    }

    pub fn tilt_stop(addr: u8) -> Command {
        let mut cmd = Command::new(addr, Command2::empty(), 0, 0x12);
        cmd
    }

    pub fn pan_right_tilt_up(addr: u8, pan_speed: &PelcoSpeed, tilt_speed: &PelcoSpeed) -> Command {
        let mut cmd = Command::new(
            addr,
            Command2::RIGHT | Command2::UP,
            pan_speed.into(),
            tilt_speed.into(),
        );
        cmd
    }

    pub fn pan_right_tilt_down(
        addr: u8,
        pan_speed: &PelcoSpeed,
        tilt_speed: &PelcoSpeed,
    ) -> Command {
        let mut cmd = Command::new(
            addr,
            Command2::RIGHT | Command2::DOWN,
            pan_speed.into(),
            tilt_speed.into(),
        );
        cmd
    }

    pub fn pan_left_tilt_up(addr: u8, pan_speed: &PelcoSpeed, tilt_speed: &PelcoSpeed) -> Command {
        let mut cmd = Command::new(
            addr,
            Command2::LEFT | Command2::UP,
            pan_speed.into(),
            tilt_speed.into(),
        );
        cmd
    }

    pub fn pan_left_tilt_down(
        addr: u8,
        pan_speed: &PelcoSpeed,
        tilt_speed: &PelcoSpeed,
    ) -> Command {
        let mut cmd = Command::new(
            addr,
            Command2::LEFT | Command2::DOWN,
            pan_speed.into(),
            tilt_speed.into(),
        );
        cmd
    }

    pub fn query_pan_position(addr: u8) -> Command {
        let mut cmd = Command::new(addr, Command2::QUERY_PAN, 0, 0);
        cmd
    }

    pub fn query_tilt_position(addr: u8) -> Command {
        let mut cmd = Command::new(addr, Command2::QUERY_TILT, 0, 0);
        cmd
    }

    pub fn set_pan(addr: u8, ang: &AzAngle) -> Command {
        let data: u16 = ang.into();
        let data1 = (data >> 8) as u8;
        let data2 = (data & 0xFF) as u8;
        let mut cmd = Command::new(addr, Command2::SET_PAN, data1, data2);
        cmd
    }

    pub fn set_tilt(addr: u8, ang: &ElAngle) -> Command {
        let data: u16 = ang.into();
        let data1 = (data >> 8) as u8;
        let data2 = (data & 0xFF) as u8;
        let mut cmd = Command::new(addr, Command2::SET_TILT, data1, data2);
        cmd
    }
}

impl AsRef<[u8]> for Command {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

// Checksum algorithm used by Pelco D.
pub fn checksum(data: &[u8]) -> u8 {
    let s: u32 = data.iter().map(|&b| u32::from(b)).sum();
    (s & 0xff) as u8
}

pub const MAX_SPEED: u8 = 0x3F;

// pelco speed 0..0x3F
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PelcoSpeed(u8);

impl Default for PelcoSpeed {
    fn default() -> Self {
        PelcoSpeed(0)
    }
}

impl PelcoSpeed {
    pub fn half(&self) -> Self {
        let half = self.0 >> 1;
        PelcoSpeed(half)
    }
}

impl TryFrom<u8> for PelcoSpeed {
    type Error = ();

    fn try_from(_val: u8) -> Result<Self, Self::Error> {
        if _val <= MAX_SPEED {
            Ok(PelcoSpeed(_val))
        } else {
            Err(())
        }
    }
}

impl From<&PelcoSpeed> for u8 {
    fn from(_val: &PelcoSpeed) -> u8 {
        _val.0
    }
}

impl core::fmt::Display for PelcoSpeed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{0:.1}", self.0)
    }
}

impl AsRef<u8> for PelcoSpeed {
    fn as_ref(&self) -> &u8 {
        &self.0
    }
}

pub const MAX_AZ: u16 = 36000;
pub const MAX_EL: u16 = 9000;

// azimuth and elevation angles
// azimuth range 0..360
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct AzAngle(pub u16);

impl Default for AzAngle {
    fn default() -> Self {
        AzAngle(0)
    }
}

impl TryFrom<u16> for AzAngle {
    type Error = ();

    fn try_from(_val: u16) -> Result<Self, Self::Error> {
        if _val <= MAX_AZ {
            Ok(AzAngle(_val))
        } else {
            Err(())
        }
    }
}

impl From<&AzAngle> for u16 {
    fn from(_val: &AzAngle) -> u16 {
        _val.0
    }
}

impl core::fmt::Display for AzAngle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:03}", self.0 / 100)
    }
}

impl AsRef<u16> for AzAngle {
    fn as_ref(&self) -> &u16 {
        &self.0
    }
}

// elevation range 0..90
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct ElAngle(pub u16);

impl Default for ElAngle {
    fn default() -> Self {
        ElAngle(0)
    }
}

impl TryFrom<u16> for ElAngle {
    type Error = ();

    fn try_from(_val: u16) -> Result<Self, Self::Error> {
        if _val <= MAX_EL {
            Ok(ElAngle(_val))
        } else {
            Err(())
        }
    }
}

impl From<&ElAngle> for u16 {
    fn from(_val: &ElAngle) -> u16 {
        _val.0
    }
}

impl core::fmt::Display for ElAngle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:02}", self.0 / 100)
    }
}

impl AsRef<u16> for ElAngle {
    fn as_ref(&self) -> &u16 {
        &self.0
    }
}

