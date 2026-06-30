use core::str::Utf8Error;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
    // invalid UTF-8.
    EncodingError,
    // что то моё
    FmtError,
    CmdFail,
    PTZWriteErr,
    PTZReadErr,
    PTZPortErr,
    PTZDataErr,
    Timeout,
    E220ReadErr,
    E220WriteErr,
    WrongResp,
    WrongCommand,
    AttemptsOvf,
}

impl From<Utf8Error> for Error {
    fn from(_: Utf8Error) -> Self {
        Error::EncodingError
    }
}

impl From<core::fmt::Error> for Error {
    fn from(_: core::fmt::Error) -> Self {
        Error::FmtError
    }
}
