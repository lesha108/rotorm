// Функции работы с дисплеем

use embedded_graphics::{
    //mono_font::{MonoTextStyle, ascii::FONT_6X12},
    pixelcolor::{Rgb565, raw::ToBytes},
    prelude::*,
    primitives::Rectangle,
    //text::Text,
};

use sky_ili9341::{AsyncDisplay, AsyncInterface};

use static_cell::ConstStaticCell;

//const FRAME_BUFFER_SIZE: usize = 32 * 40 * 2;
const FRAME_BUFFER_SIZE: usize = 32 * 40 * 2 * 4; // ???
pub static PIXEL_DATA: ConstStaticCell<[u8; FRAME_BUFFER_SIZE]> =
    ConstStaticCell::new([0; FRAME_BUFFER_SIZE]);

pub struct TextRenderer {
    data: &'static [u8],
    char_size: Size,
    char_byte_count: usize,
}

impl TextRenderer {
    pub fn new(data: &'static [u8], char_size: Size) -> Self {
        let char_bit_count = char_size.width as usize * char_size.height as usize;
        //assert!(char_bit_count % 8 == 0);
        Self {
            data,
            char_size,
            char_byte_count: char_bit_count / 8,
        }
    }

    fn unpack(&self, c: char, buf: &mut [u8], fc: &[u8], bc: &[u8]) {
        //assert!(fc.len() == bc.len());
        let color_len = fc.len();
        let idx = c as usize - ' ' as usize;
        let start = idx * self.char_byte_count;
        let mut i = 0;
        for b in &self.data[start..start + self.char_byte_count] {
            let mut code = *b;
            for _ in 0..8 {
                buf[i..i + color_len].copy_from_slice(if code & 1 == 1 { fc } else { bc });
                code >>= 1;
                i += color_len;
            }
        }
    }

    pub async fn render_text<DI>(
        &self,
        text: &str,
        top_left: Point,
        fc: Rgb565,
        bc: Rgb565,
        buf: &mut [u8],
        display: &mut AsyncDisplay<DI>,
    ) where
        DI: AsyncInterface,
    {
        let buf_size = self.char_size.width as usize * self.char_size.height as usize * 2;
        let buf = &mut buf[..buf_size];
        for (i, c) in text.chars().enumerate() {
            self.unpack(c, buf, fc.to_be_bytes().as_ref(), bc.to_be_bytes().as_ref());
            let rect = Rectangle::new(
                top_left + Point::new(i as i32 * self.char_size.width as i32, 0),
                self.char_size,
            );
            display
                .set_address_window(
                    rect.top_left.x as u16,
                    rect.top_left.y as u16,
                    rect.top_left.x as u16 + rect.size.width as u16 - 1,
                    rect.top_left.y as u16 + rect.size.height as u16 - 1,
                )
                .await
                .ok();
            display.write_pixels_raw(buf).await.ok();
            /*display
            .write_pixels(
                buf,
                BitDepth::Sixteen,
                Rectangle::new(
                    top_left + Point::new(i as i32 * self.char_size.width as i32, 0),
                    self.char_size,
                ),
            )
            .await;*/
        }
    }
}

/*
09 00 00 00 00 02 00 40 14

00001001 00000000 00000000 00000000 00000000 00000010 00000000 01000000 00010100
000010010000
000000000000
000000000000
000000000010
000000000100
000000010100


00 05 00 00 00 0C 00 00 00

00000000 00000101 00000000 00000000 00000000 00001100 00000000 00000000 00000000

000000 000000 010100 000000000000000000000000001100000000000000000000000000
--------------------
00 00 10 04 41 10 00 01 00
char ! 00000000 00000000 00010000 00000100 01000001 00010000 00000000 00000001 00000000
inv byte 00000000 00000000 00001000 00100000 10000010 00001000 00000000 10000000 00000000
000000
000000
000000
001000
001000
001000
001000
001000
000000
001000
000000
000000


*/
