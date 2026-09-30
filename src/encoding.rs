use crate::{Error, Result};

/// FEETECH direction-bit decoding, not two's complement. sign_bit must be 0..15.
pub fn decode_sign_magnitude(raw: u16, sign_bit: u8) -> Result<i32> {
    if sign_bit > 15 {
        return Err(Error::InvalidArgument("sign bit"));
    }
    let mask = 1u16 << sign_bit;
    Ok(if raw & mask != 0 {
        -i32::from(raw & !mask)
    } else {
        i32::from(raw)
    })
}
pub fn encode_sign_magnitude(value: i32, sign_bit: u8) -> Result<u16> {
    if sign_bit > 15 {
        return Err(Error::InvalidArgument("sign bit"));
    }
    let limit = (1i32 << sign_bit) - 1;
    if value < -limit || value > limit {
        return Err(Error::InvalidArgument("signed magnitude overflow"));
    }
    Ok(if value < 0 {
        (-value) as u16 | (1 << sign_bit)
    } else {
        value as u16
    })
}
pub(crate) fn signed15(raw: u16) -> i32 {
    if raw & 0x8000 != 0 {
        -i32::from(raw & 0x7fff)
    } else {
        i32::from(raw)
    }
}
pub(crate) fn signed10(raw: u16) -> i32 {
    if raw & 0x0400 != 0 {
        -i32::from(raw & !0x0400)
    } else {
        i32::from(raw)
    }
}

/// IEEE 754 binary16 to f64, including negative zero, subnormals, NaN and infinity.
pub fn half_float(raw: u16) -> f64 {
    let sign = if raw & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = i32::from((raw >> 10) & 31);
    let frac = f64::from(raw & 1023);
    match exp {
        31 => {
            if frac != 0.0 {
                f64::NAN
            } else {
                sign * f64::INFINITY
            }
        }
        0 => sign * frac * 2f64.powi(-24),
        _ => sign * (1.0 + frac / 1024.0) * 2f64.powi(exp - 15),
    }
}
