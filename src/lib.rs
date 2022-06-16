#![cfg_attr(not(test), no_std)]

use defmt::Format;
use embedded_hal::blocking::i2c;

/// INA219 I2C address with A0/A1 grounded as in adafruit breakout board
const ADDRESS: u8 = 0x40;

const CONFIG_REG: u8 = 0x00;
const SHUNT_VOLTAGE_REG: u8 = 0x01;
const BUS_VOLTAGE_REG: u8 = 0x02;
const POWER_REG: u8 = 0x03;
const CURRENT_REG: u8 = 0x04;
const CALIBRATION_REG: u8 = 0x05;

/// Bus Voltage Range
#[derive(PartialEq, Debug, Format)]
pub enum BVR {
    FSR16,
    FSR32,
}

/// Gain
#[derive(PartialEq, Debug, Format)]
pub enum Gain {
    G1w40mv,
    G2w80mv,
    G4w160mv,
    G8w320mv,
}

/// ADC Resolution
#[derive(PartialEq, Debug, Format)]
pub enum AdcRes {
    R9bit,
    R10bit,
    R11bit,
    R12bit,
    R12bit2s1060us,
    R12bit4s2130us,
    R12bit8s4260us,
    R12bit16s8510us,
    R12bit32s17ms,
    R12bit64s34ms,
    R12bit128s96ms,
}

/// Device aquisition mode
#[derive(PartialEq, Debug, Format)]
pub enum Mode {
    PowerDown,
    ShuntVoltageTriggered,
    BusVoltageTriggered,
    ShuntAndBusTriggered,
    ADCOff,
    ShuntVoltageContinuous,
    BusVoltageContinuous,
    ShuntAndBusContinuous,
}

/// the calibration configuration of the device
#[derive(PartialEq, Debug, Format)]
pub struct Ina219Config {
    brng: BVR,
    pg: Gain,
    badc: AdcRes,
    sadc: AdcRes,
    mode: Mode,
}

impl Ina219Config {
    /// the default power-on configuration of device
    pub fn default() -> Self {
        Ina219Config {
            brng: BVR::FSR32,
            pg: Gain::G8w320mv,
            badc: AdcRes::R12bit,
            sadc: AdcRes::R12bit,
            mode: Mode::ShuntAndBusContinuous,
        }
    }
    pub fn new(brng: BVR, pg: Gain, badc: AdcRes, sadc: AdcRes, mode: Mode) -> Self {
        Ina219Config {
            brng,
            pg,
            badc,
            sadc,
            mode,
        }
    }
}

/// get calibration configuration from register value
impl From<u16> for Ina219Config {
    fn from(reg: u16) -> Self {
        Ina219Config {
            brng: if reg & 0x2000 == 0x2000 {
                BVR::FSR32
            } else {
                BVR::FSR16
            },
            pg: match (reg & 0x1800) >> 11 {
                0 => Gain::G1w40mv,
                1 => Gain::G2w80mv,
                2 => Gain::G4w160mv,
                _ => Gain::G8w320mv,
            },
            badc: match (reg & 0x0780) >> 7 {
                0 | 4 => AdcRes::R9bit,
                1 | 5 => AdcRes::R10bit,
                2 | 6 => AdcRes::R11bit,
                3 | 7 | 8 => AdcRes::R12bit,
                9 => AdcRes::R12bit2s1060us,
                10 => AdcRes::R12bit4s2130us,
                11 => AdcRes::R12bit8s4260us,
                12 => AdcRes::R12bit16s8510us,
                13 => AdcRes::R12bit32s17ms,
                14 => AdcRes::R12bit64s34ms,
                15 => AdcRes::R12bit128s96ms,
                _ => panic!(),
            },
            sadc: match (reg & 0x0078) >> 3 {
                0 | 4 => AdcRes::R9bit,
                1 | 5 => AdcRes::R10bit,
                2 | 6 => AdcRes::R11bit,
                3 | 7 | 8 => AdcRes::R12bit,
                9 => AdcRes::R12bit2s1060us,
                10 => AdcRes::R12bit4s2130us,
                11 => AdcRes::R12bit8s4260us,
                12 => AdcRes::R12bit16s8510us,
                13 => AdcRes::R12bit32s17ms,
                14 => AdcRes::R12bit64s34ms,
                15 => AdcRes::R12bit128s96ms,
                _ => panic!(),
            },
            mode: match reg & 0x0007 {
                0 => Mode::PowerDown,
                1 => Mode::ShuntVoltageTriggered,
                2 => Mode::BusVoltageTriggered,
                3 => Mode::ShuntAndBusTriggered,
                4 => Mode::ADCOff,
                5 => Mode::ShuntVoltageContinuous,
                6 => Mode::BusVoltageContinuous,
                _ => Mode::ShuntAndBusContinuous,
            },
        }
    }
}

impl From<Ina219Config> for u16 {
    fn from(config: Ina219Config) -> Self {
        let mut v: u16 = match config.brng {
            BVR::FSR32 => 0x2000,
            BVR::FSR16 => 0,
        };
        v |= (match config.pg {
            Gain::G1w40mv => 0,
            Gain::G2w80mv => 1,
            Gain::G4w160mv => 2,
            Gain::G8w320mv => 3,
        }) << 11;
        v |= (match config.badc {
            AdcRes::R9bit => 0,
            AdcRes::R10bit => 1,
            AdcRes::R11bit => 2,
            AdcRes::R12bit => 3,
            AdcRes::R12bit2s1060us => 9,
            AdcRes::R12bit4s2130us => 10,
            AdcRes::R12bit8s4260us => 11,
            AdcRes::R12bit16s8510us => 12,
            AdcRes::R12bit32s17ms => 13,
            AdcRes::R12bit64s34ms => 14,
            AdcRes::R12bit128s96ms => 15,
        }) << 7;
        v |= (match config.sadc {
            AdcRes::R9bit => 0,
            AdcRes::R10bit => 1,
            AdcRes::R11bit => 2,
            AdcRes::R12bit => 3,
            AdcRes::R12bit2s1060us => 9,
            AdcRes::R12bit4s2130us => 10,
            AdcRes::R12bit8s4260us => 11,
            AdcRes::R12bit16s8510us => 12,
            AdcRes::R12bit32s17ms => 13,
            AdcRes::R12bit64s34ms => 14,
            AdcRes::R12bit128s96ms => 15,
        }) << 3;
        v |= match config.mode {
            Mode::PowerDown => 0,
            Mode::ShuntVoltageTriggered => 1,
            Mode::BusVoltageTriggered => 2,
            Mode::ShuntAndBusTriggered => 3,
            Mode::ADCOff => 4,
            Mode::ShuntVoltageContinuous => 5,
            Mode::BusVoltageContinuous => 6,
            Mode::ShuntAndBusContinuous => 7,
        };
        v
    }
}

/// A INA219 sensor on the I2C bus `I`
pub struct Ina219<I>(I)
where
    I: i2c::Read + i2c::Write;

/// A driver error
#[derive(Debug, PartialEq)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// Math overflow
    MathOverflow,
}

impl<E, I> Ina219<I>
where
    I: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Initializes the Ina219 driver.
    /// This consumes the I2C bus `I`
    pub fn init(i2c: I) -> Self {
        Ina219(i2c)
    }

    /// set the power down mode of the device
    /// if on == true, then set mode to PowerDown
    /// if on == false, then set mode to ShuntAndBusContinuous
    pub fn power_down(&mut self, on: bool) -> Result<(), Error<E>> {
        let mut cur_config = self.read_reg(CONFIG_REG)?;
        if on {
            cur_config &= !0x7;
        } else {
            cur_config = (cur_config & !0x7) | 0x7;
        }
        let command: [u8; 3] = [
            CONFIG_REG,
            (cur_config >> 8) as u8,
            (cur_config & 0xff) as u8,
        ];
        self.0.write(ADDRESS, &command).map_err(Error::I2c)
    }

    fn write_reg(&mut self, reg: u8, val: u16) -> Result<(), Error<E>> {
        let command: [u8; 3] = [reg, (val >> 8) as u8, (val & 0xFF) as u8];
        self.0.write(ADDRESS, &command).map_err(Error::I2c)
    }

    fn read_reg(&mut self, reg: u8) -> Result<u16, Error<E>> {
        let command: [u8; 1] = [reg];
        let mut rd_buffer = [0u8; 2];
        self.0.write(ADDRESS, &command).map_err(Error::I2c)?;
        self.0.read(ADDRESS, &mut rd_buffer).map_err(Error::I2c)?;
        Ok(((rd_buffer[0] as u16) << 8) + rd_buffer[1] as u16)
    }

    fn read_reg_signed(&mut self, reg: u8) -> Result<i16, Error<E>> {
        let command: [u8; 1] = [reg];
        let mut rd_buffer = [0u8; 2];
        self.0.write(ADDRESS, &command).map_err(Error::I2c)?;
        self.0.read(ADDRESS, &mut rd_buffer).map_err(Error::I2c)?;
        Ok(i16::from_be_bytes(rd_buffer))
    }

    /// reset the device
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.write_reg(CONFIG_REG, 0x8000)
    }

    /// Get the current configuration of the INA219 sensor
    pub fn get_configuration(&mut self) -> Result<Ina219Config, Error<E>> {
        let config = self.read_reg(CONFIG_REG)?;
        Ok(Ina219Config::from(config))
    }

    /// set the configuration of the INA219 sensor
    pub fn set_configuration(&mut self, config: Ina219Config) -> Result<(), Error<E>> {
        self.write_reg(CONFIG_REG, config.into())
    }

    /// set the calibration of the INA219 sensor
    pub fn set_calibration(&mut self, calib: u16) -> Result<(), Error<E>> {
        self.write_reg(CALIBRATION_REG, calib)
    }

    /// get the calibration of the INA219 sensor
    pub fn get_calibration(&mut self) -> Result<u16, Error<E>> {
        self.read_reg(CALIBRATION_REG)
    }

    /// test whether the Conversion Ready bit is set or clear
    pub fn is_cnvr_set(&mut self) -> Result<bool, Error<E>> {
        let bv = self.read_reg(BUS_VOLTAGE_REG)?;
        Ok((bv & 0x2) == 0x2)
    }

    /// get the raw bus voltage
    pub fn get_raw_bus_voltage(&mut self) -> Result<u16, Error<E>> {
        let bv = self.read_reg(BUS_VOLTAGE_REG)?;
        if bv & 0x1 == 0x1 {
            Err(Error::MathOverflow)
        } else {
            Ok(bv >> 3)
        }
    }

    /// get the raw shunt voltage
    pub fn get_raw_shunt_voltage(&mut self) -> Result<i16, Error<E>> {
        self.read_reg_signed(SHUNT_VOLTAGE_REG)
    }

    /// get the raw current
    pub fn get_raw_current(&mut self) -> Result<i16, Error<E>> {
        self.read_reg_signed(CURRENT_REG)
    }

    /// get the raw power
    pub fn get_raw_power(&mut self) -> Result<u16, Error<E>> {
        self.read_reg(POWER_REG)
    }

    /// Destroys this driver and releases the I2C bus `I`
    pub fn destroy(self) -> I {
        self.0
    }
}

/// Ina219 device as implemented on Adafruit breakout board
/// the breakout board is using a 0.1 ohm shunt resistor
pub struct AdafruitIna219<I>
where
    I: i2c::Read + i2c::Write,
{
    drv: Ina219<I>,
    cal_value: u16,
    current_divider: f32,
    power_multiplier: f32,
}

impl<E, I> AdafruitIna219<I>
where
    I: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// create a new driver
    /// configuration is set to the maximum voltage and current
    pub fn new(drv: Ina219<I>) -> Result<Self, Error<E>> {
        let mut d = AdafruitIna219 {
            drv,
            cal_value: 0,
            current_divider: 1.0,
            power_multiplier: 1.0,
        };
        d.set_calibration_32v_2a()?;
        Ok(d)
    }

    /// set the configuration to read up to 32V and 2A
    pub fn set_calibration_32v_2a(&mut self) -> Result<(), Error<E>> {
        self.cal_value = u16::to_be(4096);
        self.current_divider = 10.0;
        self.power_multiplier = 2.0;
        self.drv.set_calibration(self.cal_value)?;
        let config = Ina219Config::new(
            BVR::FSR32,
            Gain::G8w320mv,
            AdcRes::R12bit,
            AdcRes::R12bit,
            Mode::ShuntAndBusContinuous,
        );
        self.drv.set_configuration(config)
    }

    /// set the configuration to read up to 32V and 1A
    pub fn set_calibration_32v_1a(&mut self) -> Result<(), Error<E>> {
        self.cal_value = u16::to_be(10240);
        self.current_divider = 25.0;
        self.power_multiplier = 0.8;
        self.drv.set_calibration(self.cal_value)?;
        let config = Ina219Config::new(
            BVR::FSR32,
            Gain::G8w320mv,
            AdcRes::R12bit,
            AdcRes::R12bit,
            Mode::ShuntAndBusContinuous,
        );
        self.drv.set_configuration(config)
    }

    /// set the configuration to read up to 16V and 400mA
    pub fn set_calibration_16v_400ma(&mut self) -> Result<(), Error<E>> {
        self.cal_value = u16::to_be(8192);
        self.current_divider = 20.0;
        self.power_multiplier = 1.0;
        self.drv.set_calibration(self.cal_value)?;
        let config = Ina219Config::new(
            BVR::FSR16,
            Gain::G1w40mv,
            AdcRes::R12bit,
            AdcRes::R12bit,
            Mode::ShuntAndBusContinuous,
        );
        self.drv.set_configuration(config)
    }

    /// get the bus voltage in volts
    pub fn get_bus_voltage_v(&mut self) -> Result<f32, Error<E>> {
        let bv = self.drv.get_raw_bus_voltage()? * 4;
        Ok(bv as f32 * 0.001)
    }

    /// get the shunt voltage in mV
    pub fn get_shunt_voltage_v(&mut self) -> Result<f32, Error<E>> {
        let sv = self.drv.get_raw_shunt_voltage()?;
        Ok(sv as f32 * 0.01)
    }

    /// get the current in mA
    pub fn get_current_ma(&mut self) -> Result<f32, Error<E>> {
        self.drv.set_calibration(self.cal_value)?;
        let cv = self.drv.get_raw_current()?;
        Ok(cv as f32 / self.current_divider)
    }

    /// get the power in mW
    pub fn get_power_mw(&mut self) -> Result<f32, Error<E>> {
        self.drv.set_calibration(self.cal_value)?;
        let pv = self.drv.get_raw_power()?;
        Ok(pv as f32 * self.power_multiplier)
    }
}

///////////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::{
        AdcRes, Error, Gain, Ina219, Ina219Config, Mode, ADDRESS, BUS_VOLTAGE_REG, BVR,
        CALIBRATION_REG, CONFIG_REG, CURRENT_REG, POWER_REG, SHUNT_VOLTAGE_REG,
    };
    use embedded_hal_mock::i2c;

    #[test]
    fn config_default() {
        let config = Ina219Config::default();
        assert_eq!(config.brng, BVR::FSR32);
        assert_eq!(config.pg, Gain::G8w320mv);
        assert_eq!(config.badc, AdcRes::R12bit);
        assert_eq!(config.sadc, AdcRes::R12bit);
        assert_eq!(config.mode, Mode::ShuntAndBusContinuous);
        assert_eq!(u16::from(config), 0x399f);
    }

    #[test]
    fn after_power_on_reset() {
        let expectations = vec![
            i2c::Transaction::write(ADDRESS, vec![CONFIG_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x39, 0x9F]),
        ];
        let mock = i2c::Mock::new(&expectations);

        let mut ina219 = Ina219::init(mock);
        ina219.get_configuration().unwrap();

        let mut mock = ina219.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn reset() {
        let expectations = vec![i2c::Transaction::write(
            ADDRESS,
            vec![CONFIG_REG, 0x80, 0x00],
        )];
        let mock = i2c::Mock::new(&expectations);

        let mut ina219 = Ina219::init(mock);
        ina219.reset().unwrap();

        let mut mock = ina219.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn power_down() {
        let expectations = vec![
            i2c::Transaction::write(ADDRESS, vec![CONFIG_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x39, 0x9F]),
            i2c::Transaction::write(ADDRESS, vec![CONFIG_REG, 0x39, 0x98]),
            i2c::Transaction::write(ADDRESS, vec![CONFIG_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x39, 0x98]),
            i2c::Transaction::write(ADDRESS, vec![CONFIG_REG, 0x39, 0x9F]),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut ina219 = Ina219::init(mock);
        ina219.power_down(true).unwrap();
        ina219.power_down(false).unwrap();

        let mut mock = ina219.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn cnvr_bit() {
        let expectations = vec![
            i2c::Transaction::write(ADDRESS, vec![BUS_VOLTAGE_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x5D, 0x92]),
            i2c::Transaction::write(ADDRESS, vec![BUS_VOLTAGE_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x5D, 0x90]),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut ina219 = Ina219::init(mock);
        assert_eq!(ina219.is_cnvr_set().unwrap(), true);
        assert_eq!(ina219.is_cnvr_set().unwrap(), false);

        let mut mock = ina219.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn overflow() {
        let expectations = vec![
            i2c::Transaction::write(ADDRESS, vec![BUS_VOLTAGE_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x5D, 0x9B]),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut ina219 = Ina219::init(mock);
        assert_eq!(
            ina219.get_raw_bus_voltage().err(),
            Some(Error::MathOverflow)
        );

        let mut mock = ina219.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn example_calc_from_datasheet() {
        let expectations = vec![
            i2c::Transaction::write(ADDRESS, vec![CONFIG_REG, 0x01, 0x9F]),
            i2c::Transaction::write(ADDRESS, vec![SHUNT_VOLTAGE_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x07, 0xD0]),
            i2c::Transaction::write(ADDRESS, vec![BUS_VOLTAGE_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x5D, 0x98]),
            i2c::Transaction::write(ADDRESS, vec![CALIBRATION_REG, 0x50, 0x00]),
            i2c::Transaction::write(ADDRESS, vec![CURRENT_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x27, 0x10]),
            i2c::Transaction::write(ADDRESS, vec![POWER_REG]),
            i2c::Transaction::read(ADDRESS, vec![0x17, 0x66]),
        ];
        let config = Ina219Config::new(
            BVR::FSR16,
            Gain::G1w40mv,
            AdcRes::R12bit,
            AdcRes::R12bit,
            Mode::ShuntAndBusContinuous,
        );

        let mock = i2c::Mock::new(&expectations);

        let mut ina219 = Ina219::init(mock);
        ina219.set_configuration(config).unwrap();
        let shunt = ina219.get_raw_shunt_voltage().unwrap();
        assert_eq!(shunt, 0x07D0);
        let bus = ina219.get_raw_bus_voltage().unwrap();
        assert_eq!(bus, 0x0BB3);
        ina219.set_calibration(0x5000).unwrap();
        let current = ina219.get_raw_current().unwrap();
        assert_eq!(current, 10000);
        let power = ina219.get_raw_power().unwrap();
        assert_eq!(power, 5990);

        let mut mock = ina219.destroy();
        mock.done(); // verify expectations
    }
}
