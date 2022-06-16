#![no_std]
#![no_main]

use defmt_rtt as _; // defmt transport
use panic_probe as _;
use stm32f1xx_hal as _; // memory layout // panic handler

use stm32f1xx_hal::{
    gpio::{gpiob, Alternate, OpenDrain},
    i2c::BlockingI2c,
    pac,
};
use ti_ina219::Ina219;

// type definition for pins
type SclPin = gpiob::PB10<Alternate<OpenDrain>>;
type SdaPin = gpiob::PB11<Alternate<OpenDrain>>;

struct State {
    ina219: Ina219<BlockingI2c<pac::I2C2, (SclPin, SdaPin)>>,
}

#[defmt_test::tests]
mod tests {
    use defmt::{assert_eq, unwrap};
    use stm32f1xx_hal::{
        i2c::{BlockingI2c, DutyCycle, Mode},
        pac,
        prelude::*,
    };

    use super::State;

    #[init]
    fn setup() -> State {
        // enable and reset the cycle counter
        let mut core = unwrap!(cortex_m::Peripherals::take());
        core.DCB.enable_trace();
        unsafe { core.DWT.cyccnt.write(0) }
        core.DWT.enable_cycle_counter();
        defmt::timestamp!("{=u32:us}", cortex_m::peripheral::DWT::cycle_count());

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let dp = unwrap!(stm32f1xx_hal::pac::Peripherals::take());
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        //let afio = dp.AFIO.constrain();
        // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
        // `clocks`
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);

        // get the pins
        let mut gpiob = dp.GPIOB.split();
        let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

        // initialize I2C
        let i2c = BlockingI2c::i2c2(
            dp.I2C2,
            (scl, sda),
            Mode::Fast {
                frequency: 400.kHz(),
                duty_cycle: DutyCycle::Ratio16to9,
            },
            clocks,
            1000, // start_timeout_us
            10,   // start_retries
            1000, // addr_timeout_us
            1000, // data_timeout_us
        );

        let ina219 = ti_ina219::Ina219::init(i2c);
        State { ina219 }
    }

    #[test]
    fn confirm_configuration(state: &mut State) {
        let EXPECTED: ti_ina219::Ina219Config = ti_ina219::Ina219Config::default();
        let config = state.ina219.get_configuration().unwrap();
        assert_eq!(EXPECTED, config);
    }

    #[test]
    fn get_shunt_bus_voltage(state: &mut State) {
        state.ina219.set_calibration(0x5000).unwrap();
        let sv = state.ina219.get_raw_shunt_voltage().unwrap();
        let bv = state.ina219.get_raw_bus_voltage().unwrap();
        defmt::info!("shunt voltage: {=i16}", sv);
        defmt::info!("bus voltage: {=u16}", bv);
    }
}
