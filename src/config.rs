use crate::sbus::FutabaPacket;
use rp2040_hal::gpio::*;
use rp2040_hal::*;

pub type SbusTxPin = hal::gpio::Pin<bank0::Gpio0, FunctionUart, PullDown>;
pub type SbusRxPin = hal::gpio::Pin<bank0::Gpio1, FunctionUart, PullDown>;

pub type SbusRx = uart::Reader<pac::UART0, (SbusTxPin, SbusRxPin)>;
pub type SbusTx = uart::Writer<pac::UART0, (SbusTxPin, SbusRxPin)>;

pub type SbusSinkConfig = (
    hal::dma::Channel<hal::dma::CH0>,
    &'static mut FutabaPacket,
    SbusTx,
);

pub type SbusSinkTransfer =
    dma::single_buffer::Transfer<dma::Channel<dma::CH0>, &'static mut FutabaPacket, SbusTx>;

pub enum SbusSink {
    StandBy(SbusSinkConfig),
    InProgress(SbusSinkTransfer),
}
