// To change the behavior of this template, see the documentation:
// Change min and max values to set the servo movement range
// Change steps within 1 to 200 to regulate speed of servo movement
// Change threshold value to set the PWM input level for servo direction change 

#![no_std]


#![no_main]


use panic_probe as _;


// через неї була помилка - "cannot find macro `info` in this scope"
//use defmt::*;


use rtic::app;


use rp2040_hal as hal;


use hal::{
    clocks::init_clocks_and_plls,

    gpio::{FunctionPwm,Pins},

    
    pac,

    pwm::Slices,
  
    sio::Sio,

    watchdog::Watchdog,
};



use embedded_hal::pwm::SetDutyCycle;

use rtic_monotonics::rp2040::prelude::*;


#[link_section = ".boot2"] // кладем массив в специальную секцию памяти
#[no_mangle]               // запрещаем переименование символа
#[used]                    // запрещаем выкидывать как "неиспользуемый"
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

rp2040_timer_monotonic!(Mono);


#[app(device = pac, peripherals = true)]
mod app {
    // Подтягиваем все из внешнего модуля
    use super::*;
    use core::panic;
    // ===== Shared ресурсы =====
    // Shared — ресурсы, к которым могут обращаться разные задачи одновременно.
    #[shared]
    struct Shared {
        pulse_width_us_1: u32,
        pulse_width_us_2: u32,
        pulse_width_us_3: u32,
    }

    // ===== Local ресурсы =====
    // Local — ресурсы, принадлежащие конкретной задаче.
    #[local]
    struct Local {
        // Пин номер 2 вхід для PWM сигналу
        pwm_in: hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::FunctionSioInput, hal::gpio::PullDown>,
        pwm_in_2 : hal::gpio::Pin<hal::gpio::bank0::Gpio3, hal::gpio::FunctionSioInput, hal::gpio::PullDown>,
        pwm_in_3 : hal::gpio::Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionSioInput, hal::gpio::PullDown>,
        // останній час фронту ВГОРУ в мікросекундах
        last_rise_us_1: u32,
        last_rise_us_2: u32,
        last_rise_us_3: u32,

        pwm3: hal::pwm::Slice<hal::pwm::Pwm3, hal::pwm::FreeRunning>,
        pwm4: hal::pwm::Slice<hal::pwm::Pwm4, hal::pwm::FreeRunning>,   
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut resets = ctx.device.RESETS;
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        init_clocks_and_plls(
            12_000_000,          
            ctx.device.XOSC,     
            ctx.device.CLOCKS,   
            ctx.device.PLL_SYS,  
            ctx.device.PLL_USB,  
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();


        Mono::start(ctx.device.TIMER, &mut resets);


        let sio = Sio::new(ctx.device.SIO);

        // Создаем структуру pins.gpio0, pins.gpio1 и т.д.
        let pins = Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let pwm_in = pins.gpio2.into_pull_down_input(); // налаштовуємо пін як вхід з підтяжкою до землі
        let pwm_in_2 = pins.gpio3.into_pull_down_input(); // налаштовуємо пін як вхід з підтяжкою до землі
        let pwm_in_3 = pins.gpio4.into_pull_down_input(); // налаштовуємо пін як вхід з підтяжкою до землі
        // Налаштовуємо переривання на обидва фронти
        pwm_in.set_interrupt_enabled(
            hal::gpio::Interrupt::EdgeHigh,
            true,
        );
        pwm_in.set_interrupt_enabled(
            hal::gpio::Interrupt::EdgeLow,
            true,
        );
        pwm_in_2.set_interrupt_enabled(
            hal::gpio::Interrupt::EdgeHigh,
            true,
        );
        pwm_in_2.set_interrupt_enabled(
            hal::gpio::Interrupt::EdgeLow,
            true,
        );
        pwm_in_3.set_interrupt_enabled(
            hal::gpio::Interrupt::EdgeHigh,
            true,
        );
        pwm_in_3.set_interrupt_enabled(
            hal::gpio::Interrupt::EdgeLow,
            true,
        );

        logic_task::spawn().unwrap();
        // через це була помилка - "mismatched types: expected struct `rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio6, rp2040_hal::gpio::FunctionPwm>`, found struct `rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio6, rp2040_hal::gpio::FunctionSioInput, rp2040_hal::gpio::PullDown>`"
        //let _pwm0_pin = pins.gpio6.into_mode::<hal::gpio::FunctionPwm>(); //налаштування піна 6 як пвм вихід
        //let _pwm2_pin = pins.gpio7.into_mode::<hal::gpio::FunctionPwm>(); //налаштування піна 7 як пвм вихід
        //let _pwm3_pin = pins.gpio8.into_mode::<hal::gpio::FunctionPwm>(); //налаштування піна 8 як пвм вихід
        //======= PWM =====
         // Получаем доступ ко всем PWM slice-ам
        let pwm_slices = Slices::new(ctx.device.PWM, &mut resets);
        // 
        // це 
        let mut pwm3 = pwm_slices.pwm3;
        let mut pwm4 = pwm_slices.pwm4;

        // GPIO -> PWM function
        let pin0 = pins.gpio6.into_function::<FunctionPwm>(); 
        let pin1 = pins.gpio7.into_function::<FunctionPwm>();
        let pin2 = pins.gpio8.into_function::<FunctionPwm>();

        // ===== НАСТРОЙКА PWM =====
        // Делитель: 125 МГц / 125 = 1 МГц
        // 1 тик PWM = 1 микросекунда
        pwm3.set_div_int(125);
        pwm4.set_div_int(125);

        // TOP = 20000 → период 20 мс → 50 Гц
        pwm3.set_top(20_000);
        pwm4.set_top(20_000);

        // Включаем PWM
        pwm3.enable();
        pwm4.enable();


        let _pwm_pin0 = pwm3.channel_a.output_to(pin0);
        let _pwm_pin1 = pwm3.channel_b.output_to(pin1);
        let _pwm_pin = pwm4.channel_a.output_to(pin2);

        // Возвращаем ресурсы RTIC, shared -для всіх задач, local - для цієї задачі 
        (Shared {
            pulse_width_us_1: 0,
            pulse_width_us_2: 0,
            pulse_width_us_3: 0,
        }, Local {
            pwm_in,
            pwm_in_2,
            pwm_in_3,
            last_rise_us_1: 0,
            last_rise_us_2: 0,
            last_rise_us_3: 0,
            pwm3,
            pwm4,
        },

    )
    }



    
// task — RTIC задача, прив’язана до GPIO IRQ
#[task(
    binds = IO_IRQ_BANK0,                 // апаратне переривання GPIO
    local = [
        pwm_in, 
        pwm_in_2,
        pwm_in_3,
        last_rise_us_1,
        last_rise_us_2,
        last_rise_us_3],        // локальні ресурси ТІЛЬКИ для цього IRQ
    shared = [pulse_width_us_1, pulse_width_us_2, pulse_width_us_3]              // спільний ресурс
)]
fn pwm_input_irq(mut ctx: pwm_input_irq::Context) {

    let pwm_input_irq::LocalResources {
        pwm_in,
        pwm_in_2,
        pwm_in_3,
        last_rise_us_2,
        last_rise_us_1,
        last_rise_us_3,
        ..
    } = ctx.local;

    let now = Mono::now().ticks() as u32;
    // pwm_in_1
    // фронт ВГОРУ (початок імпульсу)
    if pwm_in.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
        *last_rise_us_1 = now;
        pwm_in.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
    }

    // фронт ВНИЗ (кінець імпульсу)
    if pwm_in.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
        let width = now.wrapping_sub(*last_rise_us_1);

        ctx.shared.pulse_width_us_1.lock(|pw| {
            *pw = width;
        });

        pwm_in.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
    }
    // pwm_in_2
    if pwm_in_2.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
        *last_rise_us_2 = now;
        pwm_in_2.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
    }

    // фронт ВНИЗ (кінець імпульсу)
    if pwm_in_2.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
        let width = now.wrapping_sub(*last_rise_us_2);

        ctx.shared.pulse_width_us_2.lock(|pw| {
            *pw = width;
        });

        pwm_in_2.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
    }
    // pwm_in_3
    if pwm_in_3.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
        *last_rise_us_3 = now;
        pwm_in_3.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
    }

    // фронт ВНИЗ (кінець імпульсу)
    if pwm_in_3.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
        let width = now.wrapping_sub(*last_rise_us_3);

        ctx.shared.pulse_width_us_3.lock(|pw| {
            *pw = width;
        });

        pwm_in_3.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
    }
    
}





#[derive(Copy, Clone)] // дозволяє копіювати ServoState без нього не скомпілюється
enum ServoState {
    MovingUp,
    MovingDown,
    PauseUntil(u64), //timestamp, до которого ждем
}

struct ServoCtrl {
    value: u16, // поточне значення ШІМ
    state: ServoState, // один из 4 вариантов enum ServoState
}

fn update_servo(
    servo: &mut ServoCtrl, // &mut - це мутабельне посилання, дозволяє змінювати значення без копіювання
    min: u16,
    max: u16,
    step: u16,
    now: u64,
){
    match servo.state {
        ServoState::MovingUp => {
            servo.value += step;
            if servo.value >= max {
                servo.value = max;
                //servo.state = ServoState::PauseUntil(now + 10_000_000); // пауза на межі
            }
        },
        ServoState::MovingDown => {
            servo.value -= step;
            if servo.value <= min {
                servo.value = min;
                //servo.state = ServoState::PauseUntil(now + 10_000_000); // пауза на межі
            }
        },
        ServoState::PauseUntil(until) => {
            if now >= until {
                // закінчилась пауза, змінюємо напрямок руху
                if servo.value >= max {
                    servo.state = ServoState::MovingDown;
                } else {
                    servo.state = ServoState::MovingUp;
                }
            }
        }
    }
}


// викликається тільки коли я її сам викликаю через spawn
#[task(local = [pwm3, pwm4],shared = [pulse_width_us_1, pulse_width_us_2, pulse_width_us_3])]
async fn logic_task(mut ctx: logic_task::Context) {
    let logic_task::LocalResources { 
        pwm3,
        pwm4,
        ..
    } = ctx.local;
    // повільний контроль серво
    let min = 810;
    let max = 1450;
    let threshold = 1300;


    let steps = 40; 
    let step = (max - min) / steps;
    let mut servos = [
        ServoCtrl { value: min, state: ServoState::MovingUp },
        ServoCtrl { value: min, state: ServoState::MovingUp },
        ServoCtrl { value: min, state: ServoState::MovingUp },
    ];

    loop {
        let pw_1 = ctx.shared.pulse_width_us_1.lock(|pw| *pw);
        let pw_2 = ctx.shared.pulse_width_us_2.lock(|pw| *pw);
        let pw_3 = ctx.shared.pulse_width_us_3.lock(|pw| *pw);

        if pw_1 > threshold {
            servos[0].state = ServoState::MovingUp;
        } else {
            servos[0].state = ServoState::MovingDown;
        }
        if pw_2 > threshold {
            servos[1].state = ServoState::MovingUp;
        } else {
            servos[1].state = ServoState::MovingDown;
        }
        if pw_3 > threshold {
            servos[2].state = ServoState::MovingUp;
        } else {
            servos[2].state = ServoState::MovingDown;
        }


        let now = Mono::now().ticks();
        
        update_servo(&mut servos[0], min, max, step, now);
        update_servo(&mut servos[1], min, max, step, now);
        update_servo(&mut servos[2], min, max, step, now);

        pwm3.channel_a.set_duty_cycle(servos[0].value).ok();
        pwm3.channel_b.set_duty_cycle(servos[1].value).ok();
        pwm4.channel_a.set_duty_cycle(servos[2].value).ok();

        Mono::delay(15.millis()).await;
    }
}

}
