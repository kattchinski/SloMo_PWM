// To change the behavior of this template, see the documentation:
// Change min and max values to set the servo movement range
// Change steps within 1 to 200 to regulate speed of servo movement
// Change threshold value to set the PWM input level for servo direction change 






// Говорим Rust: "Я пишу под микроконтроллер, тут нет стандартной библиотеки
// (Vec, String, файлы, потоки и т.д.)"
#![no_std]

// Говорим Rust: "У меня не будет обычной функции main(), как на ПК"
// (RTIC + embedded runtime сами управляют стартом программы)
#![no_main]

// Подключаем panic handler.
// Если в коде произойдет panic (unwrap, assert и т.п.),
// будет корректная обработка для embedded-систем.
// panic_probe обычно используется вместе с defmt для отладки.
use panic_probe as _;


// через неї була помилка - "cannot find macro `info` in this scope"
//use defmt::*;



// Импортируем макрос RTIC app — это фреймворк для задач и прерываний.
use rtic::app;

// rp2040-hal — библиотека для работы с периферией RP2040
// (GPIO, PWM, UART, таймеры, тактирование и т.д.)
use rp2040_hal as hal;

// Импортируем нужные модули из hal
use hal::{
    // Функция инициализации тактирования:
    // настраивает PLL, системную частоту, USB частоту и т.д.
    clocks::init_clocks_and_plls,

    // FunctionPwm — режим пина "PWM выход"
    // Pins — структура со всеми GPIO пинами
    gpio::{FunctionPwm,Pins},

    // pac (Peripheral Access Crate) — низкоуровневый доступ к регистрам чипа
    pac,

    pwm::Slices,
    // Sio — доступ к GPIO банку (обязателен для Pins)
    sio::Sio,

    // Watchdog нужен для инициализации тактирования
    watchdog::Watchdog,
};



use embedded_hal::pwm::SetDutyCycle;
// rtic_monotonics — модуль таймеров для RTIC.
// prelude подтягивает millis(), secs() и т.п.
use rtic_monotonics::rp2040::prelude::*;

// ===== BOOT2 =====
// RP2040 требует boot2 blob — маленький код,
// который запускается первым и настраивает XIP flash.
// Без него прошивка может не стартовать.
#[link_section = ".boot2"] // кладем массив в специальную секцию памяти
#[no_mangle]               // запрещаем переименование символа
#[used]                    // запрещаем выкидывать как "неиспользуемый"
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// ===== MONOTONIC =====
// Создаем RTIC monotonic таймер на базе TIMER RP2040.
// Он позволяет использовать Mono::delay(100.millis()).await
rp2040_timer_monotonic!(Mono);

// ===== RTIC APP =====
// Описываем RTIC-приложение.
// device = pac — используем периферию из rp2040 PAC
// peripherals = true — RTIC дает доступ к железу через ctx.device
#[app(device = pac, peripherals = true)]
mod app {
    // Подтягиваем все из внешнего модуля
    use super::*;
    use core::panic;
    // ===== Shared ресурсы =====
    // Shared — ресурсы, к которым могут обращаться разные задачи одновременно.
    // В этом примере shared ресурсов нет, но структура обязательна.
    #[shared]
    struct Shared {
        pulse_width_us_1: u32,
        pulse_width_us_2: u32,
        pulse_width_us_3: u32,
    }

    // ===== Local ресурсы =====
    // Local — ресурсы, принадлежащие конкретной задаче.
    // Здесь мы храним PWM slice (pwm0).
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

    // ===== INIT =====
    // init выполняется один раз при старте микроконтроллера.
    // В RTIC 2.x init всегда возвращает (Shared, Local).
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Контроллер ресетов периферии.
        // Его нужно передавать в инициализаторы GPIO, PWM и т.д.
        let mut resets = ctx.device.RESETS;

        // Watchdog нужен для init_clocks_and_plls
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        // ===== ИНИЦИАЛИЗАЦИЯ ТАКТИРОВАНИЯ =====
        // 12_000_000 — частота кварца (обычно 12 МГц на Pico)
        init_clocks_and_plls(
            12_000_000,          // частота кварца
            ctx.device.XOSC,     // внешний осциллятор
            ctx.device.CLOCKS,   // блок управления тактированием
            ctx.device.PLL_SYS,  // PLL системной частоты
            ctx.device.PLL_USB,  // PLL USB
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // ===== ЗАПУСК MONOTONIC =====
        // КРИТИЧЕСКИ ВАЖНО:
        // без этого Mono::delay().await будет ждать бесконечно.
        Mono::start(ctx.device.TIMER, &mut resets);

        // ===== GPIO =====
        // Sio нужен для работы с GPIO банком
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