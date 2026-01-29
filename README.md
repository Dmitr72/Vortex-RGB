# Vortex RGB Driver STM32G431RBT6

## О проекте / About the Project

**[RU]** Драйвер RGB для микроконтроллера STM32G431RBT6. Проект разработан для управления RGB-устройствами с использованием STM32CubeMX и STM32CubeIDE.

**[EN]** RGB driver for STM32G431RBT6 microcontroller. The project is designed for controlling RGB devices using STM32CubeMX and STM32CubeIDE.

---

## Вопросы и поддержка / Questions and Support

### [RU] Как задать вопрос?

**Да, вы можете задавать вопросы о текущем репозитории!** 

Для этого есть несколько способов:

1. **GitHub Issues** - создайте новое issue для:
   - Вопросов об использовании проекта
   - Сообщений об ошибках
   - Предложений по улучшению
   - Запросов новой функциональности

2. **GitHub Discussions** - если доступно в репозитории, используйте для:
   - Общих вопросов
   - Обсуждений архитектуры
   - Обмена опытом

### [EN] How to Ask Questions?

**Yes, you can ask questions about this repository!**

There are several ways to do this:

1. **GitHub Issues** - create a new issue for:
   - Questions about project usage
   - Bug reports
   - Improvement suggestions
   - Feature requests

2. **GitHub Discussions** - if available in the repository, use for:
   - General questions
   - Architecture discussions
   - Experience sharing

---

## Технические характеристики / Technical Specifications

- **MCU**: STM32G431RBTx
- **IDE**: STM32CubeIDE
- **STM32CubeMX Version**: 6.16.0
- **Firmware Package**: STM32Cube FW_G4 V1.6.1

### Периферия / Peripherals

- **ADC**: ADC1, ADC2 (множественные каналы для RGB)
- **Comparators**: COMP1, COMP2, COMP4
- **DAC**: DAC1, DAC3
- **Communication**:
  - USB Device (FS)
  - FDCAN1
  - I2C2
  - SPI1
  - USART1
- **GPIO**: Множественные пины для управления RGB

### Основные пины RGB / Main RGB Pins

- **Red (R)**:
  - R_ADC: PC1
  - R_SHT_P/N: PB11, PB15
  - R_TH_CS: PC6
  - R_DIV_CS: PC7
  - R_PEN: PC8

- **Green (G)**:
  - G_ADC: PC2
  - G_SHT_P/N: PA7, PC4
  - G_TH_CS: PB12
  - G_DIV_CS: PB13
  - G_PEN: PB10

- **Blue (B)**:
  - B_ADC: PC3
  - B_SHT_P/N: PA0, PA1
  - B_TH_CS: PC14
  - B_DIV_CS: PC15
  - B_PEN: PA5

---

## Структура проекта / Project Structure

```
.
├── Core/              # Основной код проекта / Main project code
├── Drivers/           # HAL драйверы STM32 / STM32 HAL drivers
├── Middlewares/       # Промежуточное ПО / Middleware
├── USB_Device/        # USB Device библиотеки / USB Device libraries
├── X-CUBE-MEMS1/      # MEMS сенсор библиотеки / MEMS sensor libraries
├── Debug/             # Отладочные файлы / Debug files
├── *.ioc              # Файл конфигурации STM32CubeMX / STM32CubeMX configuration
└── *.ld               # Linker script
```

---

## Сборка проекта / Building the Project

### Требования / Requirements

1. STM32CubeIDE (рекомендуется версия 1.16.1 или новее)
2. STM32CubeMX (версия 6.16.0 или новее)
3. ST-Link программатор для загрузки прошивки

### Инструкции по сборке / Build Instructions

**[RU]**
1. Откройте проект в STM32CubeIDE
2. Соберите проект: `Project > Build Project`
3. Загрузите прошивку на устройство: `Run > Debug` или `Run > Run`

**[EN]**
1. Open the project in STM32CubeIDE
2. Build the project: `Project > Build Project`
3. Flash the firmware to the device: `Run > Debug` or `Run > Run`

---

## Вклад в проект / Contributing

**[RU]** Вклад в проект приветствуется! Пожалуйста:
- Создавайте Pull Request для предложенных изменений
- Убедитесь, что код компилируется без ошибок
- Опишите изменения в описании PR

**[EN]** Contributions are welcome! Please:
- Create a Pull Request for proposed changes
- Ensure the code compiles without errors
- Describe the changes in the PR description

---

## Лицензия / License

Пожалуйста, ознакомьтесь с файлом LICENSE в репозитории (если имеется) или свяжитесь с автором проекта.

Please refer to the LICENSE file in the repository (if available) or contact the project author.

---

## Контакты / Contacts

Автор / Author: [Dmitr72](https://github.com/Dmitr72)

Репозиторий / Repository: [Vortex-RGB](https://github.com/Dmitr72/Vortex-RGB)
