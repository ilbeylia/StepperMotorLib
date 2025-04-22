# STM32 Stepper Motor Library

This README is available in multiple languages. Please select your preferred language below:

- [English](#english)
- [Türkçe](#türkçe)

---

## English

This library enables step motor control using STM32 microcontrollers. By utilizing PWM output and a direction pin, you can rotate the motor clockwise (CW) or counterclockwise (CCW), move it by a specific number of steps, and stop the operation.

### Features

- Driving step motor with PWM
- CW / CCW direction control
- Rotation with a specified number of steps
- Motor start/stop
- TIM and GPIO configuration

### Structures and Types

#### `direction_e`
Motor direction:
- `CW`  – Clockwise
- `CCW` – Counterclockwise

#### `state_e`
Motor state:
- `MOTOR_ON`
- `MOTOR_OFF`

#### `m_cnfg_s`
Motor configuration structure:
```c
typedef struct {
  TIM_HandleTypeDef *tim_handle;
  uint32_t tim_channel;
  GPIO_TypeDef *dir_portx;
  uint32_t dir_pin;
  struct {
    int counter;
    int state;
    int step;
  } m_set;
} m_cnfg_s;
```

### Functions

#### `void stepper_motor_init(...)`
Initializes the necessary settings to start the motor.

#### `void startMotor(...)`
Starts the motor with specified step count and direction.

#### `void stopMotor(...)`
Stops the motor when the specified step count is reached.

### Usage Example

```c
/* USER CODE BEGIN Includes */
    #include "StepMotor_lib.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
    m_cnfg_s stepperMotor;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      stepperMotor.m_set.counter++;
    }
    stopMotor(&stepperMotor);
  }
}
/* USER CODE END 0 */

/* USER CODE BEGIN Init */
      stepper_motor_init(&stepperMotor, &htim1, TIM_CHANNEL_1, GPIOE, GPIO_PIN_10);
/* USER CODE END Init */

while(1)
{
    startMotor(&stepperMotor, CCW, 2000); 
    Hal_Delay(100);
    startMotor(&stepperMotor, CW, 2000);
    Hal_Delay(100);
}
```

> **Note:** This library uses `HAL_TIM_PWM_Start_IT` and `HAL_TIM_PWM_Stop_IT`. PWM interrupts and the `HAL_TIM_PWM_PulseFinishedCallback()` function must be enabled.

### Developer Notes

- Place `StepMotor_lib.c` and `.h` files in the `Src/` and `Inc/` folders.
- Ensure the `tim_handle` and direction pin are correctly defined.
- Configure your timer to enable PWM output and interrupts.

### License

This project is open-source. You are free to use, modify, and share it with proper attribution.

---

## Türkçe

Bu kütüphane, STM32 mikrodenetleyiciler kullanarak step motor kontrolü yapmanızı sağlar. PWM çıkışı ve yön pinini kullanarak motoru saat yönünde (CW) veya saat yönünün tersine (CCW) döndürebilir, belirli adım sayıları kadar hareket ettirebilir ve işlemi durdurabilirsiniz.

### Özellikler

- PWM ile step motor sürme
- CW / CCW yön kontrolü
- Belirli adım sayısı ile döndürme
- Motor başlatma/durdurma
- TIM ve GPIO konfigürasyonu

### Yapılar ve Tipler

#### `direction_e`
Motor yönü:
- `CW`  – Saat yönü
- `CCW` – Saat yönünün tersi

#### `state_e`
Motor durumu:
- `MOTOR_ON`
- `MOTOR_OFF`

#### `m_cnfg_s`
Motor konfigürasyon yapısı:
```c
typedef struct {
  TIM_HandleTypeDef *tim_handle;
  uint32_t tim_channel;
  GPIO_TypeDef *dir_portx;
  uint32_t dir_pin;
  struct {
    int counter;
    int state;
    int step;
  } m_set;
} m_cnfg_s;
```

### Fonksiyonlar

#### `void stepper_motor_init(...)`
Motoru başlatmak için gerekli ayarları yapar.

#### `void startMotor(...)`
Motoru belirli adım ve yön bilgisiyle çalıştırır.

#### `void stopMotor(...)`
Motor belirtilen adım sayısına ulaştığında durur.

### Kullanım Örneği

```c
/* USER CODE BEGIN Includes */
    #include "StepMotor_lib.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
    m_cnfg_s stepperMotor;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      stepperMotor.m_set.counter++;
    }
    stopMotor(&stepperMotor);
  }
}
/* USER CODE END 0 */

/* USER CODE BEGIN Init */
      stepper_motor_init(&stepperMotor, &htim1, TIM_CHANNEL_1, GPIOE, GPIO_PIN_10);
/* USER CODE END Init */

while(1)
{
    startMotor(&stepperMotor, CCW, 2000); 
    Hal_Delay(100);
    startMotor(&stepperMotor, CW, 2000);
    Hal_Delay(100);
}
```

> **Not:** Bu kütüphane `HAL_TIM_PWM_Start_IT` ve `HAL_TIM_PWM_Stop_IT` kullanır. PWM interrupt'ları ve `HAL_TIM_PWM_PulseFinishedCallback()` fonksiyonu aktif olmalıdır.

### Geliştirici Notları

- `StepMotor_lib.c` ve `.h` dosyalarını `Src/` ve `Inc/` klasörlerine yerleştirin.
- `tim_handle` ve yön pininin doğru tanımlandığından emin olun.
- Timer konfigürasyonunuz PWM output ve interrupt aktif olacak şekilde ayarlanmalıdır.

### Lisans

Bu proje açık kaynaklıdır. Dilersen kullanabilir, geliştirebilir, kaynak göstererek paylaşabilirsin.