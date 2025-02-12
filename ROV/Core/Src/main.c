/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include <stdbool.h>
#include "string.h"
#include "math.h"
#include "ADC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68<<1 // MPU6050 sensörünün I2C adresi. 0x68 adresini sola 1 bit kaydırarak (0xD0) elde edilir.
#define PWR_MGMT_1_REG 0x6B // Güç yönetimi kaydı 1'in adresi. Sensörün çalışma modunu ayarlamak için kullanılır.
#define SMPLRT_DIV_REG 0x19 // Örnekleme hızı bölen kaydının adresi. Sensörün örnekleme hızını ayarlamak için kullanılır.
#define GYRO_CNFG_REG 0x1B // Jiroskop yapılandırma kaydının adresi. Jiroskopun hassasiyetini ayarlamak için kullanılır.
#define ACC_CNFG_REG 0x1C // İvmeölçer yapılandırma kaydının adresi. İvmeölçerin hassasiyetini ayarlamak için kullanılır
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/******************** MOTOR SİSTEMLERİ ********************************/

int depth, roll, v;
// depth: Derinlik, su altı aracının bulunduğu derinliği temsil eder.
// roll: Aracın yatay eğilme açısı, aracın sağa veya sola eğilmesini ifade eder.
// v: Aracın hızını ifade eder.

float desired_speed = 0.6;
// İstenen hız değeri (referans): Aracın hedeflediği hız. Bu, kontrol algoritmalarında karşılaştırılacak hız değeridir.

float current_speed = 0.0;
// Gerçek hız değeri: Motorlardan gelen verilerle hesaplanan aracın mevcut hızı.

float previous_x_ivme = 0.0;
// Önceki ivme değeri: Hız hesaplamalarında kullanılan önceki ivme değerini saklar.

float Kp_motor = 0.7, Ki_motor = 0.05, Kd_motor = 0.003;
// Fırçasız motorlar için PID katsayıları:
// Kp_motor: Proportional katsayısı, hatanın büyüklüğüne göre tepkiyi belirler.
// Ki_motor: Integral katsayısı, zamanla biriken hataların etkisini düzeltir.
// Kd_motor: Derivative katsayısı, hızlanan hataları engellemeye çalışır.

float integral_motor = 0.0, derivative_motor = 0.0, error_motor = 0.0, previous_error_motor = 0.0;
// Motorlar için PID kontrolü: Bu değişkenler PID algoritmasında kullanılır.
// integral_motor: Hata integralini tutar, hata birikimini düzeltir.
// derivative_motor: Hata türevini tutar, hatadaki değişim hızını ölçer.
// error_motor: Hata değeri, istenen hız ile mevcut hız arasındaki farktır.
// previous_error_motor: Önceki hata değeri, türev hesaplamasında kullanılır.

float pid_output_motor = 0.0;
// PID çıkışı (motorlar): PID algoritmasından çıkan kontrol sinyalini saklar. Bu sinyal, motor hızını kontrol etmek için kullanılır.

float Kp_pump = 1.0, Ki_pump = 2.0, Kd_pump = 4.0;
// Pompalar için PID katsayıları:
// Kp_pump: Proportional katsayısı, hatanın büyüklüğüne göre tepkiyi belirler.
// Ki_pump: Integral katsayısı, zamanla biriken hataların etkisini düzeltir.
// Kd_pump: Derivative katsayısı, hızlanan hataları engellemeye çalışır.

float integral_pump = 0.0, derivative_pump = 0.0, error_pump = 0.0, previous_error_pump = 0.0;
// Pompalar için PID kontrolü: Bu değişkenler PID algoritmasında kullanılır.
// integral_pump: Hata integralini tutar, hata birikimini düzeltir.
// derivative_pump: Hata türevini tutar, hatadaki değişim hızını ölçer.
// error_pump: Hata değeri, istenen pompaların hız ile mevcut hız arasındaki farktır.
// previous_error_pump: Önceki hata değeri, türev hesaplamasında kullanılır.

float pid_output_pump = 0.0;
// PID çıkışı (pompalar): PID algoritmasından çıkan kontrol sinyalini saklar. Bu sinyal, pompaların hızını kontrol etmek için kullanılır.

float desired_roll = 0.0;
// Hedef roll açısı: Su altı aracının hedeflediği roll açısı. Aracın sağa veya sola eğilmesi gereken açıyı belirtir.

bool fill_mode = true;
// Doldurma modu bayrağı: Su altı aracının su alma (doldurma) modunda olup olmadığını belirler. true: Doldurma modunda, false: Boşaltma modunda.

uint16_t Brushless_Motor_Left = 0;
// Sol fırçasız motor PWM değeri: Sol motor için gönderilecek PWM sinyalinin değerini belirler.

uint16_t Brushless_Motor_Right = 0;
// Sağ fırçasız motor PWM değeri: Sağ motor için gönderilecek PWM sinyalinin değerini belirler.

uint16_t joystick1_Y_ADC = 0;
// Birinci joystick'in Y ekseni ADC değeri: İlk joystick'in Y eksenindeki analog sinyali okur.

uint16_t joystick2_Y_ADC = 0;
// İkinci joystick'in Y ekseni ADC değeri: İkinci joystick'in Y eksenindeki analog sinyali okur.

uint16_t fill_pwm_right = 0;
// Sağ doldurma PWM değeri: Sağ taraftaki doldurma pompası için gönderilecek PWM sinyalinin değerini belirler.

uint16_t fill_pwm_left = 0;
// Sol doldurma PWM değeri: Sol taraftaki doldurma pompası için gönderilecek PWM sinyalinin değerini belirler.

uint16_t drain_pwm_right = 0;
// Sağ boşaltma PWM değeri: Sağ taraftaki boşaltma pompası için gönderilecek PWM sinyalinin değerini belirler.

uint16_t drain_pwm_left = 0;
// Sol boşaltma PWM değeri: Sol taraftaki boşaltma pompası için gönderilecek PWM sinyalinin değerini belirler.

uint16_t dc_motor_pwm = 0;
// DC motor PWM değeri: DC motor için gönderilecek PWM sinyalinin değerini belirler.

float desired_fill_time = 40.0;
// İstenen doldurma süresi (saniye): Su alma işleminin ne kadar süreceğini belirler.

float desired_drain_time = 30.0;
// İstenen boşaltma süresi (saniye): Su boşaltma işleminin ne kadar süreceğini belirler.

bool automation_mode = false;
// Otomasyon modu bayrağı: Aracın otomatik kontrol modunda olup olmadığını belirler. true: Otomasyon modunda, false: Manuel modda.

int depth, roll, v;
// depth: Derinlik, aracın su altındaki derinliği.
// roll: Aracın yatay eğilme açısı.
// v: Aracın hızını temsil eder.

/**************************** MPU6060 ****************************************************************/

uint8_t data;
// Genel amaçlı bayt değişkeni: Veri saklamak için kullanılan 8 bitlik değişken.

uint8_t buffer[2], cuffer[2], tuffer[2];
// I2C veri alışverişi için kullanılan bayt dizileri: MPU6050 sensöründen gelen verileri almak için kullanılır.

int16_t gyro_raw[3], acc_raw[3];
// Ham jiroskop ve ivmeölçer verileri: Sensörlerden gelen ham verileri saklar.

float gyro_cal[3];
// Jiroskop kalibrasyon değerleri: Jiroskop verilerinin doğruluğunu artırmak için kullanılan kalibrasyon verilerini saklar.

int16_t acc_total_vector;
// İvmeölçer verilerinin toplam vektör büyüklüğü: X, Y, Z eksenlerindeki ivme verilerinin toplamını tutar.

float angle_pitch_gyro, angle_roll_gyro;
// Jiroskop verilerine dayalı olarak hesaplanan pitch ve roll açıları.

float angle_pitch_acc, angle_roll_acc;
// İvmeölçer verilerine dayalı olarak hesaplanan pitch ve roll açıları.

float angle_pitch, angle_roll;
// Filtrelenmiş son pitch ve roll açıları: Hem ivmeölçer hem de jiroskop verilerinin birleşimiyle elde edilen açı.

int16_t raw_temp;
// Ham sıcaklık verisi: MPU6050 sensöründen gelen sıcaklık verisini saklar.

float temp;
// İşlenmiş sıcaklık değeri: Sensörden gelen sıcaklık verisinin işlenmiş hali.

int i;
// Genel amaçlı döngü sayacı veya indeks değişkeni.

float x_ivme = 0.0;
// X eksenindeki ivme: MPU6050 sensöründen okunan X eksenindeki ivme verisi.

float z_ivme = 0.0;
// Z eksenindeki ivme: MPU6050 sensöründen okunan Z eksenindeki ivme verisi.

float z_speed = 0.0;
// Z eksenindeki hız: Z eksenindeki ivmenin zamanla entegrasyonu ile hesaplanan hız.

float z_depth = 0.0;
// Derinlik: Z eksenindeki hızın zamanla entegrasyonu ile hesaplanan derinlik.

float prevtime, prevtime1, time1, elapsedtime1, prevtime2, time2, elapsedtime2;
// Zaman ölçümü ve geçen süre hesaplamaları: Zaman ölçümü yaparak ivme, hız ve derinlik hesaplamaları için kullanılan değişkenler.

HAL_StatusTypeDef set_gyro;
// HAL fonksiyonlarının dönüş değerini saklamak için kullanılan enum türü: I2C işlemlerinin başarılı olup olmadığını kontrol eder.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Button_Control(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 'strCopy' adlı bir karakter dizisi tanımlanır. Bu dizi 15 karakter kapasitesine sahip.
// Bu dizi, daha sonra veri kopyalama veya başka bir işlem için kullanılabilir.
char strCopy[15];

// 'map' fonksiyonu, verilen bir değeri (value) belirtilen giriş aralığından (in_min, in_max) çıkış aralığına (out_min, out_max) dönüştürür.
// Bu, örneğin bir sensör değerini farklı bir ölçekteki değere dönüştürmek için kullanılabilir.
uint16_t map(uint16_t value, float in_min, float in_max, float out_min, float out_max) {
    // Giriş değeri ile giriş aralığı arasındaki farkı, çıkış aralığına göre ölçeklendirir.
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/***************************************SUALTI İTİCİ MOTORLARI***********************/

// Motor hızını hesaplayan fonksiyon
void calculate_speed() {
    // Geçen süreyi saniye cinsinden hesapla
    float delta_time = (HAL_GetTick() - prevtime) / 1000.0;
    // Hızı hesapla (ivmenin integrali)
    v += x_ivme * delta_time;
    // Önceki hız değerini güncelle
    previous_x_ivme = x_ivme;
    // Zamanı güncelle
    prevtime = HAL_GetTick();
    // Bu fonksiyon motorun hızını hesaplar. 'delta_time' motorun hareket ettiği süredeki zaman farkını hesaplar.
    // 'v' değeri, ivme ('x_ivme') ile zaman farkının çarpımıyla güncellenir. 'previous_x_ivme' ivme değeri bir sonraki hesaplama için güncellenir.
}

// PID kontrol algoritmasını uygulayan fonksiyon
void motor_pid_control() {
    // Hata hesapla
    error_motor = desired_speed - v;
    // İntegral hesapla
    integral_motor += error_motor;
    // Türev hesapla
    derivative_motor = (error_motor - previous_error_motor) / (HAL_GetTick() - prevtime);
    // PID çıktısını hesapla
    pid_output_motor = Kp_motor * error_motor + Ki_motor * integral_motor + Kd_motor * derivative_motor;
    // Önceki hatayı güncelle
    previous_error_motor = error_motor;
    // Bu fonksiyon PID kontrol algoritmasını uygular. 'error_motor' motorun mevcut hızından istenilen hıza kadar olan farkı temsil eder.
    // Bu hata, integral ve türev bileşenleriyle birleştirilerek PID çıkışı ('pid_output_motor') hesaplanır.
    // 'previous_error_motor' bir sonraki döngüde kullanılmak üzere güncellenir.
}

// PWM çıkışını güncelleyen fonksiyon
void update_pwm() {
    // Eğer PID çıkışı negatifse, PWM değerini azalt
    if (pid_output_motor < 0) {
        TIM2->CCR1 -= (uint16_t)(-pid_output_motor);
    } else {
        // Eğer PID çıkışı pozitifse, PWM değerini artır
        TIM2->CCR1 += (uint16_t)(pid_output_motor);
    }
    // Bu fonksiyon PWM çıkışını günceller. PID çıkışı negatifse PWM değeri azaltılır, pozitifse artırılır.
    // Bu sayede motorun hızını PID kontrol algoritması ile ayarlayarak istenilen hıza ulaşılır.
}

// Su pompası için PID kontrol fonksiyonu
void pump_pid_control() {
    // Roll hatasını hesapla
    float roll_error = desired_roll - roll;
    // İntegral hesapla
    integral_pump += roll_error;
    // Türev hesapla
    derivative_pump = (roll_error - previous_error_pump) / (HAL_GetTick() - prevtime);
    // PID çıktısını hesapla
    pid_output_pump = Kp_pump * roll_error + Ki_pump * integral_pump + Kd_pump * derivative_pump;
    // Önceki hatayı güncelle
    previous_error_pump = roll_error;

    // Butonların durumuna göre PWM değerlerini ayarla
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) {
        // PWM sol doldurma ve sağ doldurma ayarları
        if (pid_output_pump > 0) {
            fill_pwm_left -= (uint16_t)(pid_output_pump);
            fill_pwm_right += (uint16_t)(pid_output_pump);
        } else {
            fill_pwm_left += (uint16_t)(-pid_output_pump);
            fill_pwm_right -= (uint16_t)(-pid_output_pump);
        }
    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET){
        // PWM sol boşaltma ve sağ boşaltma ayarları
        if (pid_output_pump > 0) {
            drain_pwm_left += (uint16_t)(pid_output_pump);
            drain_pwm_right -= (uint16_t)(pid_output_pump);
        } else {
            drain_pwm_left -= (uint16_t)(-pid_output_pump);
            drain_pwm_right += (uint16_t)(-pid_output_pump);
        }
    }
    // PWM değerlerini timer'a uygula
    TIM1->CCR1 = drain_pwm_left;
    TIM1->CCR2 = drain_pwm_right;
    TIM1->CCR3 = fill_pwm_right;
    TIM1->CCR4 = fill_pwm_left;
    // Bu fonksiyon, su pompası sistemindeki PID kontrolünü yapar. 'roll_error' sol ve sağ pompaların yönünü belirler.
    // PID çıkışı, ilgili PWM değerlerini ayarlayarak sol ve sağ pompaların çalışma hızını kontrol eder.
}

// PWM değerlerini hesaplayan fonksiyon
void calculate_pwm_values() {
    // Doldurma Süresi için gerekli voltajı hesapla
    float fill_voltage = (6.0 * 35.56) / desired_fill_time;
    // PWM değeri hesapla
    fill_pwm_left = (uint16_t)((fill_voltage / 12.0) * 1000);
    fill_pwm_right = (uint16_t)((fill_voltage / 12.0) * 1000);
    // Boşaltma Süresi için gerekli voltajı hesapla
    float drain_voltage = (6.0 * 28.80) / desired_drain_time;
    // PWM değeri hesapla
    drain_pwm_left = (uint16_t)((drain_voltage / 12.0) * 1000);
    fill_pwm_right = (uint16_t)((fill_voltage / 12.0) * 1000);
    // DC Motor Süresi için gerekli voltajı hesapla
    float dc_motor_voltage = (12.0 * 20.0) / desired_fill_time;
    // PWM değeri hesapla
    dc_motor_pwm = (uint16_t)((dc_motor_voltage / 12.0) * 1000);
    // Bu fonksiyon, pompa sisteminin doldurma, boşaltma ve DC motor için gerekli PWM değerlerini hesaplar.
    // Hesaplanan voltajlar, belirli bir zaman diliminde gereken PWM değerlerine dönüştürülür.
}



// Mikrosaniye cinsinden zamanı döndüren fonksiyon
// STM32'de bu işlevi SysTick timer sağlar.
// SysTick timer kullanarak milisaniye cinsinden zamanı döndüren fonksiyon
uint32_t getMillis(){
    return HAL_GetTick();
}

// Bu fonksiyon, su altı aracının derinliğini hesaplamak için kullanılır.
// İlk olarak geçen süreyi hesaplar, ardından hız (z_speed) ve derinlik (z_depth) hesaplamaları yapılır.
// 'z_speed' ivme (z_ivme) ile zaman farkının çarpımıyla güncellenir.
// 'z_depth' ise hız (z_speed) ile zaman farkının çarpımıyla hesaplanır.
void calculate_z_depth() {
    float delta_time = (HAL_GetTick() - prevtime1) / 1000.0; // Geçen süreyi saniye cinsinden hesapla

    // İlk integral: hız (z_speed) hesapla
    z_speed += z_ivme * delta_time;

    // İkinci integral: derinlik (z_depth) hesapla
    z_depth += z_speed * delta_time;

    prevtime1 = HAL_GetTick(); // Zamanı güncelle
}

// MPU6050 sensöründen verileri okur ve motor kontrolü için gerekli olan açıyı hesaplar.
// Bu fonksiyon ivmeölçer ve jiroskop verilerini okur ve her bir eksende ilgili açıyı hesaplar.
// Hesaplanan açıları daha sonra aracın hareketini yönlendirmek için kullanır.
void mpu6050() {
    prevtime1 = time1; // Önceki zamanı sakla
    time1 = HAL_GetTick(); // Geçerli zamanı al
    elapsedtime1 = (time1 - prevtime1) * 1000; // Geçen süreyi hesapla

    // İvmeölçer verilerini oku
    tuffer[0] = 0x3B; // İvmeölçer veri başlangıç adresi
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, tuffer, 1, HAL_MAX_DELAY); // Adresi gönder
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, tuffer, 6, HAL_MAX_DELAY); // 6 bayt veri al

    // Ham ivmeölçer verilerini birleştir
    acc_raw[0] = (tuffer[0] << 8 | tuffer[1]); // X ekseni
    acc_raw[1] = (tuffer[2] << 8 | tuffer[3]); // Y ekseni
    acc_raw[2] = (tuffer[4] << 8 | tuffer[5]); // Z ekseni

    // Sıcaklık verisini oku
    buffer[0] = 0x41; // Sıcaklık veri başlangıç adresi
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 1, HAL_MAX_DELAY); // Adresi gönder
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buffer, 2, HAL_MAX_DELAY); // 2 bayt veri al

    // Ham sıcaklık verisini birleştir ve gerçek sıcaklığa çevir
    raw_temp = (buffer[0] << 8 | buffer[1]);
    temp = (raw_temp / 340.0) + 36.53; // MPU6050 datasheet formülüne göre hesaplama

    // Jiroskop verilerini oku
    cuffer[0] = 0x43; // Jiroskop veri başlangıç adresi
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY); // Adresi gönder
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY); // 6 bayt veri al

    // Ham jiroskop verilerini birleştir ve kalibrasyon değerlerini çıkar
    gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]) - gyro_cal[0]; // X ekseni
    gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]) - gyro_cal[1]; // Y ekseni
    gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]) - gyro_cal[2]; // Z ekseni

    // Jiroskop verilerini kullanarak açıları güncelle
    angle_pitch_gyro += gyro_raw[0] * 0.0000611; // Pitch açısı
    angle_roll_gyro += gyro_raw[1] * 0.0000611; // Roll açısı

    // Jiroskop verilerini kullanarak birbirine etki eden açıları güncelle
    angle_pitch_gyro += angle_roll_gyro * sin(gyro_raw[2] * 0.000001066);
    angle_roll_gyro += angle_pitch_gyro * sin(gyro_raw[2] * 0.000001066);

    // İvmeölçer verilerinin toplam vektörünü hesapla
    acc_total_vector = sqrt((acc_raw[0] * acc_raw[0]) + (acc_raw[1] * acc_raw[1]) + (acc_raw[2] * acc_raw[2]));

    // İvmeölçer verilerini kullanarak açıları hesapla
    angle_pitch_acc = asin((float)acc_raw[1] / acc_total_vector) * 57.296; // Pitch açısı
    angle_roll_acc = asin((float)acc_raw[0] / acc_total_vector) * -57.296; // Roll açısı

    // Açıları kalibre et (gerekirse)
    angle_pitch_acc -= 0.00;
    angle_roll_acc -= 0.00;

    // Jiroskop verilerini ivmeölçer verileriyle birleştir
    if (set_gyro) {
        angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004; // Pitch açısı
        angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004; // Roll açısı
    } else {
        angle_pitch = angle_pitch_acc; // İlk okumada sadece ivmeölçer verilerini kullan
        set_gyro = true;
    }

    // Belirli bir süre beklemek için döngü
    while ((HAL_GetTick() - prevtime) * 1000 < 4000);
    prevtime = getMillis(); // Zamanı güncelle
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Kullanıcı kodu başlangıcı (başlangıçta herhangi bir işlem yapılmadı)

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Başlangıçta yapılan işlemler
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Joystick sistemini başlat
  ADC_System_Init(); // Joystick sistemini başlat

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  // PWM sinyallerini başlat (motor kontrolü için)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Sağ motor için PWM başlat
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Sol motor için PWM başlat
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Sol motor için PWM başlat

  // ESC Kalibrasyonu (Motorları kontrol etmek için başlangıç ayarları)
  #if Calibrate
    TIM2->CCR2 = 100;  // Maksimum pulse (2ms)
    HAL_Delay(2000);   // 1 bip sesi bekle
    TIM2->CCR2 = 50;   // Minimum Pulse (1ms)
    HAL_Delay(1000);   // 2 bip sesi bekle
    TIM2->CCR2 = 0;    // Sıfırlama, ADC ile kontrol edilebilsin
  #endif

  #if Calibrate
    TIM2->CCR1 = 100;  // Maksimum pulse (2ms)
    HAL_Delay(2000);   // 1 bip sesi bekle
    TIM2->CCR1 = 50;   // Minimum Pulse (1ms)
    HAL_Delay(1000);   // 2 bip sesi bekle
    TIM2->CCR1 = 0;    // Sıfırlama, ADC ile kontrol edilebilsin
  #endif

/*********************************MPU6050**********************************/
  // MPU6050 Sensör Konfigürasyonu
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
  // Sensörü uyandır ve iç saatini kullanmak üzere ayarla

  data = 0x08;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
  // Jiroskop için hassasiyeti ayarla (±500 derece/saniye)

  data = 0x10;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
  // İvmeölçer için hassasiyeti ayarla (±8g)

  // Jiroskop kalibrasyonu için 2000 ölçüm yap
  for (i = 0; i < 2000; i++) {
      prevtime2 = time2; // Önceki zamanı sakla
      time2 = HAL_GetTick(); // Geçerli zamanı al
      elapsedtime2 = (time2 - prevtime2) * 1000; // Geçen süreyi hesapla

      cuffer[0] = 0x43; // Jiroskop veri başlangıç adresi
      HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY); // Adresi gönder
      HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY); // 6 bayt veri al

      // Ham jiroskop verilerini birleştir
      gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]); // X ekseni
      gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]); // Y ekseni
      gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]); // Z ekseni

      // Kalibrasyon değerlerini topla
      gyro_cal[0] += gyro_raw[0];
      gyro_cal[1] += gyro_raw[1];
      gyro_cal[2] += gyro_raw[2];

      HAL_Delay(3); // 3 milisaniye bekle
  }

  // Kalibrasyon değerlerinin ortalamasını al
  gyro_cal[0] /= 2000;
  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 144-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1440-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 140-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Forward_12V_Motor_Pin|IN2_Backward_12V_Motor_Pin|IN1_Forward_Pumps_Pin|IN_2_Forward_Output_Pin
                          |IN3_Backward_Pumps_Pin|IN4_Backward_Pumps_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_12_V__leri_Pin Button_12_V_Geri_Pin */
  GPIO_InitStruct.Pin = Button_12_V__leri_Pin|Button_12_V_Geri_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Forward_12V_Motor_Pin IN2_Backward_12V_Motor_Pin IN1_Forward_Pumps_Pin IN_2_Forward_Output_Pin
                           IN3_Backward_Pumps_Pin IN4_Backward_Pumps_Pin */
  GPIO_InitStruct.Pin = IN1_Forward_12V_Motor_Pin|IN2_Backward_12V_Motor_Pin|IN1_Forward_Pumps_Pin|IN_2_Forward_Output_Pin
                          |IN3_Backward_Pumps_Pin|IN4_Backward_Pumps_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Automation_Pin */
  GPIO_InitStruct.Pin = Automation_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Automation_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Switch_ON_Pin Switch_ARKA_Pin */
  GPIO_InitStruct.Pin = Switch_ON_Pin|Switch_ARKA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
