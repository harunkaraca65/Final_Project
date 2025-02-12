#ifndef ADC_H
#define ADC_H

#include "stm32f4xx_hal.h"

// Joystick yapı tanımı
// Bu yapı, joystick'in ADC değerlerini ve kanal bilgilerini tutar.
typedef struct {
    uint16_t Y;                 // Y ekseni ADC değeri (joystick'in Y eksenindeki okunan değer)
    ADC_HandleTypeDef* hadc;    // ADC handle (joystick için kullanılan ADC'nin yapılandırma bilgisi)
    uint32_t Y_Channel;         // Y ekseni ADC kanalı (Y ekseninin bağlı olduğu ADC kanalı)
} Joystick;

// Sensör yapı tanımı
// Bu yapı, sensörlerin ADC değerlerini ve kanal bilgilerini tutar.
typedef struct {
    uint16_t value;             // Sensör ADC değeri (sensörün okunan ADC değeri)
    ADC_HandleTypeDef* hadc;    // ADC handle (sensör için kullanılan ADC'nin yapılandırma bilgisi)
    uint32_t channel;           // Sensör ADC kanalı (sensörün bağlı olduğu ADC kanalı)
} SENSOR;

// Joystickler (dışarıdan erişilebilir global değişkenler)
// Bu değişkenler, programın diğer bölümlerinde joysticklerin erişilebilir olmasını sağlar.
extern Joystick joystick1; // Joystick 1
extern Joystick joystick2; // Joystick 2

// Sensörler (dışarıdan erişilebilir global değişkenler)
// Bu değişkenler, programın diğer bölümlerinde sensörlerin erişilebilir olmasını sağlar.
extern SENSOR sensor1; // Sensör 1
extern SENSOR sensor2; // Sensör 2

// Fonksiyon Prototipleri

// Joystick yapılandırma fonksiyonu
// Bu fonksiyon, joystick'in ADC yapılandırmasını yapar.
void Joystick_Init(Joystick* joystick, ADC_HandleTypeDef* hadc, uint32_t y_channel);

// Sensör yapılandırma fonksiyonu
// Bu fonksiyon, sensörün ADC yapılandırmasını yapar.
void SENSOR_Init(SENSOR* sensor, ADC_HandleTypeDef* hadc, uint32_t channel);

// ADC kanalı okuma fonksiyonu
// Bu fonksiyon, belirtilen ADC kanalından değer okur.
uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel);

// Joystick değerlerini okuma fonksiyonu
// Bu fonksiyon, joystick'in Y ekseninin ADC değerini okur.
void Joystick_Read(Joystick* joystick);

// Sensör değerlerini okuma fonksiyonu
// Bu fonksiyon, sensörün ADC değerini okur.
void SENSOR_Read(SENSOR* sensor);

// Sistem başlatma fonksiyonu
// Bu fonksiyon, tüm joystick ve sensörleri başlatır ve yapılandırır.
void ADC_System_Init(void);

// Sistem güncelleme fonksiyonu
// Bu fonksiyon, joystick ve sensörlerin değerlerini sürekli olarak günceller.
void ADC_System_Update(void);

#endif // ADC_H
