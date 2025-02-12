#include "ADC.h"

// Joystick nesneleri (global)
Joystick joystick1; // Joystick 1 nesnesi
Joystick joystick2; // Joystick 2 nesnesi

// Sensör nesneleri (global)
SENSOR sensor1; // Sensör 1 nesnesi
SENSOR sensor2; // Sensör 2 nesnesi

// Joystick yapılandırma fonksiyonu
// Bu fonksiyon, joystick'lerin ADC yapılandırmasını yapar.
void Joystick_Init(Joystick* joystick, ADC_HandleTypeDef* hadc, uint32_t y_channel) {
    joystick->hadc = hadc; // ADC handle'ını joystick'e atar
    joystick->Y_Channel = y_channel; // Joystick Y ekseninin kanalını atar
    joystick->Y = 0; // Başlangıçta Y ekseni değerini sıfırlar
}

// Sensör yapılandırma fonksiyonu
// Bu fonksiyon, sensörlerin ADC yapılandırmasını yapar.
void SENSOR_Init(SENSOR* sensor, ADC_HandleTypeDef* hadc, uint32_t channel) {
    sensor->hadc = hadc; // ADC handle'ını sensöre atar
    sensor->channel = channel; // Sensörün ADC kanalını atar
    sensor->value = 0; // Başlangıçta sensör değerini sıfırlar
}

// ADC kanalı okuma fonksiyonu
// Bu fonksiyon, belirli bir ADC kanalından veri okur.
uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfigPrivate = {0}; // ADC kanal yapılandırma yapısı
    sConfigPrivate.Channel = channel; // Okunacak kanal belirlenir

    // Kanal yapılandırmasını yap
    if (HAL_ADC_ConfigChannel(hadc, &sConfigPrivate) != HAL_OK) {
        return 0; // Hata durumunda 0 döndür
    }

    // ADC'yi başlat
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return 0; // ADC başlatma hatası
    }

    // ADC dönüşümünü bekle
    if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) != HAL_OK) {
        HAL_ADC_Stop(hadc); // Hata durumunda ADC'yi durdur
        return 0; // Dönüşüm hatası durumunda 0 döndür
    }

    // ADC değerini al
    uint16_t value = HAL_ADC_GetValue(hadc);

    // ADC'yi durdur
    HAL_ADC_Stop(hadc);

    return value; // ADC değerini döndür
}

// Joystick değerlerini okuma fonksiyonu
// Bu fonksiyon, joystick'in Y ekseninin ADC değerini okur.
void Joystick_Read(Joystick* joystick) {
    joystick->Y = Read_ADC_Channel(joystick->hadc, joystick->Y_Channel); // Y eksenini oku
}

// Sensör değerlerini okuma fonksiyonu
// Bu fonksiyon, sensörün ADC değerini okur.
void SENSOR_Read(SENSOR* sensor) {
    sensor->value = Read_ADC_Channel(sensor->hadc, sensor->channel); // Sensör değerini oku
}

// Sistem başlatma fonksiyonu
// Bu fonksiyon, tüm joystick ve sensörleri başlatır ve yapılandırır.
void ADC_System_Init(void) {
    extern ADC_HandleTypeDef hadc1; // Kullanılacak ADC tanımları
    extern ADC_HandleTypeDef hadc2;
    extern ADC_HandleTypeDef hadc3;

    // Joystickler için yapılandırma
    Joystick_Init(&joystick1, &hadc2, ADC_CHANNEL_10); // Joystick 1 için yapılandırma
    Joystick_Init(&joystick2, &hadc3, ADC_CHANNEL_12); // Joystick 2 için yapılandırma

    // Sensörler için yapılandırma
    SENSOR_Init(&sensor1, &hadc1, ADC_CHANNEL_11); // Sensör 1 için yapılandırma
    SENSOR_Init(&sensor2, &hadc1, ADC_CHANNEL_13); // Sensör 2 için yapılandırma
}

// Sistem güncelleme fonksiyonu
// Bu fonksiyon, joystick ve sensörlerin değerlerini günceller.
void ADC_System_Update(void) {
    // Joystick değerlerini güncelle
    Joystick_Read(&joystick1); // Joystick 1'in değerini oku
    Joystick_Read(&joystick2); // Joystick 2'nin değerini oku

    // Sensör değerlerini güncelle
    SENSOR_Read(&sensor1); // Sensör 1'in değerini oku
    SENSOR_Read(&sensor2); // Sensör 2'nin değerini oku
}
