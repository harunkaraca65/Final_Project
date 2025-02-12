#include "main.h"
#include "ADC.h"

void Button_Control(void) {
    // Su sensörlerinin değerlerini kontrol et
    // Eğer sensör 1'in değeri 4000'in altındaysa, PA0 butonunun basımı etkili olacak
    // Eğer sensör 2'nin değeri 4000'in altındaysa, PA1 butonunun basımı etkili olacak
    if (sensor1.value < 4000 || sensor2.value < 4000) {
        // Eğer PA0 butonuna basıldıysa (PA0 LOW), PA1'i dikkate almayacağız
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) {
            // Buton 1 (PA0) basılı (PA0 LOW), PA1 etkisiz

            // PA2 (12V DC motor ENA) HIGH yaparak motoru çalıştır
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  // PA2 HIGH -> Motor aktif (Saat yönünde)

            // PA3 (ENB) HIGH yaparak motorun saat yönünde dönmesini sağla
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // PA3 HIGH -> Motor saat yönünde döner

            // PA4 ve PA5, sağ ve sol doldurma motorlarının ENA'larını aktif yapar
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // PA4 HIGH -> Sağ doldurma motoru aktif
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // PA5 HIGH -> Sol doldurma motoru aktif

            // PA6 ve PA7 boşaltma motorlarının ENA'larını pasif yapar
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // PA6 LOW -> Sağ boşaltma motoru pasif
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // PA7 LOW -> Sol boşaltma motoru pasif
        }
    }
    // Eğer PA1 butonuna basıldıysa ve ilgili switch'e basılmadıysa, PA0'ı dikkate almayacağız
    else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET) {
        // Buton 2 (PA1) basılı (PA1 LOW), PA0 etkisiz

        // PA2 (12V DC motor ENA) HIGH yaparak motoru çalıştır
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // PA2 HIGH -> Motor aktif (Saat yönünün tersine)

        // PA3 (ENB) LOW yaparak motorun saat yönünün tersine dönmesini sağla
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // PA3 LOW -> Motor saat yönünün tersine döner

        // PA4 ve PA5, sağ ve sol doldurma motorlarının ENA'larını pasif yapar
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // PA4 LOW -> Sağ doldurma motoru pasif
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // PA5 LOW -> Sol doldurma motoru pasif

        // PA6 ve PA7 boşaltma motorlarının ENA'larını aktif yapar
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // PA6 HIGH -> Sağ boşaltma motoru aktif
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // PA7 HIGH -> Sol boşaltma motoru aktif
    }
    else {
        // Her iki buton da serbest bırakıldıysa (PA0 LOW, PA1 LOW)
        // Motor ve pompaların hepsi pasif durumda
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // PA2 LOW -> Motor pasif
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // PA3 LOW -> Motor yönü pasif
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // PA4 LOW -> Sağ doldurma motoru pasif
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // PA5 LOW -> Sol doldurma motoru pasif
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // PA6 LOW -> Sağ boşaltma motoru pasif
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // PA7 LOW -> Sol boşaltma motoru pasif
    }
}


