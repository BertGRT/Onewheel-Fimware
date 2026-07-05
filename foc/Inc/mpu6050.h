/*
 * mpu6050.h -- Driver MPU6050 (I2C logiciel) + fusion complementaire de l'assiette.
 * Voir mpu6050.c pour les details. Independant de HAL : uniquement CMSIS/registres.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct {
    float pitch;        /* assiette filtree, en degres (0 = a plat)             */
    float pitch_rate;   /* vitesse angulaire d'assiette, deg/s (depuis le gyro) */
    float pitch_acc;    /* assiette brute vue par l'accelerometre (debug)       */
    int16_t ax, ay, az; /* accel brut (debug / calibration)                     */
    int16_t gx, gy, gz; /* gyro brut  (debug / calibration)                     */
    uint8_t ok;         /* 1 = derniere lecture I2C valide                      */
} mpu6050_t;

/* Init du bus I2C logiciel + reveil MPU + config plages/DLPF. Renvoie 1 si ok. */
uint8_t mpu6050_init(void);

/* Mesure du biais gyro (planche IMMOBILE et A PLAT). A appeler a l'engagement. */
void    mpu6050_calibrate(uint16_t samples);

/* Lecture + mise a jour de la fusion. dt en secondes (= OW_LOOP_DT_S). */
void    mpu6050_update(mpu6050_t *out, float dt);

#endif /* MPU6050_H */
