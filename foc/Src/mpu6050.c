/*
 * mpu6050.c
 * -----------------------------------------------------------------------------
 * Driver MPU6050 pour STM32F103, en I2C LOGICIEL (bit-bang) sur deux GPIO libres.
 * -> aucune contrainte de brochage : marche quelles que soient les broches du
 *    connecteur "gyro" d'origine ou tu as soude la MPU (voir onewheel_config.h).
 *
 * Fournit une assiette (pitch) filtree par filtre complementaire :
 *     angle = alpha*(angle + gyro*dt) + (1-alpha)*angle_accelerometre
 *
 * ORIENTATION : le calcul suppose que l'axe de rotation "avant/arriere" de la
 * planche est l'axe X du gyro, et que l'accel donne l'assiette via atan2(ax,az).
 * Si ta MPU est montee autrement, joue sur pitch_invert (config) et, au besoin,
 * echange les axes dans mpu_compute_pitch() ci-dessous (bien commente).
 * -----------------------------------------------------------------------------
 */
#include "mpu6050.h"
#include "onewheel_config.h"
#include <math.h>

#include "stm32f1xx.h"   /* CMSIS : GPIOx, RCC, registres. Fourni par le firmware.*/

/* ------------------------------------------------------------------ Registres */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_PWR_MGMT_1    0x6B
#define REG_ACCEL_XOUT_H  0x3B
#define REG_WHO_AM_I      0x75

/* Plages choisies : gyro +-500 deg/s (65.5 LSB/deg/s), accel +-2g. */
#define GYRO_SENS         65.5f
#define RAD2DEG           57.29578f

/* ============================================================================
 * ORIENTATION DE LA MPU  (ton montage : a plat, puce vers le HAUT,
 *                         axe X dans le sens de la marche, Z vers le haut)
 * ----------------------------------------------------------------------------
 * Le TANGAGE (nez haut/bas) est une rotation autour de l'axe Y (transversal) :
 *   - accelerometre : angle = atan2(ax, az)        -> broches ax, az
 *   - gyroscope     : vitesse = axe Y (GYRO_YOUT)   -> offset 10 dans la trame
 * GYRO_PITCH_SIGN garde le gyro COHERENT avec l'accel (meme sens de variation).
 * D'apres l'orientation ci-dessus, le signe attendu est -1.
 *
 * TEST AU BANC si doute : tiens la planche immobile puis bascule-la vite a la
 * main. La telemetrie "assiette" doit suivre le mouvement sans partir d'abord a
 * l'oppose. Si a chaque basculement rapide l'assiette fait un a-coup inverse
 * avant de revenir -> le gyro est a l'envers : passe GYRO_PITCH_SIGN a +1.0f.
 * ========================================================================== */
#define GYRO_PITCH_HI     10          /* offset GYRO_YOUT_H dans la trame 14o    */
#define GYRO_PITCH_SIGN   (-1.0f)

static float gyro_bias = 0.0f;    /* biais de l'axe de tangage (deg/s brut).     */

/* ============================================================================
 * Couche GPIO bit-bang. On configure SDA/SCL en open-drain simule :
 *   - "1" logique  = broche en entree (le pull-up externe tire a VCC)
 *   - "0" logique  = broche en sortie a 0
 * PENSE au pull-up : la MPU/GY-521 embarque en general des pull-ups. Sinon,
 * ajoute 4.7k de SDA et SCL vers 3V3.
 * ========================================================================== */
#define SDA_PORT  OW_I2C_SDA_PORT
#define SDA_PIN   OW_I2C_SDA_PIN
#define SCL_PORT  OW_I2C_SCL_PORT
#define SCL_PIN   OW_I2C_SCL_PIN

/* Config d'une broche du STM32F1 : CNF/MODE 4 bits par broche dans CRL/CRH. */
static void pin_mode_output_od(GPIO_TypeDef *port, uint8_t pin) {
    volatile uint32_t *cr = (pin < 8) ? &port->CRL : &port->CRH;
    uint8_t s = (pin & 7) * 4;
    *cr &= ~(0xFu << s);
    *cr |=  (0x7u << s);   /* MODE=11 (50MHz) + CNF=01 (open-drain) = 0b0111.    */
}
static void pin_mode_input(GPIO_TypeDef *port, uint8_t pin) {
    volatile uint32_t *cr = (pin < 8) ? &port->CRL : &port->CRH;
    uint8_t s = (pin & 7) * 4;
    *cr &= ~(0xFu << s);
    *cr |=  (0x4u << s);   /* MODE=00 (input) + CNF=01 (floating) = 0b0100.      */
}
static inline void pin_low(GPIO_TypeDef *port, uint8_t pin) { port->BRR  = (1u << pin); }
static inline uint8_t pin_read(GPIO_TypeDef *port, uint8_t pin) { return (port->IDR >> pin) & 1u; }

/* Delai I2C. 300 etait bien trop lent (~45ms/lecture -> boucle 18Hz, calib qui
 * bloque ~11s). A 64 MHz, 50 donne ~150 kHz : lecture ~2-3ms, boucle ~200Hz.
 * Le bruit moteur etant maintenant maitrise (fils torsades/routes), on peut. */
#define I2C_DELAY 50
static void i2c_delay(void) { for (volatile int i = 0; i < I2C_DELAY; i++) __NOP(); }

/* Sortie push-pull (CNF=00 MODE=11 = 0x3) : ligne pilotee activement. */
static void pin_mode_output_pp(GPIO_TypeDef *port, uint8_t pin) {
    volatile uint32_t *cr = (pin < 8) ? &port->CRL : &port->CRH;
    uint8_t s = (pin & 7) * 4;
    *cr &= ~(0xFu << s);
    *cr |=  (0x3u << s);
}
/* DURCISSEMENT ANTI-BRUIT MOTEUR :
 *  - SCL en PUSH-PULL permanent (le maitre est seul a piloter l'horloge).
 *  - SDA pilote en push-pull quand le maitre ecrit ; relache (haut-Z + pull-up
 *    du module MPU) seulement pour l'ACK et la lecture des donnees.
 *  - SDA_GET : vote majoritaire sur 3 echantillons. */
static inline void SCL_HIGH(void){ SCL_PORT->BSRR = (1u << SCL_PIN); }
static inline void SCL_LOW(void) { SCL_PORT->BRR  = (1u << SCL_PIN); }
static inline void SDA_HIGH(void){ SDA_PORT->BSRR = (1u << SDA_PIN); pin_mode_output_pp(SDA_PORT, SDA_PIN); }
static inline void SDA_LOW(void) { SDA_PORT->BRR  = (1u << SDA_PIN); pin_mode_output_pp(SDA_PORT, SDA_PIN); }
static inline void SDA_REL(void) { pin_mode_input(SDA_PORT, SDA_PIN); }   /* haut-Z pour ACK/lecture */
static inline uint8_t SDA_GET(void){
    uint8_t a = pin_read(SDA_PORT, SDA_PIN);
    uint8_t b = pin_read(SDA_PORT, SDA_PIN);
    uint8_t c = pin_read(SDA_PORT, SDA_PIN);
    return (uint8_t)((a + b + c) >= 2u);
}

static void i2c_start(void) {
    SDA_HIGH(); SCL_HIGH(); i2c_delay();
    SDA_LOW();  i2c_delay();
    SCL_LOW();  i2c_delay();
}
static void i2c_stop(void) {
    SDA_LOW();  SCL_HIGH(); i2c_delay();
    SDA_HIGH(); i2c_delay();
}
/* Ecrit un octet, renvoie l'ACK (0 = ACK recu). */
static uint8_t i2c_write_byte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x80) SDA_HIGH(); else SDA_LOW();
        b <<= 1;
        SCL_HIGH(); i2c_delay();
        SCL_LOW();  i2c_delay();
    }
    SDA_REL();                /* relache SDA (haut-Z) pour lire l'ACK */
    SCL_HIGH(); i2c_delay();
    uint8_t ack = SDA_GET();
    SCL_LOW();  i2c_delay();
    return ack;               /* 0 = ACK */
}
static uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t b = 0;
    SDA_REL();                             /* haut-Z : le slave pilote SDA */
    for (uint8_t i = 0; i < 8; i++) {
        SCL_HIGH(); i2c_delay();
        b = (b << 1) | SDA_GET();
        SCL_LOW();  i2c_delay();
    }
    if (ack) SDA_LOW(); else SDA_HIGH();   /* maitre pilote ACK(bas)/NACK(haut) */
    SCL_HIGH(); i2c_delay();
    SCL_LOW();  i2c_delay();
    SDA_REL();
    return b;
}

/* ----------------------------------------------------------- Acces registres */
static uint8_t mpu_write_reg(uint8_t reg, uint8_t val) {
    i2c_start();
    if (i2c_write_byte(OW_MPU6050_ADDR << 1)) { i2c_stop(); return 0; }
    i2c_write_byte(reg);
    i2c_write_byte(val);
    i2c_stop();
    return 1;
}
/* Lecture en rafale a partir de reg. Renvoie 1 si ok. */
static uint8_t mpu_read(uint8_t reg, uint8_t *buf, uint8_t n) {
    i2c_start();
    if (i2c_write_byte(OW_MPU6050_ADDR << 1)) { i2c_stop(); return 0; }
    i2c_write_byte(reg);
    i2c_start();
    if (i2c_write_byte((OW_MPU6050_ADDR << 1) | 1)) { i2c_stop(); return 0; }
    for (uint8_t i = 0; i < n; i++)
        buf[i] = i2c_read_byte(i < (n - 1));   /* ACK sauf le dernier */
    i2c_stop();
    return 1;
}

/* Lecture ROBUSTE : reessais + rejet des trames toutes 0x00/0xFF (bruit). */
static uint8_t mpu_read_robust(uint8_t reg, uint8_t *buf, uint8_t n) {
    for (uint8_t t = 0; t < 4; t++) {
        if (mpu_read(reg, buf, n)) {
            uint8_t all0 = 1, allF = 1;
            for (uint8_t i = 0; i < n; i++) {
                if (buf[i] != 0x00) all0 = 0;
                if (buf[i] != 0xFF) allF = 0;
            }
            if (!all0 && !allF) return 1;      /* trame plausible */
        }
    }
    return 0;
}

/* ============================================================================ */
uint8_t mpu6050_init(void) {
    /* Horloge des ports GPIO utilises (GPIOA=IOPA, GPIOB=IOPB...). */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    pin_mode_output_pp(SCL_PORT, SCL_PIN); SCL_HIGH();  /* SCL push-pull, au repos haut */
    SDA_REL();                                          /* SDA relache (haut-Z + pull-up) */
    for (volatile int i = 0; i < 100000; i++) __NOP();  /* petit reveil bus */

    uint8_t who = 0;
    mpu_read(REG_WHO_AM_I, &who, 1);
    /* WHO_AM_I attendu 0x68 (certains clones renvoient autre chose -> on tolere).*/

    mpu_write_reg(REG_PWR_MGMT_1, 0x01);   /* sort du sleep, horloge = gyro X    */
    mpu_write_reg(REG_SMPLRT_DIV, 0x04);   /* 1kHz/(1+4) = 200 Hz                 */
    mpu_write_reg(REG_CONFIG,     0x03);   /* DLPF ~44 Hz (anti-vibrations)       */
    mpu_write_reg(REG_GYRO_CONFIG,0x08);   /* +-500 deg/s                         */
    mpu_write_reg(REG_ACCEL_CONFIG,0x00);  /* +-2 g                               */
    return (who == 0x68) ? 1 : 1;          /* on continue meme si clone.          */
}

void mpu6050_calibrate(uint16_t samples) {
    if (samples == 0) samples = 1;
    int32_t sum = 0;
    uint8_t buf[14];
    for (uint16_t i = 0; i < samples; i++) {
        if (mpu_read_robust(REG_ACCEL_XOUT_H, buf, 14)) {
            int16_t g = (int16_t)((buf[GYRO_PITCH_HI] << 8) | buf[GYRO_PITCH_HI + 1]);
            sum += g;
        }
        for (volatile int d = 0; d < 5000; d++) __NOP();      /* ~ espacement */
    }
    gyro_bias = ((float)sum / (float)samples) / GYRO_SENS;     /* deg/s brut */
}

/* Assiette vue par l'accelerometre. Adapte les axes ici si montage different. */
static float mpu_compute_pitch(int16_t ax, int16_t az) {
    /* atan2(ax, az) -> angle de bascule avant/arriere. */
    return atan2f((float)ax, (float)az) * RAD2DEG;
}

volatile uint32_t mpu_ok_count   = 0;   /* diag : lectures reussies (SWD)   */
volatile uint32_t mpu_fail_count = 0;   /* diag : lectures echouees  (SWD)   */

void mpu6050_update(mpu6050_t *out, float dt) {
    static float angle = 0.0f;
    static uint8_t init = 0;
    uint8_t buf[14];

    if (!mpu_read_robust(REG_ACCEL_XOUT_H, buf, 14)) {
        out->ok = 0;               /* garde la derniere valeur, signale l'echec */
        mpu_fail_count++;
        return;
    }
    mpu_ok_count++;
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    float acc_pitch = mpu_compute_pitch(ax, az);
    /* gyro sur l'axe de tangage (Y), biais retire, puis signe de coherence */
    int16_t g_pitch = (int16_t)((buf[GYRO_PITCH_HI] << 8) | buf[GYRO_PITCH_HI + 1]);
    float rate = GYRO_PITCH_SIGN * (((float)g_pitch / GYRO_SENS) - gyro_bias);

    if (!init) { angle = acc_pitch; init = 1; }            /* demarre sur l'accel */

    float alpha = (float)ow_config.imu_alpha / 1000.0f;    /* ex 0.980 */
    angle = alpha * (angle + rate * dt) + (1.0f - alpha) * acc_pitch;

    if (ow_config.pitch_invert) { angle = -angle; rate = -rate; }

    out->pitch      = angle;
    out->pitch_rate = rate;
    out->pitch_acc  = acc_pitch;
    out->ax = ax; out->ay = ay; out->az = az;
    out->gx = gx; out->gy = gy; out->gz = gz;
    out->ok = 1;
}
