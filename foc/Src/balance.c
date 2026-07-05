/*
 * balance.c
 * -----------------------------------------------------------------------------
 * Boucle d'equilibrage + machine a etats de securite pour la conversion Onewheel.
 *
 * Structure du regulateur (robuste et classique pour engin auto-equilibre) :
 *
 *     erreur   = consigne_angle - angle_mesure
 *     sortie   = Kp*erreur + Ki*integrale(erreur) - Kd*vitesse_gyro
 *
 * Le terme d'amortissement s'appuie DIRECTEMENT sur la vitesse angulaire du gyro
 * (et non sur la derivee numerique de l'angle) : c'est nettement plus stable et
 * moins bruite. La "consigne_angle" vaut 0 (a plat) + trim + eventuel tiltback.
 *
 * SECURITE (dans l'ordre de priorite) :
 *   - pied absent (capteur optique)      -> desengage
 *   - |angle| > fault_angle_max          -> FAULT, coupure immediate
 *   - tension < cutoff_voltage           -> FAULT
 *   - engagement uniquement si a plat     (|angle| <= start_angle_max)
 *   - rampe de sortie limitee            -> pas d'a-coup a l'engagement
 * -----------------------------------------------------------------------------
 */
#include "balance.h"
#include "mpu6050.h"
#include "onewheel_config.h"

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/* Config vivante : valeurs par defaut, ecrasables par la flash / la GUI. */
ow_config_t ow_config = OW_CONFIG_DEFAULTS;

/* ---------------------------------------------------------------------------
 * Persistance flash : ow_config est stockee dans la DERNIERE page de flash
 * (0x0803F800, 2KB). Cette page est hors du code (firmware ~50KB) et hors de
 * l'EEPROM emulee d'EmanuelFeru (pages 0x08010000 / 0x08018000) -> pas de
 * collision, et 'program ... 0x08000000' au reflash ne l'ecrase pas.
 * ------------------------------------------------------------------------- */
#define OW_FLASH_ADDR   0x0803F800u

volatile uint8_t ow_save_req = 0;   /* 1 : la GUI demande une sauvegarde        */
volatile uint8_t ow_save_ack = 0;   /* 0 idle, 1 ok, 2 echec flash, 3 refuse    */

static uint16_t ow_crc16(const uint8_t *d, int n) {
    uint16_t c = 0xFFFF;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
    }
    return c;
}

void ow_config_load(void) {
    const ow_config_t *f = (const ow_config_t *)OW_FLASH_ADDR;
    if (f->magic == OW_CONFIG_MAGIC) {
        uint16_t crc = ow_crc16((const uint8_t *)f, sizeof(ow_config_t) - 2);
        if (crc == f->crc) ow_config = *f;   /* sinon : on garde les defauts compiles */
    }
}

uint8_t ow_config_save(void) {
    ow_config.magic = OW_CONFIG_MAGIC;
    ow_config.crc   = ow_crc16((const uint8_t *)&ow_config, sizeof(ow_config_t) - 2);

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef er = {0};
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = OW_FLASH_ADDR;
    er.NbPages     = 1;
    uint32_t pageErr = 0;
    if (HAL_FLASHEx_Erase(&er, &pageErr) != HAL_OK) { HAL_FLASH_Lock(); return 2; }

    const uint16_t *src = (const uint16_t *)&ow_config;
    int words = (sizeof(ow_config_t) + 1) / 2;
    for (int i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              OW_FLASH_ADDR + (uint32_t)i * 2, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 2;
        }
    }
    HAL_FLASH_Lock();
    return 1;
}

static ow_state_t     state = OW_STATE_IDLE;
static ow_telemetry_t tlm;
static mpu6050_t      imu;

static float   integral   = 0.0f;
static int16_t output_cur = 0;     /* commande courante (pour la rampe)         */
static float   soft       = 0.0f;  /* enveloppe soft-start 0->1 a l'engagement    */

/* --- Feedback fourni par le firmware principal (batterie, courant, vitesse) --*/
static int16_t fb_voltage = 400;   /* V x10, valeur d'attente                   */
static int16_t fb_current = 0;
static int16_t fb_rpm     = 0;

void balance_set_feedback(int16_t voltage_x10, int16_t current_x10, int16_t rpm) {
    fb_voltage = voltage_x10;
    fb_current = current_x10;
    fb_rpm     = rpm;
}

/* Footpad LOGICIEL (override GUI/SWD). 0 = neutre. En OU avec le materiel. */
volatile uint8_t ow_footpad = 0;

/* Repose-pied : lecture GPIO (active_low) OU override logiciel. */
static uint8_t footpad_pressed(void) {
    uint8_t raw = (OW_FOOTPAD_PORT->IDR >> OW_FOOTPAD_PIN) & 1u;
    uint8_t hw  = ow_config.footpad_active_low ? (raw == 0) : (raw != 0);
    return (hw || ow_footpad) ? 1u : 0u;
}
static void footpad_gpio_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    volatile uint32_t *cr = (OW_FOOTPAD_PIN < 8) ? &OW_FOOTPAD_PORT->CRL
                                                 : &OW_FOOTPAD_PORT->CRH;
    uint8_t s = (OW_FOOTPAD_PIN & 7) * 4;
    *cr &= ~(0xFu << s);
    *cr |=  (0x8u << s);                   /* entree avec pull (CNF=10, MODE=00) */
    if (ow_config.footpad_active_low) OW_FOOTPAD_PORT->BSRR = (1u << OW_FOOTPAD_PIN); /* pull-up */
    else                              OW_FOOTPAD_PORT->BRR  = (1u << OW_FOOTPAD_PIN);
}

void balance_init(void) {
    ow_config_load();          /* charge les reglages sauves (flash) si valides */
    footpad_gpio_init();
    mpu6050_init();
    mpu6050_calibrate(200);   /* calib au BOOT (moteurs off, planche a plat/immobile) */
    state = OW_STATE_IDLE;
    integral = 0.0f;
    output_cur = 0;
}

/* Limite de variation de la commande (rampe anti a-coup). */
static int16_t ramp_towards(int16_t cur, int16_t target, int16_t step) {
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

static void enter_idle(void) {
    state = OW_STATE_IDLE;
    integral = 0.0f;
    output_cur = 0;
    soft = 0.0f;
}

int16_t balance_update(void) {
    mpu6050_update(&imu, OW_LOOP_DT_S);

    const uint8_t  foot = footpad_pressed();
    const float    ang  = imu.pitch;
    const float    fault_ang = ow_config.fault_angle_max / 10.0f;
    const float    start_ang = ow_config.start_angle_max / 10.0f;

    /* --- Securites globales, valables dans tous les etats roulants --- */
    int16_t target = 0;

    switch (state) {
    /* --------------------------------------------------------------------- */
    case OW_STATE_IDLE:
        output_cur = 0;
        if (foot && imu.ok) {              /* calib deja faite au boot -> passage ARME */
            state = OW_STATE_ARMED;
        }
        break;

    /* --------------------------------------------------------------------- */
    case OW_STATE_ARMED:
        output_cur = 0;
        if (!foot) { enter_idle(); break; }
        /* Engagement DES que la planche entre dans la fenetre autour de 0.
         * Pas d'attente : le soft-start (enveloppe soft 0->1) assure la montee
         * progressive du couple, donc pas d'a-coup meme en engageant tot. */
        if (ang > -start_ang && ang < start_ang) {
            integral = 0.0f;
            soft     = 0.0f;              /* demarre l'enveloppe progressive */
            state    = OW_STATE_RIDING;
        }
        break;

    /* --------------------------------------------------------------------- */
    case OW_STATE_RIDING:
    case OW_STATE_TILTBACK: {
        /* ---- Conditions de coupure ---- */
        if (!foot)                       { enter_idle(); break; }
        if (ang > fault_ang || ang < -fault_ang) { state = OW_STATE_FAULT; break; }
        if (fb_voltage < ow_config.cutoff_voltage) { state = OW_STATE_FAULT; break; }

        /* ---- Consigne d'assiette (trim + tiltback eventuel) ---- */
        float setpoint = ow_config.setpoint_trim / 10.0f;

        uint8_t tb = 0;
        int16_t spd = fb_rpm < 0 ? -fb_rpm : fb_rpm;
        if (ow_config.tiltback_speed > 0 && spd > ow_config.tiltback_speed) tb = 1;
        if (fb_voltage < ow_config.lowbat_voltage) tb = 1;
        if (tb) {
            /* releve le nez dans le sens de la marche pour freiner le rider */
            float dir = (fb_rpm >= 0) ? 1.0f : -1.0f;
            setpoint += dir * (ow_config.tiltback_angle / 10.0f);
            state = OW_STATE_TILTBACK;
        } else {
            state = OW_STATE_RIDING;
        }

        /* ---- Erreur + zone morte ---- */
        float err = setpoint - ang;
        float db  = ow_config.deadband / 10.0f;
        if (err < db && err > -db) err = 0.0f;

        /* ---- PID (D = amortissement gyro) ---- */
        float kp = ow_config.kp / 100.0f;
        float ki = ow_config.ki / 100.0f;
        float kd = ow_config.kd / 100.0f;

        integral += err * OW_LOOP_DT_S;
        /* anti-windup : borne l'integrale */
        float imax = (ow_config.output_max / (ki > 0.01f ? ki : 0.01f));
        if (integral >  imax) integral =  imax;
        if (integral < -imax) integral = -imax;

        /* Terme P progressif (expo) : gain qui croit avec l'angle.
         * out_p = Kp*err*(1 + expo*|err|). expo=0 -> lineaire classique.
         * Doux au centre (stable), mordant aux grands angles (couple/frein). */
        float e_abs = err < 0.0f ? -err : err;
        float expo  = ow_config.angle_expo / 1000.0f;          /* 1/deg */
        float p_term = kp * err * (1.0f + expo * e_abs);

        float out = p_term + ki * integral - kd * imu.pitch_rate;

        /* Soft-start : enveloppe 0->1 sur start_ramp ms des l'engagement.
         * Multiplie TOUTE la commande (P, I et D) -> couple qui monte en douceur,
         * plus d'a-coup desequilibrant a l'entree en RIDING. Independant de la
         * rampe de conduite (output_ramp), qui reste vive pour le frein. */
        if (soft < 1.0f) {
            float sramp_s = (ow_config.start_ramp > 0) ? (ow_config.start_ramp / 1000.0f) : 0.001f;
            soft += OW_LOOP_DT_S / sramp_s;
            if (soft > 1.0f) soft = 1.0f;
        }
        out *= soft;

        if (ow_config.output_invert) out = -out;

        int16_t omax = ow_config.output_max;
        if (out >  omax) out =  omax;
        if (out < -omax) out = -omax;

        target = (int16_t)out;
        break;
    }

    /* --------------------------------------------------------------------- */
    case OW_STATE_FAULT:
    default:
        output_cur = 0;
        integral = 0.0f;
        /* on ne repart qu'apres avoir tout relache ET etre revenu a plat */
        if (!foot && ang > -start_ang && ang < start_ang) enter_idle();
        break;
    }

    /* Rampe de sortie (sauf FAULT/IDLE ou l'on force 0 tout de suite). */
    if (state == OW_STATE_RIDING || state == OW_STATE_TILTBACK) {
        output_cur = ramp_towards(output_cur, target, ow_config.output_ramp);
    } else {
        output_cur = 0;
    }

    /* --- Telemetrie --- */
    tlm.pitch      = imu.pitch;
    tlm.pitch_rate = imu.pitch_rate;
    tlm.output     = output_cur;
    tlm.state      = (int16_t)state;
    tlm.voltage    = fb_voltage;
    tlm.current    = fb_current;
    tlm.footpad    = foot;
    tlm.rpm        = fb_rpm;

    return output_cur;
}

ow_state_t balance_state(void) { return state; }
const ow_telemetry_t *balance_telemetry(void) { return &tlm; }
