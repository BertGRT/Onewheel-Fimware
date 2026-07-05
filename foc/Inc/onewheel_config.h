/*
 * onewheel_config.h
 * -----------------------------------------------------------------------------
 * Configuration + structure de parametres pour la conversion HOVERBOARD -> ONEWHEEL.
 *
 * Ce fichier s'ajoute au firmware "hoverboard-firmware-hack-FOC" (EmanuelFeru).
 * Tous les reglages "roulants" (PID, angles, limites, tiltback...) sont regroupes
 * dans la struct ow_config_t : c'est CET objet que l'interface web lit/ecrit et
 * qui est sauvegarde en flash. Les #define ci-dessous sont les valeurs par defaut
 * (compilees en dur) + le cablage bas-niveau (broches I2C, capteur optique).
 *
 * IMPORTANT SECURITE : le premier essai se fait TOUJOURS roues en l'air, planche
 * sur un support. Verifier le sens de rattrapage AVANT de monter dessus.
 * -----------------------------------------------------------------------------
 */
#ifndef ONEWHEEL_CONFIG_H
#define ONEWHEEL_CONFIG_H

#include <stdint.h>

/* =============================================================================
 * 1. FREQUENCE DE LA BOUCLE D'EQUILIBRAGE
 * ========================================================================== */
#define OW_LOOP_HZ            200          /* Hz. Boucle balance = 200 Hz (5 ms). */
#define OW_LOOP_DT_MS         (1000 / OW_LOOP_HZ)
#define OW_LOOP_DT_S          (1.0f / (float)OW_LOOP_HZ)

/* =============================================================================
 * 2. CABLAGE MPU6050 (I2C logiciel / bit-bang)
 * -----------------------------------------------------------------------------
 * La MPU6050 est en I2C, soudee sur le connecteur "UARTL1" (sideboard gauche
 * debranchee). Brochage CONFIRME EN VRAI : WHO_AM_I=0x68 lu via bit-bang I2C
 * piloté par le ST-Link, dans ce sens exact :
 *     SDA -> PA2
 *     SCL -> PA3
 * I2C LOGICIEL (bit-bang) sur PA2/PA3 (deux broches libres, non chargees par la
 * carte). NB : les anciennes pastilles UARTR1 (PA1/PC2) sont tenues a 0 par la
 * carte -> inutilisables, ne pas y revenir.
 *
 * Rappel alim : MPU en 3V3 (pas 5V), GND bien soudee (une masse froide = pas
 * d'ACK meme si les pull-ups se voient).
 * ========================================================================== */
#define OW_I2C_SDA_PORT      GPIOA
#define OW_I2C_SDA_PIN       2            /* PA2 (SDA) - confirme WHO_AM_I=0x68  */
#define OW_I2C_SCL_PORT      GPIOA
#define OW_I2C_SCL_PIN       3            /* PA3 (SCL)                           */
#define OW_MPU6050_ADDR      0x68         /* confirme (AD0=GND).                 */

/* =============================================================================
 * 3. CAPTEUR OPTIQUE (repose-pied / footpad)
 * -----------------------------------------------------------------------------
 * L'inter optique d'origine indique "un pied est sur la planche". C'est la
 * securite d'engagement : pas de pied -> pas d'equilibrage.
 *
 * >>> A CONFIRMER : c'est le SEUL brochage encore inconnu. <<<
 * Repere sur quelle broche du GD32 arrive la sortie du capteur optique (ou, si tu
 * l'as recable toi-meme, choisis un GPIO LIBRE et mets-le ici). EVITE :
 *   PA9/PA10 (pris par la MPU), PA13/PA14 (SWD/ST-Link), PA2/PA3 & PB10/PB11
 *   (USART2/USART3 reserves plus tard pour l'USB-TTL de reglage).
 * Le niveau logique (actif haut/bas) se regle a chaud via footpad_active_low.
 * Placeholder : PB5 -> A REMPLACER par ta broche reelle.
 *
 * BANC (phase 2) : footpad pas encore cable -> footpad LOGICIEL (variable
 * ow_footpad, pilotable par SWD) pour armer l'engagement roues en l'air en
 * securite. Passe a 0 quand le vrai capteur sera cable.
 * ========================================================================== */
/* Footpad materiel sur PB11 (pastille data UARTR1), actif bas (switch vers GND,
 * pull-up interne). Le footpad LOGICIEL (ow_footpad, via GUI) reste en OU logique. */
#define OW_FOOTPAD_PORT      GPIOB
#define OW_FOOTPAD_PIN       11

/* =============================================================================
 * 4. LIAISON DE CONFIGURATION (UART vers interface web via USB-TTL / Bluetooth)
 * -----------------------------------------------------------------------------
 * Reutilise l'UART deja gere par le firmware EmanuelFeru (USART2 ou USART3 selon
 * ta variante). Debit conseille : 115200. Voir ow_comms.c.
 * ========================================================================== */
#define OW_COMMS_BAUD        115200

/* =============================================================================
 * 5. STRUCTURE DE PARAMETRES REGLABLES (lue/ecrite par la GUI, stockee en flash)
 * -----------------------------------------------------------------------------
 * ORDRE ET TYPES A NE PAS MODIFIER sans mettre a jour le schema de la GUI
 * (gui/index.html -> PARAM_SCHEMA) : la GUI envoie/recoit ce bloc tel quel.
 * Tous les gains/angles sont en ENTIERS a l'echelle indiquee pour eviter le
 * flottant sur la liaison.
 * ========================================================================== */
#define OW_CONFIG_VERSION    3
#define OW_CONFIG_MAGIC      0x4F573301   /* 'O''W' + version, garde-fou flash.  */

typedef struct __attribute__((packed)) {
    uint32_t magic;            /* = OW_CONFIG_MAGIC. Valide le contenu flash.    */

    /* --- Boucle d'equilibrage (PD sur angle + amortissement gyro) --- */
    int16_t  kp;               /* gain proportionnel angle   (x100)  ex 1500=15.0*/
    int16_t  ki;               /* gain integral              (x100)  ex 50=0.5   */
    int16_t  kd;               /* gain sur vitesse gyro      (x100)  ex 300=3.0  */
    int16_t  setpoint_trim;    /* offset d'assiette "a plat" (deg x10) ex 5=0.5  */

    /* --- Limites / securite --- */
    int16_t  start_angle_max;  /* engage seulement si |angle|<= (deg x10) ex 80  */
    int16_t  fault_angle_max;  /* coupe si |angle| >          (deg x10) ex 350   */
    int16_t  output_max;       /* commande moteur max         (0..1000)          */
    int16_t  output_ramp;      /* montee max de la commande / boucle (pts)       */
    int16_t  current_limit;    /* limite de courant demandee  (A x10) info FOC   */

    /* --- Comportement Onewheel (tiltback / pushback) --- */
    int16_t  tiltback_speed;   /* seuil vitesse pour tiltback (0..1000) 0=off    */
    int16_t  tiltback_angle;   /* releve de nez au tiltback   (deg x10) ex 60    */
    int16_t  lowbat_voltage;   /* tiltback batterie faible    (V x10) ex 360     */
    int16_t  cutoff_voltage;   /* coupure batterie critique   (V x10) ex 330     */

    /* --- Filtre IMU / orientation --- */
    int16_t  imu_alpha;        /* filtre complementaire       (x1000) ex 980     */
    int16_t  pitch_invert;     /* 0/1 : inverse le signe de l'angle mesure       */
    int16_t  output_invert;    /* 0/1 : inverse le sens de rattrapage moteur     */
    int16_t  footpad_active_low;/*0/1 : niveau logique du capteur optique        */

    /* --- Confort --- */
    int16_t  deadband;         /* zone morte autour de 0      (deg x10) ex 3     */
    int16_t  startup_beep;     /* 0/1 : bip a l'engagement (si buzzer cable)     */

    /* --- Progressivite (expo) : out_p = Kp*err*(1 + expo*|angle_deg|) --- */
    int16_t  angle_expo;       /* progressivite sur l'angle   (1/deg x1000) 0=lineaire */
    int16_t  start_ramp;       /* soft-start : montee 0->100% du couple (ms) ex 600 */
    int16_t  reserved[2];      /* marge d'evolution -> ne casse pas le schema.   */
    uint16_t crc;              /* CRC16 du bloc (hors ce champ) pour la flash.   */
} ow_config_t;

/* =============================================================================
 * 6. VALEURS PAR DEFAUT
 * -----------------------------------------------------------------------------
 * Reglage ROULANT valide sur la planche de reference (banc + essais reels).
 * Rechargees a chaque boot si la flash est vide/invalide. Un autre chassis
 * (poids, roue, batterie) demandera de re-regler via l'interface web puis
 * "Sauver en flash". PREMIER ESSAI TOUJOURS ROUES EN L'AIR.
 * ========================================================================== */
#define OW_CONFIG_DEFAULTS {                    \
    .magic             = OW_CONFIG_MAGIC,       \
    .kp                = 13300, /* 133.0 : gain proportionnel (pente au centre) */ \
    .ki                = 0,     /* 0.0  */      \
    .kd                = 40,    /* 0.4 : amortissement gyro */ \
    .setpoint_trim     = -13,   /* -1.3 deg : offset de montage MPU mesure */ \
    .start_angle_max   = 80,    /* 8.0 deg : fenetre d'engagement */ \
    .fault_angle_max   = 190,   /* 19.0 deg : coupure si depasse */  \
    .output_max        = 1000,  /* plein */    \
    .output_ramp       = 210,   /* rampe de conduite (frein reactif) */ \
    .current_limit     = 200,   /* 20.0 A (courant phase, pilote i_max/curDC_max) */ \
    .tiltback_speed    = 0,     /* off */      \
    .tiltback_angle    = 60,    /* 6.0 deg */   \
    .lowbat_voltage    = 345,   /* 34.5V (10S ~3.45V/cell : alerte) */ \
    .cutoff_voltage    = 315,   /* 31.5V (10S ~3.15V/cell : coupure) */\
    .imu_alpha         = 980,   /* 0.980 : filtre complementaire */ \
    .pitch_invert      = 0,                     \
    .output_invert     = 0,                     \
    .footpad_active_low= 1,     /* PB11 (pull-up) : ouvert=1=absent, ferme=0=present */ \
    .deadband          = 3,     /* 0.3 deg */   \
    .startup_beep      = 1,                     \
    .angle_expo        = 205,   /* 0.205 /deg : progressivite (doux au centre, mordant aux grands angles) */ \
    .start_ramp        = 950,   /* 950 ms : soft-start (montee douce du couple a l'engagement) */ \
    .reserved          = {0,0},                 \
    .crc               = 0,                     \
}

/* Config globale vivante (definie dans balance.c). */
extern ow_config_t ow_config;

#endif /* ONEWHEEL_CONFIG_H */
