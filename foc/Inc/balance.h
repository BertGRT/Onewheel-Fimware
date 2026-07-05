/*
 * balance.h -- Controleur d'equilibrage Onewheel + machine a etats de securite.
 */
#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>

typedef enum {
    OW_STATE_IDLE = 0,   /* moteurs coupes, en attente du rider                 */
    OW_STATE_ARMED,      /* pied detecte, on attend que la planche soit a plat  */
    OW_STATE_RIDING,     /* equilibrage actif                                   */
    OW_STATE_TILTBACK,   /* releve de nez (survitesse / batterie faible)        */
    OW_STATE_FAULT       /* coupure : angle excessif, pied leve, batterie crit. */
} ow_state_t;

/* Telemetrie exposee a l'interface web (voir ow_comms.c). */
typedef struct {
    float   pitch;        /* assiette filtree (deg)          */
    float   pitch_rate;   /* vitesse d'assiette (deg/s)      */
    int16_t output;       /* commande moteur envoyee (-1000..1000) */
    int16_t state;        /* ow_state_t                      */
    int16_t voltage;      /* tension batterie (V x10)        */
    int16_t current;      /* courant moteur (A x10)          */
    int16_t footpad;      /* 0/1 capteur optique             */
    int16_t rpm;          /* vitesse roue estimee            */
} ow_telemetry_t;

/* Appele une fois au demarrage (apres init MPU / config). */
void balance_init(void);

/* Appele a frequence fixe OW_LOOP_HZ. Retourne la commande moteur [-1000..1000].
 * Cette valeur doit etre injectee comme "speed" (steer=0) dans le firmware FOC. */
int16_t balance_update(void);

/* Etat / telemetrie courants (pour la GUI). */
ow_state_t     balance_state(void);
const ow_telemetry_t *balance_telemetry(void);

/* Le firmware principal fournit ces mesures (adapter dans les hooks main.c). */
void balance_set_feedback(int16_t voltage_x10, int16_t current_x10, int16_t rpm);

/* Persistance de ow_config en flash (page dediee 0x0803F800).
 *  - ow_config_load() : appele au boot, charge la flash si magic+CRC valides.
 *  - ow_config_save() : ecrit la config courante en flash. Retour 1=ok, 2=echec.
 * ATTENTION : l'ecriture flash gele le CPU ~40ms -> uniquement moteurs coupes. */
void    ow_config_load(void);
uint8_t ow_config_save(void);
extern volatile uint8_t ow_save_req;   /* 1 = demande de sauvegarde (ecrit par la GUI) */
extern volatile uint8_t ow_save_ack;   /* 0=idle, 1=ok, 2=echec flash, 3=refuse (roule) */

#endif /* BALANCE_H */
