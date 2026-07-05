# Connexions — Onewheel (carte hoverboard « split » GD32F103RCT6)

Description du câblage **réellement utilisé** sur la carte de conversion.
MCU : **GD32F103RCT6** (compatible STM32F103RC), boîtier **LQFP64**, 256 KB flash,
48 KB RAM, Cortex‑M3 72 MHz.

> ⚠️ Les deux moteurs d'origine ont été **recâblés pour tourner dans le même sens**
> (onewheel = une seule roue « virtuelle » entraînée par les 2 moteurs). Côté
> firmware : même consigne sur les deux, `steer = 0`.

---

## 1. Alimentation

| Rail | Source | Remarque |
|---|---|---|
| Batterie | **10S** (~42 V pleine, ~36 V nominal) | seuils firmware : alerte 34.5 V, coupure 31.5 V |
| Logique 3V3 | régulateur de la carte | alimente MCU + MPU6050 |
| MPU6050 VCC | **3V3** | **PAS 5 V** |

---

## 2. IMU — MPU6050 (I2C **logiciel** / bit‑bang)

Soudée sur le connecteur **UARTL1** (sideboard gauche débranchée). Bus I2C émulé
en bit‑bang sur deux GPIO libres (la carte ne charge pas ces broches).

| Signal | Broche MCU | Fil | Remarque |
|---|---|---|---|
| SDA | **PA2** | bleu | confirmé `WHO_AM_I = 0x68` (lu via bit‑bang piloté ST‑Link) |
| SCL | **PA3** | vert | horloge |
| VCC | 3V3 | | 3.3 V uniquement |
| GND | GND logique | | masse **froide/mal soudée = pas d'ACK** même si pull‑ups visibles |
| AD0 | GND | | → adresse **0x68** |

**Orientation de montage** : à plat, **puce vers le haut**, axe **X = sens de
marche**, axe **Z vers le haut**. → tangage (pitch) calculé sur `atan2(ax, az)` +
gyro axe **Y**, `GYRO_PITCH_SIGN = -1`.

**Notes critiques :**
- Le bruit **EMI de commutation des moteurs** corrompt l'I2C bit‑bang. Corrigé en
  **matériel** : fils MPU **torsadés** + **puissance éloignée** → 0 échec moteurs ON.
- Anciennes pastilles UARTR1 (**PA1 / PC2**) tenues à 0 par la carte → inutilisables.

---

## 3. Repose‑pied — contact mécanique (footpad)

Sécurité d'engagement : pas de pied → pas d'équilibrage.

**Modification** : le capteur **optique d'origine est abandonné** (il passait par le µC
de la sideboard, envoyé en série au GD32 — pas exploité ici). À la place, un **contact
mécanique fabriqué** (« support de contact footpad ») est câblé **directement** :

| Signal | Broche MCU | Niveau | Câblage |
|---|---|---|---|
| Footpad | **PB11** | **actif bas** | contact entre **PB11** et **GND** ; pull‑up interne |

- Ouvert = **1** = pas de pied ; fermé (pied présent) = **0** (`footpad_active_low = 1`).
- Point de piquage = **pastille data UARTR1** (PB11). ⚠️ Cette ligne comporte une
  **résistance CMS en série** sur la carte : souder **côté puce** (en amont de la
  résistance) sinon le signal n'arrive pas — c'était la cause d'un faux contact au début.
- Sideboard **débranchée** (le domaine série optique n'est plus utilisé).
- Un **OU logiciel** (`ow_footpad`, via interface web) permet d'armer l'engagement
  roues en l'air sans toucher le contact.

---

## 4. Programmation & réglage — SWD (ST‑Link V2)

| Signal | Broche MCU |
|---|---|
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| (3V3 réf.) | 3V3 |

- **Flash** : SWD à l'adresse `0x08000000`.
- **Réglage en direct** : la passerelle `tuner/bridge.py` lit/écrit la RAM et la
  config par SWD via OpenOCD, et sert l'interface web. **Aucun UART/FTDI requis.**
- Masse SWD = masse **logique** (même domaine que MPU) — distincte de la masse
  sideboard/UART sur cette carte.

---

## 5. Moteurs (mapping carte « split » standard)

Deux moteurs BLDC triphasés à capteurs Hall, pilotés en FOC.

### Moteur DROIT — TIM1

| Fonction | Broches |
|---|---|
| Phases (high side) | PA8 (CH1), PA9 (CH2), PA10 (CH3) |
| Phases (low side / complément) | PB13 (CH1N), PB14 (CH2N), PB15 (CH3N) |
| Capteurs Hall | PC10, PC11, PC12 |

### Moteur GAUCHE — TIM8

| Fonction | Broches |
|---|---|
| Phases (high side) | PC6 (CH1), PC7 (CH2), PC8 (CH3) |
| Phases (low side / complément) | PA7 (CH1N), PB0 (CH2N), PB1 (CH3N) |
| Capteurs Hall | PB5, PB6, PB7 |

> Les 3 fils de puissance de chaque moteur + les 3 fils Hall sont ceux d'origine de
> l'hoverboard (déjà validés avec le firmware d'origine). Seul le **sens** a été
> harmonisé par recâblage.

---

## 6. Mesures analogiques (ADC)

| Mesure | Broche |
|---|---|
| Courants de phase / shunts | PA0, PC0, PC1, PC3, PC4, PC5 |
| Courant DC‑link | PC2 |
| Tension batterie | PA1 |

Conversion courant : `A2BIT_CONV = 50` (1 A ↔ 50 LSB). La limite de courant est
**réglable en direct** depuis l'interface (`i_max` phase + `curDC_max` protection DC).

---

## 7. Divers

| Fonction | Broche |
|---|---|
| LED | PB2 |
| Buzzer | PA4 |

---

## 8. Récap. des broches ajoutées pour la conversion

| Broche | Usage ajouté | Était |
|---|---|---|
| **PA2** | MPU6050 **SDA** (I2C soft) | entrée commande optionnelle (libre) |
| **PA3** | MPU6050 **SCL** (I2C soft) | entrée commande optionnelle (libre) |
| **PB11** | Footpad **contact mécanique** (actif bas, vers GND) | data UARTR1 |

Tout le reste (moteurs, Hall, ADC, LED, buzzer, SWD) = câblage **d'origine** de la carte.

---

## 9. Points de vigilance (résumé)

- **MPU en 3V3** et **GND bien soudée** (masse froide = pas d'ACK).
- **Fils I2C torsadés + éloignés de la puissance** (sinon EMI moteur → lectures perdues).
- **Footpad PB11** (contact mécanique vers GND) : souder **côté puce** (bypass résistance CMS série).
- **Masses distinctes** : logique (MPU/SWD) ≠ sideboard/UART.
- **Premier essai TOUJOURS roues en l'air** — vérifier le sens de rattrapage avant de monter.
