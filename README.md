# Onewheel firmware — STM32F103 (base FOC EmanuelFeru) + interface web de réglage

Conversion d'un **hoverboard** (carte « split » GD32F103 / STM32F103RC) en
**onewheel auto‑équilibré**. Les deux moteurs sont recâblés pour tourner dans le
même sens, une **MPU6050** sert d'IMU, et l'**inter optique** d'origine sert de
sécurité repose‑pied. Tout se règle **en direct depuis un navigateur**, via le
**ST‑Link seul** (aucun UART/FTDI nécessaire), et les réglages se **sauvent en flash**.

Le firmware s'appuie sur le
[hoverboard‑firmware‑hack‑FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC)
d'Emanuel Feru (couche FOC / HAL), avec une variante `VARIANT_ONEWHEEL` ajoutée.

---

## Arborescence

```
foc/                     firmware (base FOC EmanuelFeru + onewheel)
├─ Inc/
│  ├─ onewheel_config.h  struct de réglages + défauts + brochage bas niveau
│  ├─ balance.h          régulateur d'équilibrage + persistance flash
│  ├─ mpu6050.h          IMU (I2C logiciel)
│  └─ config.h           config FOC (mode VLT, courant, variante ONEWHEEL)
├─ Src/
│  ├─ balance.c          boucle PD + expo + soft‑start + machine à états + flash
│  ├─ mpu6050.c          I2C bit‑bang durci + filtre complémentaire
│  └─ main.c             intégration onewheel dans la boucle FOC
├─ Drivers/              HAL STM32F1 + CMSIS (upstream)
├─ STM32F103RCTx_FLASH.ld, startup_stm32f103xe.s
tuner/                   interface web de réglage (ST‑Link)
├─ bridge.py             passerelle OpenOCD (SWD) ↔ HTTP + sert la page
└─ index.html            interface (sliders, télémétrie, sauvegarde flash)
reglage.bat              lance l'interface (python tuner/bridge.py)
```

---

## Fonctionnement

1. **IMU** — MPU6050 en **I2C logiciel** sur `PA2` (SDA) / `PA3` (SCL). Assiette par
   filtre complémentaire (accel + gyro), amortissement basé directement sur la
   vitesse gyro. Lecture durcie (majority‑vote, retries) contre le bruit moteur.
2. **Équilibrage** (`balance.c`) :
   `sortie = Kp·err·(1 + expo·|angle|) + Ki·∫err − Kd·gyro`, saturée à `output_max`.
   - **expo** : réponse progressive — douce au centre (stable), mordante aux grands
     angles (couple/frein) sans rendre le centre nerveux.
   - **soft‑start** : à l'engagement, le couple monte de 0 → 100 % sur `start_ramp`
     ms → pas d'à‑coup.
   - **rampe de conduite** (`output_ramp`) : découplée du soft‑start, réglée vive
     pour un frein réactif à l'inversion de commande.
3. **Machine à états** : `IDLE → ARMÉ → RIDING/TILTBACK → FAULT`. Sécurités :
   pied absent, `|angle| > fault_angle_max`, tension < coupure.
4. **Sortie** → les 2 moteurs (même consigne, cohérent avec le recâblage) via la
   couche FOC en **mode tension (VLT)** — plus robuste que le mode couple pour tenir
   le couple à l'arrêt sur moteurs à capteurs Hall.
5. **Limite de courant réglable en live** : le slider pilote `i_max` (courant phase)
   et `curDC_max` (protection DC) sans recompiler.
6. **Persistance flash** : la config vit dans la dernière page flash (`0x0803F800`,
   hors zone code et hors EEPROM émulée). Chargée au boot, écrite via le bouton
   « Sauver en flash ».

---

## Brochage

| Fonction | Broche STM32 | Remarque |
|---|---|---|
| MPU6050 SDA | `PA2` | I2C logiciel (bit‑bang), WHO_AM_I=0x68 confirmé |
| MPU6050 SCL | `PA3` | fils torsadés, éloignés de la puissance (EMI) |
| MPU6050 VCC | 3V3 | pas 5 V |
| Repose‑pied (optique) | `PB11` | actif bas (pull‑up interne), pastille data UARTR1 |
| Programmation / réglage | SWD | **ST‑Link V2** (`SWDIO`/`SWCLK`/`GND`) |

Orientation MPU : à plat, puce vers le haut, axe **X = sens de marche**.

> Câblage complet (moteurs, Hall, ADC, alimentation, masses) : **[CONNEXIONS.md](CONNEXIONS.md)**.

---

## Compilation

Outils (non inclus dans le dépôt) : **arm‑none‑eabi‑gcc** et **OpenOCD**.
Adapter les chemins. Build en une commande (pas de `make` requis) :

```bash
cd foc
GCC=arm-none-eabi-gcc
HAL=Drivers/STM32F1xx_HAL_Driver/Src
$GCC -mcpu=cortex-m3 -mthumb -Og -Wall -fdata-sections -ffunction-sections \
  -std=gnu11 -g -DUSE_HAL_DRIVER -DSTM32F103xE -DVARIANT_ONEWHEEL \
  -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include \
  $HAL/stm32f1xx_hal_flash.c $HAL/stm32f1xx_hal_pwr.c $HAL/stm32f1xx_hal_rcc.c \
  $HAL/stm32f1xx_hal_tim.c $HAL/stm32f1xx_hal_tim_ex.c $HAL/stm32f1xx_hal_gpio_ex.c \
  $HAL/stm32f1xx_hal_adc_ex.c $HAL/stm32f1xx_hal_cortex.c $HAL/stm32f1xx_hal_flash_ex.c \
  $HAL/stm32f1xx_hal_gpio.c $HAL/stm32f1xx_hal_rcc_ex.c $HAL/stm32f1xx_hal.c \
  $HAL/stm32f1xx_hal_adc.c $HAL/stm32f1xx_hal_uart.c $HAL/stm32f1xx_hal_i2c.c \
  $HAL/stm32f1xx_hal_dma.c \
  Src/system_stm32f1xx.c Src/setup.c Src/control.c Src/comms.c Src/util.c Src/main.c \
  Src/bldc.c Src/eeprom.c Src/hd44780.c Src/pcf8574.c Src/stm32f1xx_it.c \
  Src/BLDC_controller_data.c Src/BLDC_controller.c Src/mpu6050.c Src/balance.c \
  startup_stm32f103xe.s \
  -specs=nano.specs -TSTM32F103RCTx_FLASH.ld -lc -lm -lnosys -Wl,--gc-sections \
  -o build/hover_ow.elf
arm-none-eabi-objcopy -O binary build/hover_ow.elf build/hover_ow.bin
```

## Flash (ST‑Link)

```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program foc/build/hover_ow.bin verify reset exit 0x08000000"
```

> Le `program` n'écrit que la zone code : la page de config (`0x0803F800`) survit
> aux reflashs. Une flash vide/invalide recharge les défauts de `onewheel_config.h`.

## Réglage (interface web)

```bash
python tuner/bridge.py        # ou reglage.bat
```

Ouvrir `http://127.0.0.1:8666/`. Le bridge lance OpenOCD, lit la télémétrie et écrit
les réglages **en direct par SWD**. Réglages : `Kp/Ki/Kd`, expo, trim, zone morte,
commande max, rampe sortie, démarrage progressif, limite courant, angle de coupure ;
boutons Armer / Footpad / KILL et **Sauver en flash**.

---

## Mise en route (engin qui te porte — à respecter)

1. **Premier essai TOUJOURS roues en l'air**, planche sur support stable.
2. Engager le footpad → `ARMÉ`, ramener à plat → `RIDING`.
3. **Vérifier le SENS** : nez vers le bas → les roues doivent *rattraper* (avancer
   sous la pente). Sinon cocher `output_invert`. C'est LE réglage critique.
4. Régler `Kp` (fermeté), puis `expo` (mordant aux grands angles), `Kd` si ça vibre.
   `start_ramp` pour la douceur d'engagement, `output_ramp` pour le frein.
5. Monter la limite de courant par paliers en **surveillant la chauffe des MOSFET**.
6. « Sauver en flash » quand c'est bon.
7. Essais au sol seulement ensuite, **avec protections**.

## Avertissement

Firmware d'auto‑équilibrage **critique en sécurité**. Une erreur de signe, de câblage
ou de gain peut provoquer une accélération incontrôlée. Tests roues en l'air d'abord,
protections ensuite, **à tes risques**. Aucune garantie.

Base FOC © Emanuel Feru (GPL‑3.0) — voir `foc/LICENSE`.
