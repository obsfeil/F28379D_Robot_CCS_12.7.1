# Analyse av `finalproject.out` filen

## Filnavn
`C:/Users/obsfe/workspace_v10/FinalProject/binary/finalproject.out`

## Inngangspunkt
- **_c_int00** ved adresse **000824f5**

## Minnekonfigurasjon
Denne delen viser minnekonfigurasjonen for ulike minneområder som brukes av prosjektet:

- **RAMM0**: Startadresse 00000123, lengde 000002dd, alt ubrukte.
- **RAMLS05**: Startadresse 00008000, lengde 00003000, brukt 00000298.
- **FLASH**: Inkluderer områder som FLASHA, FLASHB osv., hvor data blir lagret og brukt.

## Seksjonsallokeringskart
Dette kartet viser hvordan forskjellige seksjoner av programmet ditt er fordelt i minnet. For eksempel:

- **codestart**: Startadresse 00080000, lengde 00000002
- **.cinit**: Startadresse 00082e18, lengde 00000155, inneholder seksjoner fra forskjellige .obj filer som UART_Chip_Main.obj, LEDPatterns.obj osv.
- **.stack**: Startadresse 0000b000, lengde 00000800, uinitialisert.

## Modulsammendrag
Dette gir en oversikt over modulene som brukes i prosjektet, inkludert kode, initialiserte og uinitialiserte data:

- **F28379dSerial.obj**: 1884 bytes kode, 0 bytes initialisert data, 12080 bytes uinitialisert data.
- **F2837xD_GlobalVariableDefs.obj**: 0 bytes kode, 0 bytes initialisert data, 8232 bytes uinitialisert data.

## Globale datasymboler
Sortert etter dataside og alfabetisk:

- **_AdcaResultRegs**: Adresse 00000b00, side 2c.
- **_CpuTimer0Regs**: Adresse 00000c00, side 30.

## Symboler sortert alfabetisk
Gir en alfabetisk oversikt over symbolene med deres adresse og side:

- **_abort**: Adresse 00082630, side 0.
- **_memcpy**: Adresse 00082749, side 0.

## Generelle observasjoner
- Prosjektet bruker en omfattende mengde forskjellige minneområder inkludert RAM og FLASH.
- **LEDPatterns.obj** inneholder mange .cinit-seksjoner som initierer forskjellige LED-mønstre.
- Modulsammendraget viser at **F28379dSerial.obj** og **F2837xD_GlobalVariableDefs.obj** er de største modulene med betydelig mengde uinitialiserte data.

## Konklusjon
Den genererte .map-filen gir en omfattende oversikt over minnebruken og seksjonsallokeringen i prosjektet ditt. Dette kan hjelpe til med å forstå hvordan minnet er organisert, identifisere eventuelle problemer med minnebruk, og optimalisere programmet ditt.

---

## Beskrivelse av UART_Chip_Main
Denne C-koden er designet for å kjøre på en Texas Instruments F28379D mikrokontroller. Programmet initierer flere periferi-enheter og håndterer tidsintervaller og UART-kommunikasjon. Nedenfor er en detaljert beskrivelse av hovedkomponentene i programmet.

### Inkluderte filer
Koden inkluderer flere bibliotek og header-filer som gir tilgang til forskjellige funksjoner og definisjoner:

- Standard C-bibliotek: `<stdio.h>`, `<stdlib.h>`, `<stdarg.h>`, `<string.h>`, `<math.h>`, `<limits.h>`
- Spesifikke filer for F28x-prosjektet og enheten: `F28x_Project.h`, `driverlib.h`, `device.h`
- Filer for spesifikke funksjoner: `f28379dSerial.h`, `LEDPatterns.h`, `song.h`

### Konstanter
Definerer noen matematiske konstanter:

- `PI`, `TWOPI`, `HALFPI`

### Avbruddsrutiner
Forhåndsdefinerer avbruddsrutiner:

- `__interrupt void cpu_timer0_isr(void);`
- `__interrupt void cpu_timer1_isr(void);`
- `__interrupt void cpu_timer2_isr(void);`
- `__interrupt void SWI_isr(void);`

### Funksjonsprototyper
Deklarerer funksjoner for håndtering av UART-mottak:

- `void serialRXA(serial_t *s, char data);`
- `void serialRXC(serial_t *s, char data);`

### Globale variabler
Deklarerer flere globale tellevariabler:

- `uint32_t numTimer0calls, numSWIcalls, numRXA`
- `uint1616_t UARTPrint, LEDdisplaynum`

### Main-funksjonen
Hovedfunksjonen inneholder følgende trinn:

#### Systeminitialisering:
- Initialiserer systemkontrollen: `InitSysCtrl()`
- Initialiserer GPIO (generelle inn-/utganger): `InitGpio()`

#### LED-konfigurasjon:
- Setter opp flere GPIO-pinner for LED-er ved hjelp av `GPIO_SetupPinMux()` og `GPIO_SetupPinOptions()`.

#### Push-knapp konfigurasjon:
- Setter opp GPIO-pinner for push-knapper.

#### Avbruddshåndtering:
- Deaktiverer CPU-avbrudd: `DINT`
- Initialiserer PIE-kontroll (Peripheral Interrupt Expansion) og vektor-tabellen.
- Mapper avbrudd til ISR-funksjoner.

#### Timerkonfigurasjon:
- Initialiserer CPU-timere og setter avbruddsperioder.
- Aktiverer CPU-timer avbrudd.

#### UART-initialisering:
- Initialiserer seriell kommunikasjon: `init_serial(&SerialA,115200,serialRXA)`, `init_serial(&SerialC,9600,serialRXC)`.

#### Global avbrudd aktivering:
- Aktiverer globale avbrudd: `EINT`, `ERTM`

#### Hovedløkken:
- En uendelig løkke som sjekker `UARTPrint` og sender data over UART.

### Avbruddsrutiner (ISR)
- **SWI_isr**:
  - Øker `numSWIcalls` hver gang det kalles.
- **cpu_timer0_isr**:
  - Øker `numTimer0calls` og toggler en rød LED.
- **cpu_timer1_isr**:
  - Øker `CpuTimer1.InterruptCount`.
- **cpu_timer2_isr**:
  - Toggler en blå LED og øker `CpuTimer2.InterruptCount`. Setter `UARTPrint` til 1 for hver 50. gang.

### UART mottakshåndtering
- **serialRXA**:
  - Øker `numRXA` hver gang en karakter mottas.
- **serialRXC**:
  - Lagrer de siste 10 mottatte tegnene og utfører handlinger basert på spesifikke tegnsekvenser.

## Oppsummering
Denne koden setter opp og administrerer grunnleggende funksjonalitet for en F28379D mikrokontroller, inkludert GPIO, timere og UART-kommunikasjon. Koden bruker avbruddsrutiner for effektiv håndtering av tidsstyrte og hendelsesbaserte prosesser.

---

## Beskrivelse av UART_Segbot
Denne C-koden er utviklet for en Texas Instruments F28379D mikrokontroller og er designet for å kontrollere en selvbalanserende robot ved hjelp av et kontrollpanel. Programmet initialiserer flere periferi-enheter, håndterer tidsintervaller, ADC og SPI-kommunikasjon, samt UART-kommunikasjon. Nedenfor følger en detaljert beskrivelse av de viktigste komponentene i programmet.

### Inkluderte filer
Koden inkluderer flere bibliotek og header-filer som gir tilgang til ulike funksjoner og definisjoner:

- Standard C-bibliotek: `<stdio.h>`, `<stdlib.h>`, `<stdarg.h>`, `<string.h>`, `<math.h>`, `<limits.h>`
- Spesifikke filer for F28x-prosjektet og enheten: `F28x_Project.h`, `driverlib.h`, `device.h`
- Filer for spesifikke funksjoner: `f28379dSerial.h`, `LEDPatterns.h`, `song.h`

### Konstanter
Definerer noen matematiske konstanter:

- `PI`, `TWOPI`, `HALFPI`

### Avbruddsrutiner
Forhåndsdefinerer avbruddsrutiner:

- `__interrupt void cpu_timer0_isr(void);`
- `__interrupt void cpu_timer1_isr(void);`
- `__interrupt void cpu_timer2_isr(void);`
- `__interrupt void SWI_isr(void);`
- `__interrupt void SPIB_isr(void);`
- `__interrupt void ADCA_ISR(void);`

### Funksjonsprototyper
Deklarerer funksjoner for oppsett og håndtering av forskjellige enheter:

- `void setupSpib(void);`
- `void serialRXA(serial_t *s, char data);`
- `void serialRXC(serial_t *s, char data);`
- `void init_eQEPs(void);`
- `float readEncLeft(void);`
- `float readEncRight(void);`
- `void setEPWM2A(float);`
- `void setEPWM2B(float);`

### Globale variabler
Deklarerer flere globale variabler som brukes i kontroll og beregninger:

- Tellere: `uint32_t numTimer0calls, numSWIcalls, numRXA`
- Kontrollvariabler: `uint16_t UARTPrint, LEDdisplaynum`
- Sensorvariabler: `int16_t accelxraw, accelyraw, accelzraw, gyroxraw, gyroyraw, gyrozraw`
- Fysiske målinger: `float accelx, accely, accelz, gyrox, gyroy, gyroz`
- Kontrollparametre: `float K1, K2, K3, Kp, Ki, Kd`

### Main-funksjonen
Hovedfunksjonen inneholder følgende trinn:

#### Systeminitialisering:
- Initialiserer systemkontrollen: `InitSysCtrl()`
- Initialiserer GPIO (generelle inn-/utganger): `InitGpio()`

#### LED-konfigurasjon:
- Setter opp flere GPIO-pinner for LED-er ved hjelp av `GPIO_SetupPinMux()` og `GPIO_SetupPinOptions()`.

#### Push-knapp konfigurasjon:
- Setter opp GPIO-pinner for push-knapper.

#### Avbruddshåndtering:
- Deaktiverer CPU-avbrudd: `DINT`
- Initialiserer PIE-kontroll (Peripheral Interrupt Expansion) og vektor-tabellen.
- Mapper avbrudd til ISR-funksjoner.

#### Timerkonfigurasjon:
- Initialiserer CPU-timere og setter avbruddsperioder.
- Aktiverer CPU-timer avbrudd.

#### UART-initialisering:
- Initialiserer seriell kommunikasjon: `init_serial(&SerialA,115200,serialRXA)`, `init_serial(&SerialC,9600,serialRXC)`.

#### PWM-konfigurasjon:
- Setter opp PWM for motorstyring.

#### ADC-konfigurasjon:
- Konfigurerer ADC for å lese sensordata.

#### eQEP-konfigurasjon:
- Initialiserer eQEP for å lese enkodere på hjulene: `init_eQEPs()`

#### Global avbrudd aktivering:
- Aktiverer globale avbrudd: `EINT`, `ERTM`

#### Hovedløkken:
- En uendelig løkke som sjekker `UARTPrint` og sender data over UART.

### Avbruddsrutiner (ISR)
- **SWI_isr**:
  - Utfører balansekontrollalgoritmen og sender kontrollsignalene til motorene.
- **cpu_timer0_isr**:
  - Øker `numTimer0calls` og håndterer eventuelle periodiske oppgaver.
- **cpu_timer1_isr**:
  - Øker `CpuTimer1.InterruptCount`.
- **cpu_timer2_isr**:
  - Øker `CpuTimer2.InterruptCount`.
- **ADCA_ISR**:
  - Leser ADC-verdier og triggere SPI-overføring for å lese gyroskop- og akselerometerdata.
- **SPIB_isr**:
  - Henter sensordata fra SPI og utfører kalibrering og filtrering.

### UART mottakshåndtering
- **serialRXA**:
  - Øker `numRXA` hver gang en karakter mottas.
- **serialRXC**:
  - Håndterer innkommende data fra Bluetooth og justerer kontrollparametrene basert på kommandoer.

### Ekstra funksjoner
- **setupSpib**:
  - Setter opp SPIB for kommunikasjon med IMU (Inertial Measurement Unit).
- **init_eQEPs**:
  - Initialiserer eQEP-moduler for å lese posisjon fra hjulenkodere.
- **setEPWM2A** og **setEPWM2B**:
  - Setter PWM-signaler basert på beregnede kontrollverdier.

## Oppsummering
Denne koden setter opp og administrerer grunnleggende funksjonalitet for en F28379D mikrokontroller, inkludert GPIO, timere, ADC, SPI og UART-kommunikasjon. Koden bruker avbruddsrutiner for effektiv håndtering av tidsstyrte og hendelsesbaserte prosesser, samt implementerer en balansekontrollalgoritme for en selvbalanserende robot.
