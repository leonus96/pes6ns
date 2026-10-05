# PES6 Switch — Roadmap del port no oficial (Switch 1 con CFW)

> Objetivo: ejecutar **Pro Evolution Soccer 6 (versión PSP)** como homebrew nativo (`.nro`) en una **Nintendo Switch 1 con Atmosphère**, mediante **recompilación estática** (MIPS/Allegrex → C/C++ → ARM64), sin emulador.
>
> Este archivo es la guía de trabajo para sesiones de Claude Code. La sección "Instrucciones para Claude Code" está copiada en `CLAUDE.md`.

---

## 0. Contexto y decisiones tomadas

### Por qué la versión de PSP
- Hardware objetivo muy por debajo de la Switch (CPU MIPS 333 MHz, 32 MB RAM): el rendimiento no es el cuello de botella principal.
- Ya existe prueba de concepto: OptiJuegos recompiló PES6 PSP a WebAssembly (pes6.optijuegos.net) con 60 FPS, multijugador local y online. **Su código no está publicado** (a la fecha de este plan).
- La recompilación estática genera código nativo por adelantado (AOT) → no necesita JIT, que en Switch homebrew es problemático.

### Rutas posibles
| Ruta | Descripción | Estado |
|---|---|---|
| **A — Atajo** | Conseguir el código de la recomp de OptiJuegos y portar su runtime a Switch | Depende de que lo publique o lo comparta. Preguntarle (X: @OptiJogos) |
| **B — Principal** | Crear un perfil `pes6` propio sobre **PSPRecomp** (MIT) y portarlo a Switch | **Ruta elegida por defecto** |

Si la Ruta A se abre en algún momento, saltar directo a la Fase 5 adaptando su runtime.

### Framework base elegido
- **Base:** [`jessicanataliagta/PSPRecomp`](https://github.com/jessicanataliagta/PSPRecomp) — MIT, C++20, CMake. Tiene un juego funcionando de punta a punta (GTA: Vice City Stories, `profiles/vcs`) y separa framework genérico de perfiles por juego. Herramientas: `psp_analyze`, `psp_recomp`, `dump_function`. Guía de perfiles en `docs/PROFILE_GUIDE.md`.
- **Parches de Switch de referencia:** [`szczuru/PSPRecomp`](https://github.com/szczuru/PSPRecomp) rama `switch-port` — añade ramas `__SWITCH__` en display (libnx Framebuffer + PadState), audio (`audout` 48 kHz) y CMake (`NINTENDO_SWITCH`, `nx_create_nro`), más un workflow de GitHub Actions con la imagen `devkitpro/devkita64`. **No está compilado ni probado**: tomarlo como plantilla.
- **Herramientas auxiliares:** [`sp00nznet/psprecomp`](https://github.com/sp00nznet/psprecomp) — MIT, en C. Útil por su `allegrexrecomp` (pelar ISO/PBP/PRX, `info`, `funcs`, `cover`, `dis`) y por su documentación de bring-up (`docs/BRINGUP.md`, `docs/VFPU.md`). Su runtime aún no muestra juegos en pantalla.
- **Referencias, NO vincular:** PPSSPP (GPL) se usa solo como **oráculo** para comparar comportamiento. `uofw` (MIT) y `pspsdk` (BSD) como referencia de HLE/ABI. Evitar copiar código GPL (p. ej. de `sal063/PSP-recompilation-project`) salvo que se decida licenciar todo como GPL.

### Estado del repo (2026-10-04)
- Clonado de `jessicanataliagta/PSPRecomp` (`f6e7d41`). Remotos: `upstream` (base) y `szczuru` (rama `switch-port`, solo lectura/referencia). `origin` = fork propio [`leonus96/pes6ns`](https://github.com/leonus96/pes6ns) (público).
- El repo será **público** → `profiles/pes6/generated/` está en `.gitignore`.

---

## 1. Prerrequisitos (antes de la primera sesión)

### Hardware / consola
- [ ] Switch 1 con Atmosphère + Homebrew Menu (hbmenu) funcionando.
- [ ] Saber lanzar homebrew en **modo title takeover** (mantener R al abrir un juego), no desde el Álbum (modo applet = poca RAM).
- [ ] `nxlink` disponible para enviar `.nro` por red y recibir logs por stdout.
- [ ] (Opcional) `sys-clk` para pruebas de rendimiento con overclock.

### Juego (copia propia)
- [x] Dump propio de PES6 PSP (UMD o PSN). Anotar **serial, región y SHA-1** del ISO. *(**ULES-00476** — Pro Evolution Soccer 6, Europa, DISC_VERSION 1.03, PSP_SYSTEM_VER 2.81; ISO 1 248 329 728 bytes, SHA-1 `e5adf0b1a8386a5a33a358c39e7aa76d6a7c54b6`)*
- [x] Obtener el `EBOOT.BIN` **desencriptado** *(el del ISO está cifrado — cabecera `~PSP`; `BOOT.BIN` está vacío/ceros. Descifrado con PPSSPP 1.20.4 → `profiles/pes6/game/EBOOT_DECRYPTED.BIN`, ELF SHA-256 `361b85a6…5dd7`)* (ELF/PRX). PSPRecomp deja la desencriptación fuera del framework. Opciones: la opción de PPSSPP para volcar el EBOOT desencriptado al arrancar el juego, o `allegrexrecomp decrypt` de sp00nznet con material de claves propio.
- [ ] Extraer el resto de `PSP_GAME/USRDIR` (assets) a una carpeta local ignorada por git.

### Entorno de desarrollo
- [x] macOS (o Linux) con CMake ≥ 3.20, Ninja, clang con C++20. *(CMake 4.4.3, Ninja, Apple clang)*
- [x] devkitPro instalado + paquetes vía `dkp-pacman`: `switch-dev`, `switch-sdl2`, `switch-mesa`, `switch-ffmpeg` (si se necesita decodificar ATRAC3), `switch-tools`. *(macOS arm64: `switch-dev` + `switch-portlibs`; devkitA64 GCC 16.1 local vs 15.2 en la imagen de CI)*
- [x] PPSSPP de escritorio (oráculo / comparación). *(PPSSPPSDL 1.20.4 vía `brew install --cask ppsspp`)*
- [x] Alternativa sin toolchain local: CI con la imagen Docker `devkitpro/devkita64`. *(CI en verde; también se puede compilar local con `docker run devkitpro/devkita64`)*

---

## 2. Estructura del repo propuesta

```
pes6-switch/                     (fork de jessicanataliagta/PSPRecomp)
├── ROADMAP.md                   este archivo
├── CLAUDE.md                    instrucciones para Claude Code
├── include/ src/ tools/ tests/  framework (tocar lo mínimo; cambios genéricos → PR upstream)
├── switch/hello/                Fase 1: homebrew mínimo de prueba (framebuffer + pad + nxlink)
├── switch/selftest/             tests del framework corriendo en la consola
└── profiles/pes6/
    ├── CMakeLists.txt
    ├── README.md                cómo construir y qué archivos aporta el usuario
    ├── config/                  mapa de funciones, overrides de análisis
    ├── generated/               código AOT generado  ← NO publicar (contiene código del juego)
    ├── host/                    HLE propio, bootstrap, display/audio/input por plataforma
    │   ├── display_window.cpp   ramas: _WIN32 / __APPLE__+SDL / __SWITCH__ / headless
    │   ├── audio_output.cpp
    │   └── input.cpp
    ├── scripts/                 build/run/regenerate estables
    ├── tests/
    ├── game/                    ← ignorado: EBOOT desencriptado + assets del usuario
    ├── analysis/                ← ignorado: salidas temporales de análisis
    └── progress/                ← ignorado: bitácora de sesiones, handoffs
```

`.gitignore` debe cubrir: `profiles/pes6/game/`, `analysis/`, `progress/`, `generated/` (si el repo es público), `*.iso`, `*.cso`, `EBOOT.BIN`, `*.PRX`, claves.

---

## 3. Fases

Cada fase tiene **tareas**, **criterio de terminado (DoD)** y **estimación** para una persona a tiempo parcial con ayuda de Claude Code.

### Fase 1 — Toolchain y "hola mundo" en Switch  *(1–3 días)*
- [x] Instalar devkitPro y compilar un ejemplo de `switch-examples` (gráficos con SDL2 + un framebuffer de libnx). *(ejemplo propio `switch/hello` con framebuffer de libnx; probado en la consola)*
- [ ] Configurar `nxlink` y verificar logs (`printf` → terminal del Mac). *(el usuario transfiere con MTP Responder + OpenMTP; nxlink aún sin probar)*
- [x] Crear una plantilla CMake mínima con `-DCMAKE_TOOLCHAIN_FILE=$DEVKITPRO/cmake/Switch.cmake`, `nx_generate_nacp`, `nx_create_nro`. *(`switch/hello/`; compila sin warnings en la imagen `devkitpro/devkita64` (GCC 15.2) → `pes6_hello.nro` de 207 KB)*
- [x] Workflow de GitHub Actions que construya el `.nro` y lo suba como artifact. *(`.github/workflows/ci.yml`: job `switch-hello` + job `framework` en macOS/Linux; en verde desde `a359cb2`)*

**DoD:** un `.nro` propio pinta un patrón en pantalla, lee los Joy-Con y manda logs por nxlink; CI en verde.

### Fase 2 — Framework PSPRecomp en escritorio  *(2–5 días)*
- [x] Fork de PSPRecomp; compilar solo el framework en macOS: `cmake -S . -B out/framework -DPSPRECOMP_PROFILE=""` y correr `ctest`. *(compila y `ctest` pasa en macOS y Linux sin cambios. Extra: el framework completo también compila y enlaza para Switch con devkitA64 sin cambios)*
- [ ] Arreglar lo que dependa de MSVC/Windows en el framework (no en `vcs`), manteniendo los cambios genéricos y aislados (candidatos a PR upstream).
- [x] Leer `docs/PROFILE_GUIDE.md`, `docs/SOURCE_PROVENANCE.md` y recorrer `profiles/vcs/host` para entender: bootstrap, `Runtime::register_hle()`, `register_function()`, `register_native_fast_path()`, backend GE por software.
- [x] Documentar en `progress/` un mapa de cómo fluye un frame en el perfil VCS (display list GE → rasterizador CPU → presentación). *(`progress/arquitectura_vcs.md`. Hallazgos clave: el HLE (~226 funciones) vive en `profiles/vcs/host/vcs_profile.cpp`, no en el framework; fuera de Windows el perfil es headless; ATRAC3+/MPEG se decodifican buscando archivos sueltos — no servirá con los `.afs` de PES6)*

- [x] (Extra) `switch/selftest`: `.nro` que corre los tests del framework en la consola (`PSPRECOMP_TESTS_NO_SUBPROCESS` omite los 7 tests que lanzan `psp_recomp`). **PASS en la consola (113 ms)** tras corregir `Runtime::set_game_root` para rutas `sdmc:`.

**DoD:** framework compila y pasa tests en macOS; existe una nota que explica la arquitectura del perfil VCS.

### Fase 3 — Análisis de PES6  *(3–7 días)*
- [x] `psp_analyze` sobre el EBOOT desencriptado; guardar salida en `analysis/`. *(ELF estático `we10psp`, entry `0x089943DC`, 229 imports, 5 738 funciones)*
- [ ] Contrastar con `allegrexrecomp info/funcs/cover` (sp00nznet) para cobertura de funciones y decodificación.
- [x] Listar **todos los imports (NIDs)** que usa PES6 por módulo (`sceGe`, `sceGu`/display, `sceCtrl`, `sceAudio`, `sceSas`, `sceAtrac3plus`, `sceMpeg`/`scePsmf`, `sceIo`, `sceUtility` savedata/OSK, `sceKernel` threads/semáforos/eventflags, `sceNet`/adhoc…).
- [x] Tabla de brechas: NID → ¿implementado en PSPRecomp? → prioridad (bloqueante para arrancar / para menú / para partido / opcional). *(`progress/brechas.md`: 146/229 ya existen en el perfil VCS, faltan 83)*
- [x] Detectar si el juego carga PRX adicionales (módulos en `USRDIR`) que también haya que recompilar. *(solo módulos de Sony → HLE; nada que recompilar)*
- [ ] Identificar uso de VFPU (PES probablemente lo usa en física/animación) e instrucciones aún no soportadas. *(~4.9k instr. VFPU; las 34 000 "no soportadas" del analizador son sobre todo datos leídos como código; quedan unas decenas candidatas por clasificar)*

**DoD:** `profiles/pes6/config/` con el mapa de funciones inicial y un documento `progress/brechas.md` con la lista priorizada de HLE faltante.

### Fase 4 — Perfil `pes6` en escritorio  *(la fase grande: 1–3 meses)*
Trabajar primero en macOS/Linux, donde depurar es barato. Usar PPSSPP como oráculo en cada hito.

- [x] **4a. Esqueleto:** crear `profiles/pes6/` según la guía; `CMakeLists.txt`; generar el corpus AOT con `psp_recomp` (o una herramienta propia del perfil si hace falta lowering específico); bootstrap de rutas (`PSP_DATA`, `USRDIR`).
  - DoD: el binario compila, enlaza y ejecuta hasta el primer `sceDisplaySetFrameBuf` en modo headless.
  - *Estado:* perfil creado (opción B: HLE de VCS copiado y podado en `profiles/pes6/host/`), corpus AOT generado (117 unidades de 16 KiB, 73 226 entradas), `PES6Native` compila y enlaza en macOS y ejecuta código de PES6 hasta el primer import sin HLE (`sceKernelGetModuleIdByAddress`). Arreglados 2 bugs de `psp_recomp` (bucle infinito en JAL→import; JAL a destinos sin unidad).
  - **Hallazgo confirmado:** el ELF declara ~40 overlays como secciones vacías con dirección de carga (`title.ovl`/`bootset.ovl`… en `0x08D17800`, `game.ovl`/`select.ovl`… en `0x08D4E800`, `masterleague.ovl`, `edit.ovl`…). Varios comparten dirección. El código vive en `over.afs` y se carga con lecturas `disc0:/sce_lbn…`.
  - *Progreso del arranque (c8b31fd):* imprime su banner, inicializa sonido/hilos, recibe el aviso de UMD, lee las tablas de los `.afs`, carga el primer overlay en `0x08D17800` y unos 1,3 MB de datos; 62 vblanks; **se detiene en el primer código de overlay (`0x08D18E68`)**. Pantalla aún negra (el logo lo dibuja el overlay).
  - **Overlays (opción A, elegida por el usuario):** `scripts/extract_overlays.py` extrae los 36 overlays `MWo3` de `over.afs` y recompila AOT los 23 con código (incluido `game.ovl`, 3,4 MB); el host retira el código de una región al leer sobre ella y registra la traducción verificada (cabecera + hash) al saltar a ella. Framework: `--tag/--unit-base/--seeds` en `psp_recomp`, `unregister_code_range` y resolver de funciones faltantes en `Runtime`.
  - **Scratchpad** (16 KiB en `0x00010000`) añadido a `GuestMemory`.
  - *Estado (43f6d30):* `bootset.ovl` se ejecuta, 1800 vblanks sin fallos, **primeras imágenes**: panel de aviso con fundido; el texto sale mal decodificado. Después espera (¿botón? ¿memory stick?).
- [x] **4b. Primer frame:** completar HLE bloqueante (memoria, threads, IO, GE); presentar el framebuffer con SDL2 en escritorio.
  - DoD: se ven los logos de Konami / pantalla de título. *(logo de KONAMI con fundido, vblank ~700, licencias JFA y **pantalla de título en español** "PULSAR CUALQUIER BOTÓN" hacia vblank 1700; en el Mac corre ~10x más rápido que una PSP sin limitador; ventana SDL2 en `display_window_sdl.cpp`; diálogos del sistema y OSK con respuesta automática; vídeo de intro saltado con un override de la rutina de reproducción `0x088217C4`)*
- [x] **4c. Menús e input:** `sceCtrl` mapeado a teclado/mando SDL; navegación de menús.
  - DoD: se puede llegar a "Partido amistoso" y elegir equipos.
  - *Estado:* título → START → `select.ovl`/`select1.ovl`; primera vez: diálogos del sistema (Archivo de Opciones creado) y pantalla **"Nivel de juego"** con texto correcto. Tras elegir nivel venía una pantalla de carga (Reebok + PES6) que no terminaba: el hilo de streaming de música se despierta desde un manejador de **sub-interrupción VBLANK** (30, 0) que el HLE heredado de VCS no ejecutaba, y lee por **`umd0:`** (dispositivo de bloques, unidades de sector) que no estaba implementado. Corregido (+ `sceAtracSetDataAndGetID`, `Get/SetSecondBuffer`): **menú principal completo** hacia v3000 y, con ✕ en ruta, Partido → Exhibición → Configuración personal → **Seleccionar equipo** (Europa A). Después: Austria (local) vs Bélgica (visitante) → uniformes → ajustes generales → formación (`formation.ovl`) → **Comienzo del partido**: se carga `game.ovl`, presentación de alineaciones en el estadio 3D y **el partido arranca** (saque, reloj y marcador avanzan, la IA juega; v~10200). Arreglo necesario: semillas de overlays a partir de pares `lui`+`addiu/ori` del código (callbacks de `game.ovl` que registra el EBOOT). Navegación con teclado en la ventana SDL (`scripts/play.sh`) verificada por el usuario hasta jugar el partido.
- [x] **4d. Partido jugable:** arreglar VFPU, rasterizado (geometría transformada, texturas, blending, depth), timing de vblank.
  - DoD: un partido completo de principio a fin sin crashear; comparar capturas con PPSSPP.
  - *Estado:* `scripts/match_route.sh` juega en headless y de forma determinista un amistoso completo (Austria–Bélgica, 5 min, sin prórroga ni penaltis): saque, faltas, saques de puerta, goles con celebración y repetición, tiempo añadido, descanso ("Intervalo"), segunda parte, pitido final y pantalla **"Resultado"** con estadísticas (v~47200, ~3 min en el Mac). Arreglos: (1) `psp_recomp` usaba la unidad AOT de un bloque de 16 KiB para destinos fuera de la imagen que comparten bloque (`game.ovl` → `training_free.ovl` en 0x08D4C168) sin fijar `ctx.pc` → bucle infinito en la primera falta; (2) reloj de pared del HLE fijado (`PES6_FIXED_CLOCK`, por defecto en `run.sh`): con el del host la misma ruta divergía al empezar el partido; (3) GE: la Z interpolada se truncaba (2795 → 2794,9998 → 2794) y los quads 2D fallaban el test de profundidad en píxeles sueltos = el "ruido" de los paneles semitransparentes y de las banderas. Comparación con PPSSPP 1.20.4 (14 pantallas, `analysis/comparacion_ppsspp/`): menús idénticos. La diferencia de luz del partido no era el "Tiempo: Al azar" (la ruta ahora fija Día/Verano): el rasterizador por software calculaba la **niebla** por vértice pero nunca la aplicaba (arreglado en `ge_renderer.cpp`), y las capturas de ventana de PPSSPP venían con el perfil de color del monitor (hay que pasarlas a sRGB). Tras ambas cosas, saque y alineación coinciden con PPSSPP a ±2–3 niveles de media por canal.
- [x] **4e. Audio:** `sceAudio`/`sceSas` para efectos; música/comentarios (probablemente ATRAC3 → decodificar con ffmpeg como hace VCS).
  - DoD: efectos y música audibles y sin desincronización evidente.
  - *Estado:* **DoD cumplido** (prueba de escucha del usuario con `play.sh`: fluido en menús y partido). La música sale por ATRAC3plus (canal 5, 2048 muestras; 5 temas entre menú y partido) y los efectos, el público y el ambiente por SAS (canal 6, VAG/ADPCM). Lo hecho:
    - **Mezclador** `audio_output.cpp` sobre el tiempo virtual (portado del sink waveOut de VCS), con dispositivo SDL2 (`audio_device_sdl.cpp`) y captura WAV (`PSPRECOMP_AUDIO_WAV`) para verificar en headless.
    - **ATRAC3plus** con libavcodec (`atrac_decoder_ffmpeg.cpp`, FFmpeg opcional por pkg-config). Se decodifica trama a trama desde lo que el juego entrega con `AddStreamData` (PES6 guarda la música en `.afs`, no hay `.AT3` sueltos). Se respeta el desfase inicial del chunk `fact` y los bucles del `smpl`.
    - **Comparación con PPSSPP** (`DumpAudio`): el pico del efecto de confirmar es 25767 en PPSSPP y 25780 aquí; la RMS del partido, 8352 frente a 8745.
    - **Cortes en tiempo real (arreglado):** se presentaba la ventana en cada espera de vblank (dos por período en algunas fases), SDL/Metal bloqueaba el present y el limitador adelantaba el reloj virtual casi en cada fotograma, dejando huecos en el hilo de SAS. Ahora se presenta una vez por período y el limitador solo salta ante retrasos ≥100 ms.
    - **Pendiente (calidad):** el reverb de SAS es una aproximación (el bus wet incluye la señal directa) y satura un 0,33 % de muestras frente al 0,05 % de PPSSPP.
- [x] **4f. Guardado:** `sceUtility` savedata → archivos en disco; poder importar **option files** existentes (carpeta `PSP/SAVEDATA`).
  - DoD: guardar/cargar Liga Master y cargar un option file de la comunidad. *(Liga Master cumplido; la importación de option files cifrados queda aplazada por decisión del usuario.)*
  - *Estado:* los guardados se escriben **en claro** en `PSP/SAVEDATA/<juego><nombre>/` (o en `PES6_SAVEDATA_DIR`) con `DATA.BIN`, `ICON0.PNG` y un `PARAM.SFO` como el de la PSP (mismas claves y tamaños, 4912 bytes, `SAVEDATA_PARAMS` a cero = sin cifrar, como lo marca PPSSPP). La Liga Master usa LISTSAVE/LISTLOAD con una lista de 16+ nombres (`5008000`…); sin diálogo del sistema, la entrada se elige sola (carga: según el `focus` del juego, la más reciente; guardado: sobrescribe la más reciente, o el hueco que pide el `focus`) y se devuelve en `saveName`. Al cargar se rellenan los textos del `sfoParam` desde el `PARAM.SFO`. Comprobado: Liga Master nueva → Guardar → reinicio → Cargar datos → partido de liga completo → Guardar → reinicio → Cargar datos → hub en "Semana 7 / Partido 2, Pos. 8". Regresión: `scripts/ml_route.sh` (PASS, ~90 s).
  - Los guardados de una PSP o de PPSSPP van cifrados (PES6 pasa clave propia, `secureVersion`=3; `SAVEDATA_PARAMS`=0x41). No se descifran: la carga devuelve 0x80110306 (datos dañados; el juego sigue como en un primer arranque) y, antes de sobrescribirlos, se mueven a `<carpeta>.encrypted-backup`.
- [ ] **4g. Videos (si los hay):** `sceMpeg`/PSMF → ffmpeg, o saltarlos con un parche.

**Herramientas de depuración a montar desde el inicio:** trazas de entrada a funciones, watch de escrituras en memoria, chequeo de balance de stack, watchdog que vuelque estadísticas en cuelgues (ver `docs/BRINGUP.md` de sp00nznet para el enfoque).

### Fase 5 — Port a Switch  *(1–3 semanas)*
Seguir el patrón de tres ramas que ya usa el proyecto: `_WIN32` / `__SWITCH__` / genérico.
- [ ] Display: rama `__SWITCH__` con libnx `Framebuffer`/`NWindow` (blit del frame del rasterizador CPU, escalado y letterbox). Alternativa: SDL2 de devkitPro para compartir código con escritorio.
- [ ] Input: `PadState` → misma estructura de input del host. ZL/ZR libres para funciones extra.
- [ ] Audio: `audout` (48 kHz estéreo) con resampler y cola de buffers; respetar volumen por canal.
- [ ] Rutas: datos del juego en `sdmc:/switch/pes6/` (EBOOT desencriptado + `USRDIR`); guardados en `sdmc:/switch/pes6/SAVEDATA/`. No usar romfs con assets del juego (no se distribuyen).
- [ ] CMake: rama `NINTENDO_SWITCH` (link `nx`, `switch-ffmpeg` si aplica; `.nacp` con título/icono propios, no de Konami).
- [ ] Memoria: verificar al iniciar si se está en modo applet y avisar en pantalla que se use title takeover.
- [ ] CI: el workflow de la Fase 1 ahora construye `profiles/pes6` (sin datos del juego).

**DoD:** el `.nro` arranca en la consola, llega al menú y se puede jugar un partido (aunque vaya lento).

### Fase 6 — Rendimiento  *(2–6 semanas)*
- [ ] Medir FPS en portátil y dock con logs por nxlink (tiempo por frame: CPU guest vs rasterizado vs presentación).
- [ ] Compilar con `-O2/-O3`, LTO, `-mcpu=cortex-a57`; revisar hot paths del código generado.
- [ ] Paralelizar el rasterizador por software (4 núcleos A57) y/o vectorizar con NEON.
- [ ] Native fast paths (`register_native_fast_path`) para las funciones más calientes del juego.
- [ ] Si no alcanza: **backend GE por GPU** con OpenGL ES 3 (Mesa de devkitPro) o deko3d. Permite resolución interna 2x–4x (720p portátil / 1080p dock).
- [ ] Detectar modo dock/portátil (`appletGetOperationMode`) y ajustar resolución.

**DoD:** 30 FPS estables en partido como mínimo; meta de 60 FPS.

### Fase 7 — Pulido  *(1–3 semanas)*
- [ ] Multijugador local: 2 jugadores con Joy-Con separados (el juego de PSP es de un jugador local; requiere investigar cómo OptiJuegos lo logró — probablemente reutilizando el modo adhoc/2P interno).
- [ ] Parche de 60 FPS (si la lógica corre a 30).
- [ ] Menú de ajustes propio (overlay): mapeo de botones, escala/filtro, resolución interna.
- [ ] Icono y metadatos propios del `.nro`; README de usuario: "trae tu propia copia", cómo colocar archivos.

**DoD:** experiencia comparable al port web, en la Switch.

### Fase 8 (opcional) — Online
- [ ] Investigar el modo adhoc/WLAN del juego (`sceNetAdhoc*`) y mapearlo a sockets de libnx (servidor relay tipo "salas").
- [ ] Interop con el online del port web solo si OptiJuegos documenta su protocolo.

---

## 4. Riesgos y mitigaciones

| Riesgo | Impacto | Mitigación |
|---|---|---|
| HLE faltante (threads, savedata, ATRAC3, MPEG) | Bloquea arranque o features | Tabla de brechas de la Fase 3; implementar por prioridad; consultar `uofw` y pspsdk; comparar con PPSSPP como oráculo |
| VFPU con instrucciones no soportadas | Física/animación rota de forma sutil | Tests unitarios por instrucción; comparar resultados contra PPSSPP |
| Bugs "plausibles" del recompilador | Valores erróneos sin crash | Instrumentación desde el día 1 (trazas, watch de memoria, balance de stack) |
| Rasterizador por software lento en A57 | FPS bajos | Paralelizar/NEON; backend GPU en Fase 6 |
| Parches de Switch de szczuru sin probar | Errores de compilación/API de libnx | Esperarlos; corregir contra la documentación real de libnx |
| Licencias | Contaminar el proyecto con GPL | Mantener límite MIT; PPSSPP solo como oráculo |
| Legal (Konami) | Takedown | No publicar EBOOT, assets, claves ni el código AOT generado; el usuario aporta su copia y genera localmente |

---

## 5. Estimación total
- Con la Ruta A (código de OptiJuegos disponible): **2–6 semanas**.
- Con la Ruta B (perfil propio sobre PSPRecomp): **3–6 meses** a tiempo parcial, dominado por la Fase 4.

---

## 6. Instrucciones para Claude Code
*(copiado en `CLAUDE.md`)*

- Trabaja **por fases y en orden**. Antes de empezar, lee este ROADMAP y `profiles/pes6/progress/ULTIMA_SESION.md`.
- Al terminar cada sesión: marca las casillas completadas aquí y escribe en `progress/ULTIMA_SESION.md` qué se hizo, qué falló, hipótesis abiertas y el siguiente paso concreto.
- **Nunca** agregues a git: ISOs, EBOOT, PRX, assets del juego, claves, ni `generated/` si el repo es público. Verifica `.gitignore` antes de cada commit.
- Cambios en el framework (raíz) deben ser genéricos y sin direcciones del juego; lo específico de PES6 va en `profiles/pes6/`.
- No copies código de proyectos GPL (PPSSPP, JPCSP, `sal063/PSP-recompilation-project`). PPSSPP solo como referencia de comportamiento.
- Depura con datos, no con suposiciones: añade instrumentación antes de concluir la causa de un bug y registra las conclusiones descartadas en `progress/`.
- Primero escritorio (macOS/Linux), luego Switch. No avances a la Fase 5 sin cumplir el DoD de la 4d.
- Pide confirmación antes de refactors grandes del framework o de cambiar decisiones de la sección 0.

---

## 7. Referencias
- PSPRecomp (base): https://github.com/jessicanataliagta/PSPRecomp
- Parches Switch de referencia: https://github.com/szczuru/PSPRecomp (rama `switch-port`)
- psprecomp (herramientas y docs de bring-up): https://github.com/sp00nznet/psprecomp
- Firmware PSP en C (MIT): https://github.com/uofw/uofw
- pspsdk (ABI/headers): https://github.com/pspdev/pspsdk
- Tests de comportamiento PSP: https://github.com/hrydgard/pspautotests
- Port web de PES6 PSP (referencia de lo alcanzable): https://pes6.optijuegos.net/
- devkitPro / libnx: https://devkitpro.org — https://github.com/switchbrew/libnx
- Macros CMake de devkitPro para Switch (`nx_generate_nacp`, `nx_create_nro`): https://github.com/devkitPro/pacman-packages/tree/master/cmake/switch
