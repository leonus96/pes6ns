# PES6 Switch — instrucciones para Claude Code

Port no oficial de PES6 (PSP) a Switch 1 (Atmosphère) por recompilación estática sobre PSPRecomp. El plan completo y el estado de cada fase están en `ROADMAP.md`.

## Reglas de trabajo

- Trabaja **por fases y en orden**. Antes de empezar, lee `ROADMAP.md` y `profiles/pes6/progress/ULTIMA_SESION.md`.
- Al terminar cada sesión: marca las casillas completadas en `ROADMAP.md` y escribe en `profiles/pes6/progress/ULTIMA_SESION.md` qué se hizo, qué falló, hipótesis abiertas y el siguiente paso concreto.
- **Nunca** agregues a git: ISOs, EBOOT, PRX, assets del juego, claves, ni `profiles/pes6/generated/` (el repo es **público**). Verifica `.gitignore` y `git status` antes de cada commit.
- Cambios en el framework (raíz: `include/ src/ tools/ tests/`) deben ser genéricos y sin direcciones del juego; lo específico de PES6 va en `profiles/pes6/`. Los cambios genéricos son candidatos a PR upstream: mantenlos aislados.
- No copies código de proyectos GPL (PPSSPP, JPCSP, `sal063/PSP-recompilation-project`). PPSSPP solo como referencia de comportamiento.
- Depura con datos, no con suposiciones: añade instrumentación antes de concluir la causa de un bug y registra las conclusiones descartadas en `progress/`.
- Primero escritorio (macOS/Linux), luego Switch. No avances a la Fase 5 sin cumplir el DoD de la 4d.
- Pide confirmación antes de refactors grandes del framework o de cambiar decisiones de la sección 0 del ROADMAP.

## Remotos git

- `upstream` → `jessicanataliagta/PSPRecomp` (base).
- `szczuru` → `szczuru/PSPRecomp`, rama `switch-port`: parches de Switch **sin probar**, solo como plantilla (`git diff HEAD szczuru/switch-port -- <ruta>`).
- `origin` → fork propio (pendiente de crear).

## Comandos

```bash
# Framework en escritorio (macOS)
cmake -S . -B out/framework -G Ninja -DPSPRECOMP_PROFILE=""
cmake --build out/framework
ctest --test-dir out/framework --output-on-failure

# Homebrew de prueba de la Fase 1 (requiere devkitPro)
cmake -S switch/hello -B out/switch-hello -G Ninja -DCMAKE_TOOLCHAIN_FILE="$DEVKITPRO/cmake/Switch.cmake"
cmake --build out/switch-hello
nxlink -s out/switch-hello/pes6_hello.nro   # envía a la consola y queda escuchando stdout
```

## Notas del entorno

- El toolchain de Switch (`Switch.cmake`) define `NINTENDO_SWITCH`, `__SWITCH__` y enlaza `-lnx -lm` automáticamente. No hay `libdl` en Switch: no enlazar `${CMAKE_DL_LIBS}` bajo `NINTENDO_SWITCH`.
- El framework compila en macOS/clang sin cambios (solo warnings `-Wsign-conversion` en `include/psprecomp/common.hpp`).
