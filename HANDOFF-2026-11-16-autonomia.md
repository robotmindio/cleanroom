## ffc calificada / deploy de candidata — 17 noviembre 2026, 13:40 MYT / 05:40 UTC

- **ffc9566 calificada y sellada**: 110/110 compute,485.85s;16/16Pi,128.92s,
  ambos sellos verificados independientemente exit0. Bootstrap21 y ROS50 pasan;
  recovery24/owner6 y entrenamiento9 sin skips. Unit compute inactive/result0.
- Runtime0aa se restauró, arm RPC success/readiness exit0. Probe inicial
  `restored-before-start-footprint-ffc9566-20261117.json` tomó fase DISARMED durante
  arranque; después se armó normalmente. Fresh stow.012322, tres cámaras frescas.
- **Deploy canónico ffc EN CURSO** desde la rama exacta, limpia y subida, antes del
  merge para probar físicamente cuanto antes. Se avisó al usuario de este cambio
  de orden: el merge espera prueba física + cierre CI. El wrapper privado ahora
  fija origin/main21dd y upstream de la candidata==ffc en vez de exigir tree equal
  con main; el deploy-split tracked conserva todos sus preflights y los dos sellos.
  PID/session toolkit94825; log
  `bootstrap/deploy-start-footprint-ffc9566-20261117.log`. No repetir mientras siga.
- PR193 draft/headffc. Hosted build/test/device/static pasan, CodeQL verdes,
  cobertura en curso (run44024126766). Main/local/origin siguen21dd444.
  No usar expected-head0aa ni asumir que193 ya se mergeó.
- Tras deploy: preflight8s y recorder nuevo prefijo
  `qualification-explore-start-footprint-ffc9566-20261117-1`, duración300;
  admitir UNA VEZ con helper `run-bounded-explore-start-footprint-ffc9566-20261117.py`
  (180activos/radio2/wall240), reconciliar hasta stop confirmado + recursos libres.
  Buscar traslación/navegación útiles, no sólo giro. Primer ensayo0aa/gen57 ya
  terminal/cancelado y recorder terminado; pruebas+assessment conservadas.
- Después hace falta el ensayo **con DB nueva separada**, helpers ffc preparados
  descritos abajo (240activos/radio2/wall300, recorder360 + checkpoint de freshDB).
  Todavía NO se inició esa unidad ni se admitió el ensayo fresco.
- Si física+CI pasan: ready/review y merge193 con expected headffc; fetch main y
  comprobar árbol igual a ffc ya desplegada. No hace falta otro restart del mismo
  árbol. Finalizar con runtime primario ARMED, cámaras/sensores sanos y recursos
  libres; actualizar HANDOFF/WIP/ROLLOUT con evidencia física real.

---

## Ensayo físico y segundo bloqueo — 17 noviembre 2026, 13:30 MYT / 05:30 UTC

**Seguir hasta demostrar navegación útil y arranque con mapa nuevo. No darlo por cerrado.**

- PR **#192 mergeado**, expected head0aa; main/originmain/localmain limpio:
  **21dd4444c996aa74f5fe31bfcc397901ff617823**, árbol igual a 0aa.
  Cinco checks verdes, build-test-lint24m25s. Deploy canónico 0aa exit0, tres stow
  frescos, readiness dos veces, ambos marcadores/current0aa, previous e746,
  rearmado y tres cámaras frescas. Log `deploy-bootstrap-0aa796b-20261117.log`.
- **Ensayo generación57**, ID `qualification-explore-bootstrap-0aa796b-20261117-1`,
  radio2m/120s. Arrancó `observing_without_frontier`, giró y generó frontiers,
  pero después repitió `START_OCCUPIED` del planner: el viejo pause/resume no
  podía corregir la huella global inicial. Se canceló explícitamente:
  **canceled, complete=false, stop confirmed, recursos libres**, visited0/unreachable0.
  No reusar ese ID/gen; runner y recorder ya terminaron. Evidencia completa prefijo
  del ID, cancel-start-footprint-gen57*, y `*-assessment.json`.
  Entre admisión/terminal: 3134 muestras odom, giro **1.59950rad (~91.6°)**;
  recorrido integrado .05987m (deriva durante giro; no llamarlo avance útil),
  área conocida raw **3.3125→máximo5.8050m²**, último5.7900. Resultado regional .970m².
  El bootstrap ya observa físicamente, **falta desbloquear navegación**.
- **PR #193 draft**, head **ffc9566d6834ec8100475de1b81332183d13376d**, rama limpia/subida
  `fix/exploration-start-footprint-bootstrap`, worktree locked
  `/home/nex/roboto/cleanroom-worktrees/exploration-start-footprint-bootstrap-20261117`.
  `StartingFootprintBlocked(Paused)` separa START_OCCUPIED y `_explore_step` lo
  recupera con `_observe_step`. El contador sólo se reinicia cuando un route
  global pasa; antes se borraba cada vez que aparecía un candidato y nunca
  alcanzaba el paso local. No cambia Nav2/collision/stow/freshness/unknown policy.
  Test nuevo reprodujo rojo el bucle (timeout20s); ambos casos relevantes verdes
  **2/2 en16.15s**, Ruff pasa. 4archivos, 43add/8del.
- Staging canónico ffc **activo, aún no calificado/sellado**:
  `lekiwi-stage-start-footprint-ffc9566.service` (CTEST_PARALLEL_LEVEL2),
  log `~/.ros/lekiwi/stage-start-footprint-ffc9566.log`;
  Pi `lekiwi-stage-device-start-footprint-ffc9566.service`, log stage-device-*.
  Fuentes detach/locked; entry Pi hashc6d8665c... exacto. CI run **44024126766**.
  Esperados110suites compute (bootstrap21 casos ahora)/16Pi.
- **Runtime compute parado de forma planificada otra vez**, robot apoyado torque-off:
  dos check-travel-stow max-age.5 pasaron alrededor de05:27UTC; disarm success=True
  confirmó los nueve sin torque. Selección compute/Pi sigue0aa; cinco servicios Pi
  activos. Restaurar current/rearm después de calificar, antes de deploy canónico.
- Nuevo backup tras el ensayo y cerrar writer:
  `map-after-bootstrap-gen57-20261117/rtabmap.db`, integrityok,18,067,456bytes,
  SHA **fe4e939dd3be3529c7236415daa22a889fde05bec776272c9238017d525cb157**.
- Helpers ffc preparados, **NO ejecutados** en bootstrap:
  `deploy-start-footprint-ffc9566-20261117.sh`;
  `run-bounded-explore-start-footprint-ffc9566-20261117.py`, ID
  `qualification-explore-start-footprint-ffc9566-20261117-1`,180s/radio2/wall240;
  `run-bounded-explore-fresh-map-ffc9566-20261117.py`, ID
  `qualification-explore-fresh-map-ffc9566-20261117-1`,240s/radio2/wall300.
  Recorder **ya corregido** `record-bootstrap-motion-and-growth-20261117.py`
  incluye QoSProfile/DurabilityPolicy, --duration300/360 respectivamente.
  Su primer arranque 0aa falló por import antes de admitir; streams vacíos
  preservados como `.init-failed`. Se corrigió y ensayo57 sí grabó completo.
  Fresh helpers `start-fresh-map-bootstrap-ffc9566-20261117.sh` y
  `restore-primary-map-after-bootstrap-ffc9566-20261117.sh` usan DB separada
  `bootstrap/fresh-map-bootstrap-ffc9566-20261117/rtabmap.db` y unidad
  `lekiwi-qualification-fresh-map-ffc9566`, con stow/disarm gates y SHA primario.

Próximo: terminar sellos/CI ffc; restaurar0aa ARMED; review/merge expectedffc;
deploy canónico con stow fresco. Repetir ensayo actual y comprobar avance real,
no sólo giro; luego DB fresca separada, reconciliar y restaurar primario ARMED.

---

## Sellos verificados / espera CI — 17 noviembre 2026, 13:09 MYT / 05:09 UTC

- **0aa796b calificada y sellada en ambos roles**: 110/110 compute, 468.34 s;
  16/16 Pi, 90.31 s; verificación independiente de ambos sellos exit0.
  Compute final es `lekiwi-stage-bootstrap-0aa796b-par2.service`, inactive/result0,
  log `~/.ros/lekiwi/stage-bootstrap-0aa796b-par2.log`. Se reanudó el build sin
  modificar fuente y con CTEST_PARALLEL_LEVEL=2; todas las suites y deadlines
  originales conservados. La unidad 0aa sin par2 se detuvo antes de sus tests.
  20 bootstrap, 50 ROS, 24 recovery, 6 owner y 9 entrenamiento: cero fallos/skips.
- **Runtime e746 restaurado y ARMED**, readiness exit0. Primer arm RPC durante
  arranque fue rechazado por permisos aún no publicados; auto-arm normal se
  recuperó y el RPC posterior dio success=True. Probe
  `primary-restored-predeploy-0aa796b-20261117.json`: fresh stow .012322 rad,
  permisos true, tres cámaras frescas, collision clear. Nada físico admitido aún.
- PR #192 sigue draft/head0aa/main5f0b79. Hosted build/tests, device y static pasan;
  **cobertura en curso**, cuatro checks CodeQL ya verdes. Run **44020141121**.
  Falta terminar CI, ready/merge expected0aa, main tree equality, deploy canónico.
- Helpers **aún NO ejecutados** de 0aa: primer ensayo120s activos, segundo con DB
  nueva240s activos (wall300), **ambos radio2.0m**, igual al trial previo sin
  movimiento. Recorder nuevo `--prefix ID --duration 240` para primer ensayo,
  `--duration 360 --checkpoint <freshDB>.exploration.json` para el segundo.
  Arranque fresco helper también guarda snapshot y SHA raw del mapa primario
  tras parar su writer; restore comprueba ese SHA antes de arrancar el primario.
  No copiar los helpers viejos de e624 ni reusar IDs si ya se admitieron.

Sigue pendiente desplegar y demostrar movimiento/observaciones útiles, primero
sin frontier en el mapa actual y después con DB nueva separada; reconciliar cada
ensayo y dejar el runtime primario ARMED, tres cámaras y sensores sanos.

---

## Revisión final en calificación — 17 noviembre 2026, 12:54 MYT / 04:54 UTC

Este bloque sustituye el head e624 del checkpoint de abajo. **No se desplegó aún.**

- PR #192 actualizado a **0aa796bebe74d7d7cce926d6d6896042a74e1dfc**, fuente limpia/subida,
  mismo worktree locked `exploration-active-bootstrap-20261117`.
  Sólo cambia una expectativa en `test_exploration_recovery.py` sobre e624:
  cuando todos los targets fallan, la nueva observación local también debe intentarse
  y su fallo sigue incomplete. Ahora comprueba las dos tentativas y el mensaje nuevo.
  Ambos casos focalizados pasaron (2/2, 18.47 s); no cambió código runtime ni límites.
- e624: Pi terminó **16/16, 126.62 s**, sello verificado. Compute encontró ese único
  fallo de expectativa en recovery (23/24), se detuvo la unidad durante owner_guard;
  **e624 compute no está sellada/calificada**. Se preservan logs/resultados.
  Hosted e624 pasó 109/110 suites, único fallo el mismo test; quatro checks CodeQL
  verdes, build-test-lint falló. Log privado `bootstrap-e62411f-hosted-failure.log`.
- **Nuevos staging canónicos 0aa activos** en compute y Pi:
  `lekiwi-stage-bootstrap-0aa796b.service` y
  `lekiwi-stage-device-bootstrap-0aa796b.service`; logs homónimos en
  `~/.ros/lekiwi/stage-bootstrap-0aa796b.log` y `stage-device-bootstrap-0aa796b.log`.
  Fuentes detach/locked, entry Pi SHA c6d8665c... byte-exacto del script tracked.
  Se esperan 110 compute/16 Pi y ambos sellos; no editar fuentes de release.
- Runtime compute sigue **parado de forma planificada y robot torque-off apoyado**
  desde los dos fresh stow y disarm success descritos abajo. Selección compute/Pi
  sigue e74683f; servicios Pi activos. Restaurar current/rearm antes del deploy.
- Helpers finales **preparados, NO ejecutados** en bootstrap:
  `deploy-bootstrap-0aa796b-20261117.sh`;
  `run-bounded-explore-bootstrap-0aa796b-20261117.py`, ID
  `qualification-explore-bootstrap-0aa796b-20261117-1`;
  `run-bounded-explore-fresh-map-0aa796b-20261117.py`, ID
  `qualification-explore-fresh-map-0aa796b-20261117-1`.
  Ambos ensayos 120 s activos/radio2.5m, wall180s, admiten una vez, cancel/reconcile
  y recursos libres requeridos. Recorder nuevo
  `record-bootstrap-motion-and-growth-20261117.py --prefix ID --duration240`
  mide odom y área del mapa y acepta `--checkpoint` para el mapa nuevo.
- Para demostrar el caso nuevo físicamente, helpers preparados
  `start-fresh-map-bootstrap-0aa796b-20261117.sh` y
  `restore-primary-map-after-bootstrap-0aa796b-20261117.sh`: usan el launcher tracked
  `ros-start.sh` y su argumento existente rtabmap_database, DB separada
  `bootstrap/fresh-map-bootstrap-0aa796b-20261117/rtabmap.db`, unidad user
  `lekiwi-qualification-fresh-map-0aa796b`. Exigen dos stow frescos/disarm success
  antes de cada parada y restauran servicio principal + rearm/readiness.
  No arrancar esa unidad hasta desplegar/validar la candidata canónica.
- Backup consistente del mapa principal antes de ensayos:
  `map-before-bootstrap-fix-20261117/rtabmap.db`, integrity ok, 17,530,880 bytes,
  SHA **71820a0a7814cbd1ae18c474b8514e959248686656f559587c5120be40fcf116**.
  Memoria espacial: `spatial-before-bootstrap-fix-20261117.sqlite3`, integrity ok,
  SHA **8b98abff02270a1a36f1d848d3a89ba20b506a8f9c3a1347badb2c0a662cd546**.
  No reemplazar el mapa de producción ni borrar historia para obtener aceptación.

Próximo: terminar y verificar 0aa compute/device + CI; review/merge expected head;
restaurar current/rearm, deploy canónico, preflight/recorder/ensayo inicial sin
frontiers, reconciliar; ensayo con DB nueva separada, reconciliar y restaurar
mapa/runtime/ARMED. **No terminar sin movimiento y observación útiles demostrados.**

---

## Corrección activa — 17 noviembre 2026, 12:44 MYT / 04:44 UTC

El usuario rechazó expresamente dar por terminada la autonomía con cero targets
alcanzables. **La aceptación física sigue abierta; no cerrar con ese resultado.**

- PR #192 draft: https://github.com/robotmindio/cleanroom/pull/192, head
  **e62411f7a7ea1fe50f675eaa2d904e8845217020**, rama limpia/subida y worktree locked
  `/home/nex/roboto/cleanroom-worktrees/exploration-active-bootstrap-20261117`.
  Corrige `target=None => complete=true`: frontera desconocida o borde de primer
  mapa exige observación. Spin 45° bajo padre NavigateToPose con lease; tras ocho
  vistas, avance o retroceso 15 cm a 5 cm/s sólo sobre rayos láser medidos libres.
  Giros usan el costmap local fresco, todos los movimientos verifican huella barrida,
  y quedan activos collision monitor, stow, freshness, región y presupuesto originales.
  No cambia límites, 3D search evidence ni el mapa global para fabricar free space.
- Diagnóstico readonly real: costmap local sin obstáculos; los rayos medidos
  permiten **retroceso** de .15 m. La zona frontal del scan contiene rayos ausentes
  o tapados por el cuerpo y sigue desconocida para la traslación conservadora.
  Evidencia `bootstrap-local-grid-measured-v2-20261117.{json,npz}` en bootstrap.
- Pruebas focalizadas: 38 pure; 50 ROS existentes; 12 casos del navegador nativo
  (incluye tres nuevos, ahora registrados en bootstrap); owner restart con primer
  mapa y guard reiniciado; finalmente **20 bootstrap, 121.28 s, cero fallos**.
  Se preservan dos fallos intermedios de aserción de número exacto de giros: bajo
  carga se pausó correctamente por copia vieja del scan. Ahora se usa/valida el
  scan más reciente tras responder el costmap y se revela el mapa en el test por
  la traslación efectiva, sin relajar freshness. Ruff completo pasa.
- Calificación canónica **en curso, no sellada aún**: unidad compute
  `lekiwi-stage-bootstrap-e62411f.service`, log `~/.ros/lekiwi/stage-bootstrap-e62411f.log`;
  Pi `lekiwi-stage-device-bootstrap-e62411f.service`, log homónimo `stage-device-*`.
  Entry Pi byte-exacto de stage-release (SHA c6d8665c...); fuentes detach/locked.
  Reusa overlays nativos; se esperan 110 suites compute y 16 Pi. CI PR en curso.
- **Runtime compute temporalmente parado para liberar recursos**, selección de
  ambos hosts sigue e74683f, sin deploy nuevo. Antes de parar: fresh travel_stow
  .5 s pasó, `/safety/disarm` success=True confirmó todos los servos torque-off,
  segundo fresh travel_stow pasó. Unidad compute inactive. Pi conserva sus cinco
  servicios activos. Root Pi ~2.2G después de comenzar build. Restaurar compute y
  rearmar e746 antes del deploy canónico; no dejar este estado como final.
- Wrapper preparado **no ejecutado** `bootstrap/deploy-bootstrap-e62411f-20261117.sh`.
  Ensayo preparado **no admitido** `run-bounded-explore-bootstrap-e62411f-20261117.py`,
  ID nuevo `qualification-explore-bootstrap-e62411f-20261117-1`, 120 s activos/radio2.5m,
  fence e624, recorder requerido, wall180s con cancel/reconcile y stop confirmado.
  Medir giro, traslación y observaciones reales; no basta el resultado terminal.

Próximo: terminar/verificar los dos sellos y CI; revisión propia/merge con expected
head e624; restaurar runtime/rearm; deploy canónico con stow fresco, readiness;
ensayo único con recorder y validar exploración útil, corrigiendo lo que falle.

---

## Estado final — 17 noviembre 2026, 11:07 MYT / 03:07 UTC

Este bloque sustituye los pendientes de calificación/deploy y wrist de abajo.

- **PR #191 mergeado** con expected head
  **e74683fce67a464377fd5e28411f1412af3b7672**. Merge/main/origin-main:
  **5f0b79f2bf55bbf6d814e373c480db4207d72fad**, limpio, árbol igual a e74683f.
  #190 está mergeado como 2ab33a0; ambos fixes están en main.
- Calificación canónica e746: **109/109 compute**, cero fallos, **1637.01 s**;
  **16/16 Pi**, cero fallos, **91.36 s**. Ambos sellos verificados. Compute incluyó
  **50 casos exploration_ros** y **9 casos de entrenamiento**, cero skips.
  Los cinco checks hosted pasaron (build-test-lint 22m45s; CodeQL). Se conservaron
  los diagnósticos fallidos previos, sin cambiar deadlines ni límites de seguridad.
  Entrenamiento pasó en 259.62 s dentro de su límite existente; hubo una breve
  parada de compute por presión de recursos, con dos gates stow frescos y disarm
  RPC success. Se restauró 9503b92 a ARMED antes del deploy canónico de e746.
- **e74683f está desplegada en compute y Pi**. Current y marcadores de ambos roles
  coinciden; **previous=9503b92** en ambos. Deploy canónico pasó sin retry, hizo los
  checks travel_stow antes de cortes/reintentos, readiness dos veces y rearm.
  Log: `bootstrap/deploy-mapping-start-e74683f-20261117.log`.
- Estado final fresco: **ARMED**, arm/base permissions y arm_stowed true;
  max error SRDF travel_stow **0.012322 rad**, dentro de 0.02. Front GENERAL,
  wrist JYU2C y Astra frescos; lidar/depth sanos; collision clear. Pi kernel
  **6.8.0-1068-raspi, DMA=40**, módulo persistente y arranque verificado, cinco
  servicios activos. Runtime compute activo. Root Pi ~2.3G libre.
- Mapper vivo: **approx_sync_max_interval=0.15**, queue=15, nuevo prefijo nativo
  ya verificado; allow_search=true y search_geometry_max_age_sec=1.0. Cadencia
  post-e746 de 30 s: 300 scans, 277 depth clouds, 56 RGBD, 33 maps/graphs/octrees;
  scan max age .1112 s, depth .0332 s, RGBD .5953 s, mapas .6583 s. 600 estados
  collision clear, cero intrusiones. Es evidencia de ese intervalo, no de horas.
- **Explore generación 56**, ID
  `qualification-explore-autonomy-e74683f-20261117-1`, admitido una sola vez con
  25 s activos/radio 2 m. Mostró `starting: waiting for mapping pose to settle`,
  capturó centro **(-0.4730558, 0.0174737)** después de inicializar y terminó
  **succeeded, complete=true**, mensaje `no further reachable observation targets`,
  **visited_targets=0**, stop=confirmed, recursos liberados. Entre admisión y
  terminal: **129 muestras odom, recorrido=0 m, desplazamiento=0 m**, max stamp age
  .003244 s; 43 muestras map/TF y checkpoint. Esto verifica inicialización y fin,
  **no movimiento de base ni exploración útil/cobertura de la sala**. El .805 m²
  es el área regional del resultado; no llamarlo crecimiento de mapa.
- El inspector readonly tampoco encontró targets en el mapa de localización para
  radios 2, 2.5, 3 o 4 m, manteniendo footprint/clearance/margen. No se amplió el
  radio del ensayo ni se marcó espacio desconocido como libre. La aceptación de
  movimiento/exploración útil sigue abierta: investigar conectividad del espacio
  conocido alrededor de la huella y coherencia/posición del mapa. No forzar drive
  crudo ni reducir máscaras/StopZone/freshness para obtener movimiento.
- Snapshot final: `continue-20261117-final-e74683f.json` y tres JPGs; Fiber
  `final-e74683f-capabilities-20261117.json` confirma base/arm/perception_query libres.
  Trial completo y resumen: prefijo `qualification-explore-autonomy-e74683f-20261117-1`.
  Recorder incluyendo map/TF terminó exit0; no tareas de movimiento, recorders,
  archivadores, timer USB ni staging propios activos. La unidad compute terminó
  inactive/result0; el staging Pi terminó antes del reboot. CI automático de main
  puede ejecutarse por los merges; los heads revisados/calificados ya pasaron.

### Continuación y fuentes conservadas

- Worktree limpio/subido/bloqueado de #191:
  `/home/nex/roboto/cleanroom-worktrees/exploration-mapping-startup-20261117`.
  #190: `/home/nex/roboto/cleanroom-worktrees/verified-usb-recovery-20261117`,
  head 67530c3, limpio/subido/bloqueado. No editar releases seleccionadas/selladas.
- Handoff/WIP/ROLLOUT de ops se actualizan en este checkpoint; mantener WIP para
  aceptación física real de explore/find, calibración y skills entrenadas.
  Reach y skills generales siguen deshabilitados; no se cambió esa política.
- No pedir más ajustes manuales a esa postura apoyada. El recovery USB admite
  poses estables con **los nueve motores ya torque-off**, sin cortar torque de
  un brazo alimentado; el builder recupera sólo un PCI controlador no usado,
  separado del motor. Cortes alimentados/deploy/reboot conservan fresh travel_stow
  estricto. El retorno nominal ya se hizo por MoveIt 0.1 scaling, SUCCESS.
- Backups/evidencia del Pi, snapshot SQLite válido con 56/56 IDs previos retenidos,
  recibos de prune y archivo verificado de 92 journals siguen en bootstrap. Root
  source/.env, current/previous, CAD y configuraciones de seguridad se preservan.

---

## Verificación de arranque — 17 noviembre 2026, 10:35 MYT / 02:35 UTC

- El Pi se reinició de forma controlada para verificar persistencia del fix. Gate
  travel_stow fresco antes de disarm, RPC success, **nueve servos torque-off**
  frescos y recursos libres, otro gate fresco antes de reboot. Ambos pasaron.
- Boot ID cambió a `e2411503-fec9-47cc-8c75-2d6d2ebd4102`, kernel sigue
  6.8.0-1068-raspi y el VL805 arrancó automáticamente con **DMA=40**, driver bound,
  módulo de `updates/lekiwi`. Astra, CP2102 y **wrist JYU2C volvieron**. No hay error
  de setup ni -71 en ese arranque. Las cinco unidades de dispositivo están activas.
- Se ejecutó el rearm normal del repo tras verificar sensores; RPC success.
  Probe `continue-20261117-kernel-reboot-rearmed.json`: ARMED, arm/base permissions
  true, arm_stowed true, max error .012322 rad, front/wrist/Astra frescos, collision
  clear. Por tanto el pendiente de wrist ausente de los bloques viejos está resuelto.
  Evidencia: `pi-kernel-reboot-{before,after}.txt`, `before-kernel-reboot.json`,
  `pi-kernel-reboot-rearm.log` y prefijos `continue-20261117-*kernel-reboot*` en bootstrap.
- **e74683f device pasó 16/16** en 91.36 s, se selló y verificó independientemente.
  La unidad transient de staging terminó antes del reboot; su registro systemd
  puede desaparecer por ese reboot, pero log/XML/sello/fuente locked permanecen.
- **Los cinco checks hosted de #191 ya están verdes**: compute/device build/test,
  static, coverage y CodeQL. El PR sigue draft hasta terminar la calificación local
  compute. Compute completó build y está ejecutando **109 suites**; recovery pasó
  104.95 s, owner guard 163.33 s, y está entrando en exploration_ros. Sin fallos
  reportados hasta ese punto. Unidad `lekiwi-stage-mapping-start-e74683f.service`
  sigue activa; no repetir staging ni modificar la candidata.
- El runtime ROS seleccionado sigue **9503b92 en ambos hosts**, rearmado. No se ha
  ejecutado todavía el wrapper e746 ni una segunda prueba física. Gen55 terminal
  y recursos libres. Siguiente paso: terminar/verificar sello compute, revisar y
  mergear #191 con head e746, comparar árbol de main, deploy canónico y nuevo trial.

---

## Actualización activa — 17 noviembre 2026, 10:16 MYT / 02:16 UTC

- **PR #190 mergeado** con head guard tras cinco checks verdes (build-test-lint
  21m49s, compute/device, static, coverage y CodeQL). Merge/main/origin-main:
  **2ab33a01d59d5b6c8d845347b0771a410ccfc44b**. Main limpio. El backport del VL805
  sigue aplicado y persistente; el robot sigue seleccionado/rearmado en 9503b92.
- **PR #191 draft** https://github.com/robotmindio/cleanroom/pull/191,
  head **e74683fce67a464377fd5e28411f1412af3b7672**, rama limpia/subida
  `fix/exploration-mapping-startup`, worktree bloqueado
  `/home/nex/roboto/cleanroom-worktrees/exploration-mapping-startup-20261117`.
  Contiene sólo dos archivos sobre main: exploration_node.py y test_exploration_ros.py.
  Espera observaciones map/SLAM/TF posteriores al cambio de modo y pose estable
  antes de fijar el centro. Usa settle_sec y tolerancias SLAM existentes; feedback
  starting para inicialización, paused para fallos reales. No cambia radio/margen,
  freshness, cuotas, cancelación, default armed ni allow_reach.
- Tres casos ROS reprodujeron el defecto en la fuente anterior (3 failed) y pasan
  con el cambio. Se añadieron pose correction tardía y cancelación al iniciar.
  Una ejecución combinada dio dos expectativas de feedback y un test unitario
  cuyo deadline de módulo envejeció tras >100 s de ROS; los 38 unitarios pasan
  ejecutados por archivo como CTest. Se corrigió el feedback con MappingStarting.
  Otra ejecución encontró el logger rclpy de severidad dinámica; arreglado con
  call sites separados. Los **seis casos de pause/recovery que fallaban pasan**
  en 34.21 s. Ruff pasa. No presentar todavía todo el head final como calificado:
  la matriz completa final se está ejecutando en staging canónico.
- **Staging compute activo**: `lekiwi-stage-mapping-start-e74683f.service`, Nice10,
  log `~/.ros/lekiwi/stage-mapping-start-e74683f.log`; release source bloqueado.
  Build del paquete aún avanzaba (66% a 10:14), sin fallo. Reutiliza overlay nativo
  9503b92; no se reconstruye la seleccionada.
- **Staging device activo**: `lekiwi-stage-device-mapping-start-e74683f.service`,
  log Pi `~/.ros/lekiwi/stage-device-mapping-start-e74683f.log`, entry canónico
  `~/.ros/lekiwi/stage-device-mapping-start-e74683f-entry.sh` (hash c6d8665c... igual
  al script tracked). Bundle e746 transferido, fuente detach/locked. No repetir
  staging mientras esté activo. Hosted #191: cuatro checks CodeQL verdes; build/test
  y coverage en curso. Verificar ambos sellos y todos los checks antes de merge/deploy.
- Wrapper de deploy nuevo **preparado, no ejecutado**:
  `bootstrap/deploy-mapping-start-e74683f-20261117.sh`. Exige árbol igual a main,
  fuente limpia/head correcto/sello y usa deploy-split canónico más ssh-agent propio.
  Tras merge #191 comparar el árbol; no ejecutar antes de la calificación completa.
- Journals **ya archivados y vacuum completado**. El primer export textual terminó
  SIGTERM/143 a los 15 minutos; se conserva como `.export.partial.gz`, no es backup
  completo. Se rotaron journals y archivaron 92 archivos binarios: archivo privado
  `pi-journal-archived-20261117.tar.zst`, validación zstd + tar completa,
  **3,607,244,800 bytes** descomprimidos, SHA256
  **c8387a86682d4ea7250c2f533480a0d8f5358232c26dde124183d2c321e5f4f3**.
  `journalctl --vacuum-size=1G` liberó **2.1G**, dejando root Pi ~2.5G libres antes
  del nuevo build, ~1GB journals. Recibo `pi-journal-vacuum-20261117.log` y `.files`.
  Los procesos de export/archive/timers USB ya terminaron; sólo staging/CI propios
  quedan activos, no goal físico ni recorder.
- Snapshot consistente de la base después del fallo de transición:
  `bootstrap/map-after-mapping-transition-20261117/rtabmap.db`, integrity=ok,
  SHA256 **7c954eafd80ac7fbeae0c94c37a07eb3ac91817c9bfa13c88652bde32bf998b3**.
  65 nodos, **los 56 IDs del backup anterior retenidos**, ninguno perdido. El
  primer Node.time_enter fue actualizado por RTAB, explicando el aviso de cambio
  de identidad; no afirmar que se borró la base. Ese tag merece seguimiento para
  spatial memory, separado de la coherencia inicial que #191 está corrigiendo.

Próximo: terminar calificación/CI, verificar seals, revisión propia explícita y
merge #191 con expected head. Después deploy canónico con fresh travel_stow,
readiness y rearm; repetir sensor/pose/mapping checks y un único Explore con nuevo
ID y recorder. Gen55 ya está terminal, stop confirmado y recursos libres; no
reusar su ID/UUID. Wrist JYU2C sigue ausente (-71), front GENERAL/Astra/lidar sanos.

---

## Checkpoint actual — 17 noviembre 2026, 09:51 MYT / 01:51 UTC

Este bloque sustituye el estado offline y los pendientes de device/deploy de abajo.

- **9503b92f275cbbdb4fc7652947eaa27c7dff5f1a está desplegada en compute y Pi**, con
  current/marcadores verificados y previous=fa91885. La candidata pasó 107/107 compute
  y ahora **16/16 device**; ambos sellos verificados antes del deploy canónico.
  Deploy sin retry, readiness dos veces, rearmado. Log privado:
  `~/.ros/lekiwi/qualification/20261116-bootstrap/deploy-native-sync-9503b92-20261117.log`.
  El mapper vivo declara **approx_sync_max_interval=0.15**, queue=15 y carga la
  biblioteca de `compute-f1a97b18aab5517c92f3726350697741`. Search sigue enabled.
- Se recuperó el enlace Pi directo por tailnet/LAN. La falta de Astra **y lidar**
  era una regresión del kernel Ubuntu 6.8.0-1068 frente a 1065: el VIA VL805 PCIe
  `1106:3483`, dirección `0000:01:00.0`, fallaba probe con `can't setup: -12` y DMA=36.
  RAM está en la ventana PCIe de 37 bits. Se aplicó el cambio upstream a DMA=40,
  compilado con headers/compiler exactos y fuentes Ubuntu verificadas por SHA256.
  Cargó sin reboot ni reiniciar motores; Astra y CP2102 aparecieron inmediatamente.
  Se instaló override `/lib/modules/6.8.0-1068-raspi/updates/lekiwi/xhci-pci.ko`,
  depmod y update-initramfs completados; initrd root y firmware coinciden e incluyen
  el módulo. SHA256 módulo: c5839240b45b01194962b7b8e3b7cde01dfa909e730e6e7cd26079720b65457b.
  No volver a usar --activate con ese controlador ya bound/en uso.
- **PR #190 draft**: https://github.com/robotmindio/cleanroom/pull/190, head
  **67530c3a14c11ba5e67a30fb8142d71683de02f4**, rama subida/limpia
  `fix/verified-usb-recovery`, worktree bloqueado
  `/home/nex/roboto/cleanroom-worktrees/verified-usb-recovery-20261117`.
  Incluye builder del backport exacto y recovery USB verificado torque-off. 139
  pruebas locales relevantes y Ruff completo pasan. Los checks CodeQL pasaron;
  build-test-lint de este head sigue en curso. Main sigue b8984e4; no mergear hasta
  inspeccionar CI final. El backport de sistema ya está aplicado y documentado.
- El usuario confirmó la postura apoyada como válida y pidió no más ajustes a mano.
  Se verificaron los nueve servos **ya torque-off**, DISARMED, joints estables,
  permisos falsos y recursos libres. Recovery acepta esa situación sin exigir
  coincidencia angular nominal: no corta torque de un brazo alimentado. Los checks
  de torque cut/deploy siguen en 0.02 rad. Hubo dos aborts previos antes de OFF por
  recolección temporal; se añadieron regresiones. Un ciclo interrumpido por el
  timer anterior no probó VBUS completo; el **ciclo 4** sí verificó todos los hubs
  OFF y restauración. No arregló el VL805 porque éste es otro controlador PCIe.
- Tras recuperar percepción, el driver rearmó por política normal. MoveIt planificó
  y ejecutó a **0.1 velocity/acceleration** desde la postura medida a travel_stow;
  ambos resultados SUCCESS, después gate estricto pasó. Tras deploy max error
  0.012322 rad, ARMED, permisos arm/base y arm_stowed true. No se amplió tolerancia.
  El intento Fiber `reach` se detuvo antes de admisión porque allow_reach=false;
  esa política permanece intacta. El retorno fue la operación MoveIt de operador.
- El usuario movió Astra del puerto 1 al 2 del hub de expansión. Ambas funciones
  siguen detectadas (ahora 6-1.2.1/.2; no fijar estos números). **Front=GENERAL**
  está funcionando. **Wrist=JYU2C** dejó de enumerar con errores USB -71; no confundir
  sus roles. Cámara de muñeca no requerida para este ensayo de base, sí pendiente
  para manipulación. Astra, depth cloud y lidar están sanos.
- Cadencia postdeploy de 30 s: 301 scans, 279 depth clouds, 56 pares RGBD; scan age
  max .1135 s, depth cloud .0493 s, RGBD .5998 s, mapas .6841 s. 600 estados clear,
  cero intrusiones. Son datos de ese intervalo, no calificación continua de horas.
- **Explore generación 55**, ID `qualification-explore-autonomy-9503b92-20261117-1`,
  duración activa 25 s/radio 2 m: admitido/dispatched una vez, terminó FAILED con
  `exploration region stopping margin reached`, stop=confirmed, recursos libres.
  **No movimiento físico**: 86 muestras odom entre admisión y terminal, recorrido
  ~8.9e-11 m. Durante el cambio localization->mapping, map/costmap cambió de marco
  antes de capturar/usar un centro coherente. Nav2 rechazó start fuera de bounds;
  spatial_memory registró nueva identidad RTAB. Investigar transición; no ampliar
  radio/margen ni reintentar sin entenderlo. Recorder detenido, no goal propio activo.

### Trabajo activo y siguiente paso

1. Terminar y verificar el archivo privado de journals del Pi antes de vacuum:
   `pi-journal-before-vacuum-20261117.export.gz` en bootstrap. Proceso compute SSH
   237859; Pi journalctl 74588 (nice 15), gzip 74587. Exporta ~3.1G de journal;
   aún activo al checkpoint. Validar gzip/exit0, conservar hash, luego vacuum-size
   para recuperar espacio. No borrar fuentes, releases activas o pruebas.
2. Comprobar CI final de #190, revisión propia explícita y merge con expected head
   si pasa. No reconstruir ni modificar 9503b92 sellada/seleccionada. El tool nuevo
   es mantenimiento; el runtime ROS seleccionado sigue 9503b92.
3. Diagnosticar coherencia al iniciar mapping. `_execute` llama `_set_mapping(True)`
   y captura enseguida `_pose()` con TF aún válido pero previo; en ~0.5 s cambian
   mapa y pose. Leer callbacks RTAB y fixtures `test_exploration_ros.py`; añadir
   prueba que reproduzca transición con mapa/TF previos antes de cambiar runtime.
   No se ha escrito aún ningún cambio de exploración. No tarea física pendiente.
4. Repetir sólo con nuevo ID, recorder antes de admisión y cancel/reconcile con
   recursos libres. No afirmar exploración completada ni crecimiento de mapa.

Evidencia nueva: prefijos `continue-20261117-*`, `operator-return-stow-vl805-*`,
`sensor-cadence-post-9503b92-20261117.*` y `qualification-explore-autonomy-9503b92-*`
en `~/.ros/lekiwi/qualification/20261116-bootstrap/`. Build/receipt del módulo en
Pi `~/.ros/lekiwi/qualification/20261117-vl805-build/`; copia de receipt en compute.
Se retiraron sólo cuatro builds sellados superseded mediante el script tracked,
con manifest/evidencia archivados; recibos `device-sync-9503b92-prune-20261117.*`.

---

# Handoff de autonomía del robot y Fiber

## Checkpoint, 17 de noviembre de 2026, 00:07 MYT / 16 de noviembre, 16:07 UTC

**Trabajo de software completado y subido; despliegue de #189 bloqueado por Pi
inaccesible.** Esta sección sustituye todos los estados pendientes anteriores.

- PRs #186–#189 mergeados con guardia del head y checks finales verdes. #189 se
  mergeó como **b8984e4a45f9f11ee102d252007f9cce70321e08** desde head
  **9503b92f275cbbdb4fc7652947eaa27c7dff5f1a**. Main/origin/main están en b8984e4,
  limpios, y su árbol coincide exactamente con la candidata 9503b92.
- #189 obtuvo los cinco checks hosted verdes, incluidos build/test/lint y coverage.
  El staging compute canónico terminó **107/107**, cero fallos, en **806.34 s**,
  se selló y una verificación independiente del inventario pasó. Entrenamiento:
  nueve casos, cero skips, 37.10 s. Búsqueda ROS: siete casos, 26.95 s. Nueva
  regresión nativa de sync: 3.65 s. No se cambió ningún plazo ni límite de seguridad.
- Unidad **lekiwi-stage-sync-9503b92.service**: inactive, Result=success,
  ExecMainStatus=0. Log `~/.ros/lekiwi/stage-sync-9503b92.log`. Release compute
  `~/lekiwi_ws/releases/9503b92f275cbbdb4fc7652947eaa27c7dff5f1a/`; fuente bloqueada.
  Overlay completo: `~/lekiwi_ws/overlays/compute-f1a97b18aab5517c92f3726350697741/`.
  No reconstruir ni modificar esta release sellada. Se preservó el intento cef62ca
  interrumpido y el fallo diagnóstico previo de RGBDX; ninguno es calificación.
- Se completó el merge de software con CI compute/device y la calificación real
  de compute, donde cambia el runtime. La calificación real de **device sigue
  siendo requisito antes de seleccionar/desplegar**; no se intentó estando offline.
  Las revisiones publicadas se identifican como revisión propia, no independiente.

La release **fa91885** sí se desplegó y verificó antes de la pérdida de enlace,
con 106/106 compute y 16/16 Pi. Compute aún selecciona fa91885 y conserva su
marcador; previous es 2d87543. En Pi, esos mismos valores son la última verificación
histórica, no una consulta actual. #189 no se desplegó ni reinició servicios.
La prueba Explore de generación 54 registró 0.251153 m de recorrido odométrico,
0.165840 m netos y cero objetivos visitados, y terminó canceled, stop=confirmed,
sin recursos retenidos. No repetir sus IDs ni afirmar exploración completada.

**Bloqueo físico actual:** SSH tailnet volvió a agotar plazo y la LAN no resuelve
ARP (FAILED / Destination Host Unreachable). El probe final de ocho segundos
`final-sync-9503b92.json` recibió cero joints, cero imágenes, 159 invalid source
y permisos de brazo/base falsos. Postura y torque actuales **no verificables**;
la última muestra ARMED/travel_stow de abajo es histórica. No disarmar, reiniciar
motor/Astra/host ni cortar alimentación sin travel_stow confirmado desde joints
frescos, dentro de 0.02 rad. No se intentó ningún corte durante esta pérdida de
telemetría. Se pidió al usuario comprobar alimentación/Wi-Fi manteniendo la
alimentación y despejar la silla/objetos cercanos; ambas respuestas siguen pendientes.
No atribuir todos los retornos a la silla o al chasis ni ampliar máscaras/StopZone.

Fiber sigue respondiendo desde compute. El recibo final
`final-sync-9503b92-capabilities.json` conserva el mismo journal y muestra base,
arm y perception_query libres. available=true indica capacidad del servidor,
no readiness física. El stack compute sigue activo; no hay tareas de movimiento,
recorders, diagnósticos, staging o despliegues propios activos. Las unidades
fallidas históricas y los recibos originales se conservan. El push a main puede
lanzar su CI automático; el head de PR calificado es 9503b92 y su árbol ya se
comparó con el merge, sin nuevas diferencias de fuente.

### Siguiente paso al recuperar conexión

1. Confirmar current/previous/marcadores Pi, servicios, disco disponible y joints
   frescos. El espacio Pi era escaso tras fa91885: comprobarlo antes del nuevo
   overlay, sin borrar current/previous, fuentes, .env ni evidencia. Mantener el
   motor_guard tailnet; no forzar LAN abriendo sus puertos.
2. Calificar **device 9503b92** con staging canónico completo, 16/16 y sello
   verificado. Rama limpia/subida y worktree bloqueado:
   `/home/nex/roboto/cleanroom-worktrees/rtabmap-sync-interval-20261116`.
   Su .env privada está ignorada y en modo 600. Usar el helper bundle/SSH si
   GitHub aún no resuelve en Pi; no mutar releases selladas ni repetir staging activo.
3. Desde ese worktree exacto, verificar árbol de main y ambos sellos y preparar
   el wrapper canónico siguiendo `deploy-autonomy-fa91885.sh`, con head 9503b92
   y ssh-agent propio temporal para id_ed25519. Verificar travel_stow fresco antes
   de cada corte; no usar supported_rest como sustituto. No ejecutar el wrapper
   viejo de fa91885 para desplegar la nueva versión.
4. Tras deploy, comprobar marcadores/current de ambos roles y **valor real 0.15**
   en el mapper recién lanzado, prefijo de biblioteca y queue=15. En fa91885 el
   intervalo sigue sin declararse; el JSON null y la assertion fallida son evidencia
   válida del defecto, no un valor ya corregido en vivo. Repetir cadencia de sensores
   y geometría, escena/StopZone y odometría/TF/localización tras el silencio largo.
5. Sólo con las condiciones físicas y frescura resueltas, validar y enviar otro
   Explore acotado con ID nuevo, recorder incremental iniciado antes de admisión,
   plazo de pared y cancelación reconciliada. Radio 2 m permite un objetivo dentro
   del margen de parada existente de 1.1 m; no reducir ese margen. Conservar journal,
   terminal/stop/recursos y odometría del intervalo real. La calibración, views/map
   aprobados para find móvil, evidencia útil de búsqueda y skills entrenados siguen
   sin calificarse. Actualizar ROLLOUT y retirar WIP al completar esa entrega.

Fuentes propias limpias y subidas. El recibo de ajuste de I/O sólo del cgroup de
staging es `stage-sync-9503b92-io-scheduling.json`; mantuvo un compilador, Nice=10
y cambió idle a best-effort 7 para evitar inanición por otros lectores. No se
cambió configuración de runtime, DNS, firewall, frescura, colisiones ni stow.
La evidencia privada sigue en `~/.ros/lekiwi/qualification/20261116-bootstrap/`.

## Continuación, 16 de noviembre de 2026, 14:47 UTC

Actualización **14:54 UTC**: head vigente de #189 es
**9503b92f275cbbdb4fc7652947eaa27c7dff5f1a**. El build completo encontró que
RGBDXSync también usa las macros compartidas y carecía de la referencia local
node; se añadió esa referencia. El intento cef62ca se detuvo antes de pruebas y
selección; no es calificación. Log de error completo conservado. El rebuild
completo pasó, incluidos todos los plugins/ejecutables standalone, en 3 min 12 s
incrementales. La regresión de seis escenarios/tres valores inválidos y el probe
ABI del mapper empaquetado pasaron de nuevo con la biblioteca ya instalada.
Recibos `native-sync-regression-installed-9503b92.log`,
`native-sync-mapper-abi-9503b92.json` y `native-sync-diagnostic-build-nodelet-fix.log`.
El staging canónico ahora activo es **lekiwi-stage-sync-9503b92.service**, log
`~/.ros/lekiwi/stage-sync-9503b92.log`, wrapper `stage-sync-9503b92.sh` privado.
El diagnóstico ya terminó; no repetirlo. CI final activa sobre 9503b92. La Pi
sigue inaccesible; no hay calificación device ni despliegue de #189.

La Pi dejó de responder: Tailscale ping/SSH agotan plazo y la dirección LAN
192.168.110.229 no responde a ICMP ni ARP (neighbor INCOMPLETE; SSH sin ruta).
El driver registró silencio de telemetría a las 14:23:05 UTC. El probe de ocho
segundos `waiting-clearance-fa91885.json` recibió **cero joint_states e imágenes**,
160 invalid source y permisos falsos. **La postura/torque actuales no están
confirmados; ARMED y stow de abajo son la última medición histórica, no frescura
actual.** No reiniciar, disarmar ni cortar torque sin recuperar joint_states y
verificar travel_stow. Se pidió al usuario confirmar alimentación/Wi-Fi sin apagar;
también sigue pendiente el despeje físico. No se intentó ningún nuevo movimiento.
Fiber responde desde compute y sus tres recursos siguen libres; available=true
en capabilities no demuestra readiness física. Evidencia `network-loss-20261116-1440.txt`
y `offline-pi-capabilities-cef62ca.json`. Revisar odometría/TF/localización tras
recuperar enlace: los warnings muestran predicción de costmap durante el silencio.

Se implementó la corrección real del intervalo nativo en **draft
[PR #189](https://github.com/robotmindio/cleanroom/pull/189)**, head
`cef62caca2f749d18635cced4a4b07cd896f3fae`, rama limpia/subida
`fix/rtabmap-sync-interval`, worktree bloqueado
`/home/nex/roboto/cleanroom-worktrees/rtabmap-sync-interval-20261116`.
Construye rtabmap_sync 0.23.13 fijado a cb2117f y parchea las siete macros de
sync aproximado (2–8 entradas) sin añadir miembros ni métodos virtuales. Declara
approx_sync_max_interval como inmutable; cero mantiene el default previo y los
valores negativos/no finitos fallan al construir. Build, keys, staging e inventario
incluyen la dependencia. No cambia ningún límite de colisión/frescura ni fuente CAD.

La regresión C++ aislada pasó seis escenarios de sincronización y tres límites
inválidos. Con el mismo binario compilado contra headers empaquetados, la biblioteca
vieja falla por parámetro ignorado y la nueva pasa. Callbacks observados:
sin límite/0.4 s=11; límite 0.15/0.4 s=0; 0.15/0.05 s=11; 0.15/0.15 s=11;
exacto/sellos iguales=12; exacto/0.05 s=0. El mapper empaquetado real, en dominio
aislado y sin entradas del robot, cargó la biblioteca nueva (comprobado en /proc),
devolvió 0.15/queue=15/read_only=true y salió cero por SIGINT. Logs privados
`native-sync-regression-{baseline-head,patched}.log` y
`native-sync-mapper-abi-cef62ca.{json,log}`. La compilación diagnóstica completa
(incluidos plugins standalone) aún sigue; la biblioteca principal ya pasó.
126 checks iniciales de servicios/releases, diez de política y ambos casos de
invalidación de overlay pasaron, igual que ShellCheck y Ruff.

**Staging compute canónico nuevo activo:** `lekiwi-stage-sync-cef62ca.service`,
log `~/.ros/lekiwi/stage-sync-cef62ca.log`; wrapper privado
`20261116-bootstrap/stage-sync-cef62ca.sh`. Fuente de release bloqueada; no tocar
su build ni ejecutar otro staging. CI hosted sigue activo. Device staging no se
ha intentado mientras la Pi esté inaccesible. No mergear/seleccionar cef62ca sin
calificación completa de ambos roles y CI final verde. Current y marcadores
siguen en fa91885; no se desplegó #189 ni se reinició ningún servicio por ese PR.

## Continuación, 16 de noviembre de 2026, 14:30 UTC

**Desplegado y verificado en ambos equipos: `fa91885a069d92d48e9d247811fd0010763033d4`.**
Esta sección sustituye todos los estados pendientes históricos de abajo. Main y
origin/main están en `dbe89b6c0b181c84be43f4363b649f7adde7e673`; su árbol coincide
con la release. #186, #187 y #188 se revisaron y mergearon con guardia de SHA tras
sus cinco checks finales verdes. Las revisiones publicadas se identificaron como
revisión propia, no aprobación independiente.

La candidata final pasó staging canónico **106/106 compute** (1239.10 s) y
**16/16 Pi**, con ambos inventarios sellados y verificados. Entrenamiento pasó
en 125.23 s y búsqueda ROS en 31.12 s. El ENOSPC Pi de abajo se conserva como
fallo histórico; su retry terminó correctamente. No ampliar plazos ni reutilizar
como calificación diagnósticos interrumpidos o builds retirados.

El primer despliegue fa91885 falló en la comprobación posterior al reinicio:
esperaba DISARMED y observó LINK_LOST transitorio. Los checks de stow y las
peticiones de disarm habían pasado; no se cambió ningún límite. El probe posterior
confirmó DISARMED, travel_stow fresco y readiness. Un único retry canónico desde
la misma release sellada terminó correctamente, pasó readiness dos veces y rearmó.
Logs `~/.ros/lekiwi/deploy-autonomy-fa91885.log` y
`~/.ros/lekiwi/deploy-autonomy-fa91885-retry.log`. Current y los marcadores
deployed-revision de compute/Pi son fa91885; previous en ambos sigue en 2d87543.

Se ejecutaron dos tareas Explore nuevas, con una sola admisión/dispatch por ID,
25 s de presupuesto activo, plazo de pared y reconciliación de cancelación:

- `qualification-explore-autonomy-fa91885-20261116-1`, generación 53, radio 1.5 m:
  succeeded inmediatamente, «no further reachable observation targets», visited=0.
  El margen de parada existente de 1.1 m dejaba sólo 0.4 m útiles; no es prueba
  de exploración física. El recorder falló al serializar DiagnosticStatus.level
  como bytes: no hay recibo de odometría de esta primera tarea.
- `qualification-explore-autonomy-fa91885-20261116-2`, generación 54, radio 2 m:
  hubo movimiento y pausas StopZone/depth stale; se canceló al alcanzar el plazo.
  Estado final canceled, stop=confirmed, holding_resources=false; los tres
  recursos quedaron libres. Las 2075 muestras de odometría entre admisión y fin
  registraron **0.251153 m de recorrido integrado y 0.165840 m de desplazamiento**,
  giro neto -0.316195 rad y edad máxima de sello 0.0114 s. visited=0, complete=false.
  mapped_area=2.5575 m² es la métrica de la región de radio 2 m, no crecimiento
  demostrado frente a los 0.52 m² de la tarea de radio distinto.

El recorder privado se corrigió, se verificó con un smoke de 3 s y ahora guarda
JSONL incremental además del JSON final. Terminó limpiamente después de la tarea.
Recibos privados `explore-fa91885-trial2-summary.json`, ambos `*-fiber.json`,
`qualification-explore-autonomy-fa91885-20261116-2-odometry.json{,l}` y
`live-explore-map-fa91885.{json,npz}`. No volver a ejecutar los wrappers de las
identidades 1/2; ante dudas consultar esas mismas identidades.

**Parado, armado y sin tarea retenida.** El último probe de ocho segundos
(`posttrial2-fa91885.json`) confirmó travel_stow fresco, error máximo 0.009253 rad,
pero contó 122 StopZone y 38 estados libres. El muestreo de escena tuvo 45/80
scans con retornos a 0.206–0.234 m, entre 157.00 y 298.94 grados. La cámara muestra
ruedas de silla cercanas; no atribuir todos los retornos a la silla o al robot.
Se pidió al usuario apartar la silla y objetos cercanos; **respuesta pendiente**.
Mantener la base parada y repetir escena/frescura/permisos tras su confirmación.
No cambiar StopZone, máscaras ni las protecciones para permitir el ensayo.

Dos límites nuevos de verificación deben conservarse explícitos:

- En vivo, allow_search=true, search_geometry_max_age_sec=1.0 y
  search_max_candidates=200. RTAB-Map recibe sync_queue_size=15 y topic_queue_size=5,
  Astra RGBD única y ray tracing nativo. Sin embargo el RTAB-Map principal instalado
  **no declara `approx_sync_max_interval`**: la consulta devuelve not-set/null.
  El 0.15 pasado en launch NO demuestra un límite efectivo. El helper
  `postdeploy-native-parameters.py` falla su assertion correctamente; el JSON
  registra el valor real. No hay corrección nueva implementada ni override manual.
- La muestra estacionaria de 30 s posterior al despliegue conserva una interrupción
  conjunta de sensores: edades hasta 16.9 s, huecos de llegada hasta 17.4 s y
  mapas hasta 19.34 s; 336 estados invalid source. NTP/chrony no mostraron un
  error de reloj relevante. Tailscale usa DERP sin conexión directa y la Pi falla
  resolución DNS/controlplane; no está demostrada una causa única. ICMP LAN es
  rápido pero TCP LAN falla. El motor_guard permite sus puertos sólo por lo o
  tailscale0: **no ampliar esa regla para forzar LAN**. No se cambió DNS/firewall.

Fuentes y worktrees propios siguen limpios/subidos; no hay staging, despliegue,
recorder ni tarea de movimiento propios activos. Las unidades fallidas históricas
conservan su evidencia. La rama operativa guarda este checkpoint; no limpiar
worktrees o releases ajenos. Los worktrees nuevos bajo cleanroom-worktrees y las
fuentes de release fa91885 están bloqueados. Siguiente paso físico: confirmación
de despeje, probe fresco y, si es seguro, otro ensayo acotado con ID nuevo. Los
límites de calibración, find móvil y habilidades de brazo del final siguen abiertos.

## Continuación, 16 de noviembre de 2026, 13:11 UTC

**Candidata vigente: `fa91885a069d92d48e9d247811fd0010763033d4`.**
Actualización 13:22 UTC: compute pasó 47/106 y sigue activo. El primer build
Pi de fa91885 falló por ENOSPC al escribir los scripts de entorno de colcon;
no llegó a las pruebas ni obtuvo sello. Log y tar de logs conservados como
`device-fa91885-disk-full.log` y `device-fa91885-disk-full-build-logs.tar.gz`.
El prune protegido retiró sólo los builds Pi supersedidos 725a519, a27b657 y
9bd408f, preservando fuentes, .env y pruebas archivadas; current y previous no
cambiaron. Esos tres builds Pi ya no están sellados ni son candidatos activables.
Recibo privado `device-fa91885-prune-receipts.json`. El usuario normal no podía
inspeccionar /proc del systemd de usuario; el dry-run rechazó correctamente la
operación. Se repitió con el sudo existente y el mismo LEKIWI_LOGS para mantener
ambos locks, con todas las protecciones, y se devolvieron los recibos a nex.
Se borraron sólo los generados del intento fa91885 propio, fallido y sin sello,
tras archivar logs y confirmar fuente limpia, ningún proceso, current/previous
y locks. Rebuild Pi limpio activo: **`lekiwi-stage-autonomy-fa91885-retry.service`**,
log `~/.ros/lekiwi/stage-device-autonomy-fa91885-retry.log`; quedaron 572 MiB
libres antes de reconstruir. No reiniciar el primer unit ni duplicar staging.

PR #187 pasó todos los checks y se mergeó como `5c7cfc0`; main se actualizó y
su árbol coincidió con 9bd408f. Esa candidata pasó 106/106 compute y 16/16 Pi,
se selló y ambos inventarios verificaron. Entrenamiento pasó en 250.92 s y
búsqueda ROS en 29.20 s. Queda sellada, sin seleccionar, ahora supersedida.

La última revisión antes de desplegar detectó que deploy-split comprobaba stow
antes del primer corte, pero volvía a disarm después del reinicio sin comprobar
otra muestra fresca; el driver puede rearmarse al arrancar. No se ejecutó ningún
despliegue. [PR #188](https://github.com/robotmindio/cleanroom/pull/188) corrige
el helper común: antes de cada petición de corte, también en reintentos y llamadas
posteriores, exige travel_stow SRDF dentro de 0.02 rad con muestra de edad <=0.5 s.
Un fallo termina el despliegue conservando torque. Pasaron 94 comprobaciones de
servicios, incluidos seis casos nuevos que ejecutan el helper con ROS simulado;
shellcheck, Ruff y flake8 también pasan. Revisión propia publicada; CI pendiente.

Rama subida/limpia `fix/deploy-fresh-stow-check`, worktree bloqueado:
`/home/nex/roboto/cleanroom-worktrees/deploy-fresh-stow-20261116`.
`.env` privada copiada con modo 600, ignorada. Nuevos stagings completos activos
en ambos equipos, unidad `lekiwi-stage-autonomy-fa91885.service`; logs:
`~/.ros/lekiwi/stage-autonomy-fa91885.log` y
`~/.ros/lekiwi/stage-device-autonomy-fa91885.log`. Se transfirió el commit
subido por el helper bundle/SSH; las dos fuentes de release están bloqueadas.
No tocar releases selladas ni repetir staging si la unidad sigue activa.

Después de 106/106 + 16/16, sellos verificados y CI final válida, mergear #188
con guardia de SHA, actualizar main y comprobar igualdad de árboles. El wrapper
privado **aún no ejecutado** es
`20261116-bootstrap/deploy-autonomy-fa91885.sh`; revisa head/limpieza/árbol/sello
compute y abre/cierra un ssh-agent temporal. Hace el despliegue canónico con el
nuevo helper protegido. Repetir stow fresco antes de ejecutarlo.

Los helpers privados preparados (aún sin movimiento) son
`stationary-preflight.py`, `postdeploy-native-parameters.py`,
`sensor-cadence-post-fa91885.py`, `record-explore-odometry.py` y
`run-bounded-explore-fa91885.py`. El último exige marcador/current fa91885 y
recorder.ready; usa el tenant de agentes, valida y envía una sola identidad
`qualification-explore-autonomy-fa91885-20261116-1`, 25 s activos y radio 1.5 m,
observa hasta 60 s de pared y cancela/reconcilia si sigue activo. Revisarlo antes
de uso. No repetir la admisión si ya existe su JSON; consultar el mismo task_id.
El recorder usa SIGUSR1 para finalizar y guardar; también termina solo en 150 s.
Luego verificar terminal, stop=confirmed, recursos libres y odometría/journal.

Ambos current y marcadores siguen en 2d87543. El probe precut-9bd408f confirmó
ARMED, 160 estados de colisión libres y travel_stow fresco (máx. 0.009254 rad),
pero esa muestra ya es histórica. No hubo cortes de torque ni movimientos.

## Continuación, 16 de noviembre de 2026, 12:44 UTC

Esta sección sustituye los estados pendientes de la foto original de abajo.
PR #186 pasó todos los checks de su head revisado y se mergeó como `0bff4a2`.
Main se actualizó y su árbol coincidió con la candidata a27b657.
El staging compute de esa candidata terminó **104/106**: timeout de entrenamiento
a 300 s y dos escenarios de búsqueda sin resultado antes de sus plazos.
La evidencia original disponible (log completo, informe CTest y resultados XML)
se conserva en `20261116-bootstrap/staging-a27b657-failed/`. El LastTest.log
archivado quedó vacío por una consulta posterior de CTest; usar el log de staging.

El diagnóstico de entrenamiento pasó sus nueve casos en 179.60 s, sin ampliar
el plazo; el primer wrapper de diagnóstico tuvo un desajuste de nombre de XML,
por lo que el recibo fiable es el XML real de pytest y su salida cero, no el
resultado de ese wrapper. Durante los diagnósticos otro proceso de limpieza
eliminó el worktree limpio y la release compute a27b657 sin seleccionar.
El usuario contestó «all yours» al aviso de coordinación. Current no cambió.
No usar el diagnóstico de búsqueda interrumpido como calificación.

Nuevo fix en [PR #187](https://github.com/robotmindio/cleanroom/pull/187), head
**`9bd408f2e54652c788266b3f1942ec7120acb606`**, rama subida y limpia
`fix/autonomy-staging-fixes`, worktree persistente y bloqueado:
`/home/nex/roboto/cleanroom-worktrees/autonomy-staging-20261116`.
Agrupa la proyección de frusta en bloques de 1024 celdas y conserva la prueba
completa de volumen de cada celda contra el octree. No cambia frescura, límites,
colisiones ni postura. Ocho comparaciones con el código escalar anterior dieron
exactamente las mismas celdas; evidencia versionada en
`docs/evidence/search-visibility-batching-20261116.json`.
Pasaron 55 pruebas de geometría, 16 de reproyección ROS y siete de búsqueda ROS
(suite completa 41.29 s), además de Ruff/flake8. Se observó un warning de un
future Destroyable durante el teardown, tras los siete pases. Revisión propia
publicada, explícitamente no independiente; CI de build/test sigue en curso.

Staging canónico nuevo en ambos equipos, unidad de usuario
`lekiwi-stage-autonomy-9bd408f.service`. Logs compute y Pi respectivamente:
`~/.ros/lekiwi/stage-autonomy-9bd408f.log` y
`~/.ros/lekiwi/stage-device-autonomy-9bd408f.log`.
La Pi no resolvía GitHub; se transfirió el commit ya subido con el helper
`transfer_device_revision` y SSH explícito antes de arrancar staging.
Los worktrees de fuente de ambas releases nuevas están bloqueados contra
limpieza accidental. Aún no hay selección ni despliegue nuevo.
Esperar 106/106 compute y 16/16 device, verificar sellos, y mergear #187 sólo
con CI final verde del mismo head antes del despliegue canónico desde este
worktree. La candidata a27b657 está supersedida.

La consulta Fiber real mediante la API autenticada del tenant de agentes dejó
libres base, arm y perception_query; explore/navigate/search disponibles.
El helper privado `20261116-bootstrap/fiber-action.py` captura el JWT en memoria
y usa las acciones rosbridge, sin imprimir ni persistir credenciales.
El probe `stationary-preflight.py --prefix predeploy-9bd408f` confirmó ARMED,
permisos, 161 estados de colisión libres y travel_stow fresco, error máximo
0.009254 rad; repetir justo antes de cortar torque. Las escenas guardadas se
inspeccionaron: no se ve el cargador/cable previo. No se ha enviado movimiento.

Estado consultado el 16 de noviembre de 2026 a las 10:50 UTC. Repositorio:
`/home/nex/roboto/cleanroom`, GitHub `robotmindio/cleanroom`.

La exploración sin mapa previo y la retención de geometría de búsqueda están
implementadas, revisadas y mergeadas. Falta terminar la validación de la versión
conjunta, mergear el PR de cierre de RViz, desplegar y repetir Explore en el robot.
**La candidata todavía no está seleccionada en ninguno de los dos equipos.**

## Objetivo y autorización

El usuario pidió resolver los pendientes de robot/Fiber, activar la búsqueda,
aplicar los cambios de SLAM y revisar y mergear nuestros PR. Exigió que Explore
pueda iniciar sin cobertura válida ni mapa previo. No hace falta solicitar otra
aprobación para revisar, mergear, desplegar u operar el robot dentro de ese alcance.

El usuario identificó el bloque negro y el cable cercanos como el cargador conectado.
Después confirmó que desconectó el cargador y apartó el cable. No quedan preguntas
pendientes. Hay que comprobar de nuevo la escena y los permisos antes de mover.

## Reglas que condicionan la continuación

- Leer `AGENTS.md` y respetar los controles de colisión, frescura, Nav2 y postura.
  No ampliar filtros ni tolerancias para conseguir que una prueba avance.
- Antes de disarm, reiniciar motores/Astra/host o cualquier corte de torque,
  confirmar con `/joint_states` frescos la postura SRDF `travel_stow`, dentro de
  **0.02 rad**, con el brazo apoyado. `supported_rest` no sustituye esta condición.
  Si no cumple, llevarlo mediante MoveIt a `travel_stow` o no cortar torque.
- El comportamiento normal mantiene el robot armado; un fallo debe conservar
  torque. Usar el despliegue canónico, que comprueba postura y rearma al terminar.
- Configurar el comportamiento en archivos versionados y lanzadores del repositorio.
  No sustituir una corrección por cambios manuales de parámetros o widgets.
- Mantener las geometrías CAD vendorizadas. Los ajustes ROS pertenecen al xacro.
- No reconstruir ni modificar una release sellada o seleccionada. Preservar
  evidencias de fallos; no subir secretos, `.env`, manifiestos privados ni core dumps.
- Usar worktrees persistentes, commits recuperables y ramas subidas. No delegar
  en subagentes sin autorización explícita aplicable.

## Cambios entregados y PR abierto

| PR | Estado | Resultado |
| --- | --- | --- |
| [180](https://github.com/robotmindio/cleanroom/pull/180) a [183](https://github.com/robotmindio/cleanroom/pull/183) | Mergeados y desplegados previamente | Búsqueda nativa habilitada, geometría Astra, pruebas y presupuesto CTest, reconocimiento de `supported_rest` y recarga segura del planner. |
| [184](https://github.com/robotmindio/cleanroom/pull/184) | Revisado y mergeado | Explore activa el mapeo antes de exigir el primer mapa o pose localizada. |
| [185](https://github.com/robotmindio/cleanroom/pull/185) | Revisado y mergeado | Conserva metadatos de geometría durante la latencia de inferencia y acota sincronización RGBD/rango. |
| [186](https://github.com/robotmindio/cleanroom/pull/186) | Revisión propia publicada, abierto | Corrige dos fallos de cierre de RViz; CI completa todavía en ejecución al consultar. |

`main` y `origin/main` están en
`276efa0029809f892ad3f6e3656ab9ecd93c0d84`.
El merge de #184 fue `0d706b5664dc014961acf9e4355e25a427a56e97`.

### Explore sin mapa ni cobertura previa

El círculo de espera era: Explorer exigía mapa/Nav2 antes de activar mapping,
y Nav2 esperaba el primer mapa. Ahora admite la inicialización si falta mapa
o hay un mapa guardado sin localización, y activa mapping antes de pedir la pose.
Cada movimiento sigue pasando el checkpoint completo; no se inventan TF ni espacio
libre. Cancelación, plazo y recuperación del modo anterior siguen cubiertos.

Se revisaron también el workflow y `task_gateway.py`: Fiber admite y despacha
Explore sin mapa aprobado ni cobertura de búsqueda. Las exigencias de un mapa
aprobado para ciertos objetivos de alto nivel no son requisitos de Explore nativo.

Las pruebas incluyen ausencia de mapa/TF, mapa guardado sin localizar, cancelación
antes del primer mapa, StopZone y vencimiento sin mapa. Pasaron 69 comprobaciones
ROS/recuperación en la revisión final; CI hospedada de #184 pasó antes del merge.
La prueba física de arranque de Explorer con una base de mapas vacía sigue pendiente;
no borrar la base de producción para simularla.

### Geometría y sincronización de búsqueda

`SearchEvidence` conserva metadatos hasta la capacidad ya existente del registro
de capturas, en lugar de solo cuatro escenas. Mantiene los límites independientes
de cuatro árboles decodificados y 32 MiB de payload, coincidencia de revisión y
geometría no futura con antigüedad máxima de un segundo respecto de la captura.

Las topologías elegibles de búsqueda con rango usan `sync_queue_size=15` y
`approx_sync_max_interval=0.15`; la cola de transporte sigue en cinco. Se actualizaron
las siete instantáneas de lanzamiento. Pasaron 16 pruebas de geometría/reproyección
y 79 de configuración/lanzamiento; CI completa de #185 pasó antes del merge.

La comparación nativa aislada con los mismos sensores durante 50 s produjo
49 mapas frente a 30; el intervalo máximo bajó de 3.759 s a 1.786 s. Esto mejora
la cadencia, pero no demuestra frescura continua de un segundo ni calibración física.
Evidencia versionada: `docs/evidence/search-sync-latency-20261116.json`.

Las matrices manuales largas de búsqueda fallaron localmente en plazos de costmap
y teardown: una ejecución dio 5 fallos, 56 pases y 5 errores; otra con el Python
canónico dio 5 fallos, 2 pases y 5 errores. El primer caso seleccionado también
falló sobre main sin estos cambios. No se atribuyó una causa sin demostrarla ni se
ampliaron los plazos para ocultarlo. La batería completa de staging actual es la
condición independiente para seleccionar la candidata.

### RViz y condición de merge de 186

Head revisado: `1d369a14ab7a14f5d2c4d2706835255f3d458641`.
Se reprodujeron bajo debugger dos fallos distintos: desconexión del filtro de
imagen después de destruirlo y descarga de Cyclone DDS mientras seguía ejecutando
un hilo receptor. `lekiwi_rmf/Image`, exportado desde la biblioteca de plugins
existente, desconecta y limpia el callback antes de la destrucción de las bases.
Los tres paneles de cámara usan esa clase. `scripts/rviz.sh` mantiene la biblioteca
Cyclone configurada cargada durante la vida del proceso; es un workaround documentado
de finalización, conserva las señales y los códigos reales de salida.

Pasaron la regresión nativa, diez controles de aislamiento, 31 comprobaciones de
colisión/layout y shellcheck. El plugin final con cámaras reales cerró con SIGTERM
y SIGINT, retorno cero, sin timeout, en 7.05 s y 2.80 s. Son dos ensayos, no una
calificación de resistencia. Evidencia: `docs/evidence/rviz-shutdown-20261116.json`.

La primera CI pasó 105/106 y detectó que la nueva prueba nativa no utilizaba el
runner aislado. La inscripción fue corregida y comprobada. La CI del head final
tiene CodeQL completo en verde y `build-test-lint` en curso:
[ejecución 43847812205](https://github.com/robotmindio/cleanroom/actions/runs/43847812205).
El comentario de revisión ya está publicado; no quedan hallazgos bloqueantes en
el diff revisado. Mergear únicamente con los checks completos verdes de ese head.

## Robot y pruebas físicas

Ambos enlaces `current` siguen seleccionando la release
`2d87543c0c3e770e68b969c6e531d01fc71c27f7`, previamente verificada y desplegada.
El árbol equivale al merge anterior `0b36c5c`. No se desplegaron #184, #185 ni #186.

El brazo se había desviado 0.022 rad de `travel_stow`, lo que interrumpía correctamente
los inputs de mapping. Se corrigió con un plan y ejecución MoveIt protegidos, a
escala 0.10 y sin cambiar tolerancias ni cortar torque. Una comprobación fresca
posterior pasó. Repetirla justo antes del despliegue; la causa de aquella deriva
no quedó demostrada.

Después de desconectar el cargador, la muestra de ocho segundos tuvo 80 escaneos,
cero intrusiones y 160 estados libres del monitor de colisión. No se amplió ningún
filtro. La coincidencia temporal no demuestra la identidad de todos los retornos
anteriores. Antes hubo un retorno de 0.197–0.200 m a 298.81 grados que el CAD no
identificó concluyentemente; no enmascararlo sin medición.

La última consulta Fiber dejó libres `base`, `arm` y `perception_query`. Estaban
disponibles `explore`, `navigate` y `search`; `skill` y `reach` no estaban disponibles.
No se dejó una tarea de movimiento activa ni se envió el giro diagnóstico propuesto.

La tarea previa `qualification-explore-zero-coverage-20261116-1`, generación 52,
fue admitida y despachada por la release anterior con cobertura de búsqueda cero;
pausó por fuente de seguridad inválida y se canceló. Resultado terminal `canceled`,
`stop=confirmed`, `holding_resources=false`, área reportada 2.56 m² y cero objetivos
visitados. No es una prueba de desplazamiento autónomo con la nueva implementación.
Prueba privada: `explore-predeployment-proof.json` en el directorio de evidencias.

La búsqueda nativa anterior, trial 3, también terminó cancelada con parada confirmada,
recibo estable y sin doble dispatch al repetir la petición. Recorrió 0.013664 m con
límite de 0.12 m. Cobertura cero significa `no_coverage`, nunca ausencia del objeto.
En el contrato, `data.found` de status indica que existe la fila de tarea; el
resultado del objeto está en `data.search`.

## Candidata y procesos que siguen ejecutándose

La rama `release/robot-autonomy-20261116` está subida y limpia en
**`a27b65745539c05ce6ed9a45a598fa90b5738750`**. Su árbol fue comprobado idéntico
al del head final de #186; incluye #184 y #185. Después del merge de #186 hay que
comprobar que también coincide con main. Los SHA pueden diferir por el squash.

| Equipo | Estado consultado | Unidad y log |
| --- | --- | --- |
| Compute local | Staging activo; 70/106 completadas, `test_perception_training` en curso | Unidad de usuario `lekiwi-stage-autonomy-a27b657.service`; `~/.ros/lekiwi/stage-autonomy-a27b657.log` |
| Pi `nex@100.107.20.66` | 16/16 pasaron; unidad inactiva, salida cero; inventario sellado verificado | Misma unidad de usuario; `~/.ros/lekiwi/stage-device-autonomy-a27b657.log` |

Los procesos de staging continúan sin seleccionar la candidata. La suite compute
completa debe terminar **106/106** y sellarse; cualquier fallo bloquea el despliegue.
Las releases están en `/home/nex/lekiwi_ws/releases/<SHA>` en ambos equipos.
No lanzar otro staging mientras el actual posea el lock.

La candidata anterior `725a5196e1b61f499248290d25dedee5da5034ee` quedó supersedida:
su staging compute fue detenido antes de selección y su Pi pasó 16/16. No desplegarla.

La Pi tenía poco disco. El script protegido `scripts/prune-release-builds.py`
retiró los builds de cuatro releases históricas selladas, archivó evidencia y dejó
intactos fuentes, worktrees, `.env`, overlays, current y previous. Recibos privados
en `device-prune-receipts.json`; no continuar borrando releases indiscriminadamente.

## Secuencia para continuar

1. Actualizar el estado real de staging y CI; las cifras anteriores son una foto.
   Si una prueba falla, leer su evidencia y corregir la causa antes de seleccionar
   nada. No ampliar plazos ni ignorar fallos para fabricar una release válida.

   ```bash
   systemctl --user show lekiwi-stage-autonomy-a27b657.service -p ActiveState -p ExecMainStatus
   tail -n 15 ~/.ros/lekiwi/stage-autonomy-a27b657.log
   gh pr checks 186
   ```

2. Con todos los checks de #186 verdes y su head sin cambios, mergear con guardia
   de SHA. La CLI instalada no admite `gh pr checks --json`; usar `gh pr view
   --json headRefOid,statusCheckRollup` para consultar datos estructurados.

   ```bash
   gh api --method PUT repos/robotmindio/cleanroom/pulls/186/merge \
     -f sha=1d369a14ab7a14f5d2c4d2706835255f3d458641 \
     -f merge_method=squash \
     -f commit_title='fix(rviz): disconnect image filters safely and retain Cyclone through shutdown'
   git fetch origin
   git pull --ff-only
   git diff --exit-code origin/main release/robot-autonomy-20261116
   ```

   Ejecutar `pull` desde el checkout principal en main, limpio. Si aparece un
   cambio nuevo de fuente o árbol, detener la selección y recalificar lo necesario.

3. Confirmar los dos sellos de a27 con `scripts/check-release.py verify`, usando
   roles `compute` y `device`. Comprobar de nuevo `travel_stow` fresco antes de
   cualquier corte de torque. Si hace falta una corrección pequeña del brazo,
   revisar primero `restore-held-stow.py` y sus recibos; no repetir un UUID ejecutado.

4. Desplegar desde el worktree limpio de la candidata mediante el script canónico:

   ```bash
   cd /home/nex/roboto/cleanroom/.claude/worktrees/robot-autonomy-release
   scripts/deploy-split.sh nex@100.107.20.66 \
     --workspace /home/nex/lekiwi_ws --rosbridge-tailnet
   ```

   La `.env` privada activa ya está copiada con modo 600 al worktree de candidata
   y permanece ignorada por Git. La configuración SSH predeterminada menciona una
   `id_rsa` inexistente. Para el despliegue usar un ssh-agent temporal con la clave
   ya existente `~/.ssh/id_ed25519`; los accesos directos funcionan con
   `ssh -i ~/.ssh/id_ed25519 -o IdentitiesOnly=yes -o BatchMode=yes nex@100.107.20.66`.
   Hay un wrapper privado preparado, **todavía no ejecutado**, que comprueba SHA,
   árbol de main y limpieza y crea/cierra su propio ssh-agent:
   `~/.ros/lekiwi/qualification/20261116-bootstrap/deploy-autonomy.sh`.
   Revisarlo y ejecutarlo con bash cuando se cumplan los pasos anteriores. Registrar
   el log; no dejar un ssh-agent o un despliegue sin seguimiento.

5. Verificar selección y marcadores en ambos equipos, armado y postura frescos,
   readiness y parámetros nativos de sincronización. Repetir la muestra estacionaria
   de cadencia con la nueva release; el A/B previo no sustituye esta observación.

6. Leer `rosbridge.capabilities`, verificar recursos libres y validar una nueva
   tarea Explore acotada. Usar identidad estable nueva, por ejemplo
   `qualification-explore-autonomy-a27b657-20261116-1`, duración 20–30 s y radio
   1–2 m. La tarea debe poder arrancar sin cobertura de búsqueda; distinguir
   admisión, dispatch, pausa de seguridad y movimiento real. Observar durante un
   plazo de pared limitado: una pausa conserva el presupuesto de tiempo activo.
   Cancelar si corresponde y consultar hasta estado terminal, parada confirmada
   y recursos libres. Un resultado desconocido requiere reconciliar, no otra tarea.
   Conservar journal y odometría; no afirmar exploración física si no hubo movimiento.

7. Actualizar `ROLLOUT.md` con el resultado real, retirar `WIP.md` al completar la
   entrega y subir el checkpoint. Al terminar, informar cambios sin commit, commits
   sin push, tareas/procesos propios activos y el siguiente paso exacto si queda algo.

## Worktrees y evidencia para retomar

Todos los worktrees de esta tarea estaban limpios y sincronizados con su upstream
al consultar; este handoff se guarda y sube en la rama operativa.

| Worktree bajo `.claude/worktrees/` | Rama | Head antes del handoff |
| --- | --- | --- |
| `exploration-bootstrap` | `fix/exploration-bootstrap` | `830395d` |
| `search-geometry-latency` | `fix/search-geometry-latency` | `bae6578` |
| `rviz-image-shutdown` | `fix/rviz-image-shutdown` | `1d369a1` |
| `robot-autonomy-release` | `release/robot-autonomy-20261116` | `a27b657` |
| `agent-search-rollout` | `ops/agent-search-rollout` | `7415414` |

Hay otros worktrees ajenos a la tarea. No limpiarlos ni subir sus commits. Se había
observado `fix/exploration-footprint-planner` adelantada en un commit; no se modificó.
Para retomar, inspeccionar `git worktree list`, `git branch -vv`, `git stash list`
y el estado de cada worktree relevante.

Directorio privado principal: `~/.ros/lekiwi/qualification/20261116-bootstrap/`.
Contiene cuerpos/revisiones de PR, recibos del brazo, pruebas Explore, muestras
de sensores, A/B, trazas del debugger, regresiones, fallos de CI y ensayos RViz.
Los datos de la primera fase y el backup consistente del mapa están en
`~/.ros/lekiwi/qualification/20261116-agent-search/`. No imprimir cores ni archivos
`.crash`; hay trazas de texto y un core desempaquetado propio además del original.

Los logs ROS de nodos están directamente en `~/.ros/log/<nodo>_<pid>_<timestamp>.log`;
`~/.ros/log/latest` contiene solo el `launch.log` del lanzamiento más reciente.

Persisten límites de aceptación: frescura continua de geometría de búsqueda,
calibración física y prueba de puntos de vista con evidencia útil. Los objetivos
de alto nivel de tipo find móvil requieren mapa y vistas aprobados; no inventarlos
ni confundir esa aprobación con habilitar Explore o búsqueda nativa. Las habilidades
entrenadas del brazo tampoco están calificadas por estos cambios.
