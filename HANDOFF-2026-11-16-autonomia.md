# Handoff de autonomía del robot y Fiber

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
