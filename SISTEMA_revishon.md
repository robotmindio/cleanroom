   Prioridad    Hallazgo                            Evidencia / impacto
  ━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Crítica      Alimentación/temperatura            Kernel registra undervoltage repetido. CPU a 84.2 °C; vcgencmd indica limitación de frecuencia y throttling activos, además de
                                                    eventos históricos de subtensión. Riesgo de caídas, lecturas erráticas, corrupción de SD/USB y comportamiento inseguro de motores.
  ───────────  ──────────────────────────────────  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   Alta         Robot armado                        safety/state = ARMED; los servos están con torque habilitado. No manipular físicamente el robot así.
  ───────────  ──────────────────────────────────  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   Alta         Puertos de control expuestos        ZMQ de movimiento/torque (5555, 5557) y rosbridge (9090) escuchan en 0.0.0.0, sin autenticación de aplicación. Cualquier equipo con
                                                    acceso a esa red podría intentar publicar comandos. No pude confirmar el firewall sin privilegios.
  ───────────  ──────────────────────────────────  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   Alta         Recursos al límite                  Carga ~18, swap en uso y solo ~1.2 GB de RAM disponible. RViz está ejecutándose en la Raspberry y consume bastante; sumado a Nav2,
                                                    RTAB-Map, cámaras y el host, agrava el throttling.
  ───────────  ──────────────────────────────────  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   Alta         Base RTAB-Map continúa creciendo    La DB activa ya está en 459 MB tras unas horas. La poda/rotación al arranque funciona, pero no limita el crecimiento durante una
                                                    ejecución larga; puede volver a llenar disco si el stack queda activo muchos días.
  ───────────  ──────────────────────────────────  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   Media        Servicio systemd en fallo           lekiwi-host.service falló porque intentó arrancar mientras el host manual ya poseía el bus serie. El proceso manual funciona, pero
                                                    systemd no lo está supervisando ahora; hay dos vías de arranque en conflicto.
  ───────────  ──────────────────────────────────  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   Media        Ruido de logs                       host.log contiene 63.342 avisos “No command available” y ya ocupa 2.1 MB. Es spam de log evitable; a largo plazo añade I/O y ocupa
                                                    almacenamiento.
  ───────────  ──────────────────────────────────  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   Media        Estado del repositorio              Hay numerosas modificaciones y archivos nuevos sin consolidar. git diff --check está limpio y las pruebas locales pasaron, pero
                                                    conviene hacer una revisión/commit coherente antes de desplegar otra vez.

  Estado positivo comprobado:

  - LD06 está implementado y funcionando: nodo /ld06_lidar, publicación de /scan, TF base_footprint → laser, y consumidores activos en RTAB-Map/Nav2/collision monitor.
  - Nav2 (controller, planner, bt_navigator, collision_monitor) está active.
  - RTAB-Map publica /map; odometría, cámaras y driver están presentes.
  - La poda de RTAB-Map quedó en scripts/rtabmap-db-maintenance.py y se integra en los arranques.
  - El disarm corta torque mediante el host de seguridad, pero el reinicio posterior dejó el estado actual en ARMED.

  Orden recomendado: 1) corregir fuente/cableado de 5 V y refrigeración, 2) desarmar antes de tocar el robot, 3) ejecutar RViz desde una estación remota, 4) restringir puertos por
  firewall/red, 5) unificar el arranque en systemd y 6) añadir un límite de crecimiento de RTAB-Map en ejecución.
