# x-IMU3 — Orientación, Aceleración y Posición en Tiempo Real

Conjunto de scripts en Python para conectarse a una **IMU x-IMU3** (x-io Technologies) por TCP, leer sus datos en tiempo real (orientación, aceleración, giroscopio, magnetómetro) y estimar velocidad/posición mediante integración inercial (*dead reckoning*), con visualización en vivo usando Matplotlib.

## ¿Qué hace este proyecto?

1. Descubre y se conecta automáticamente a un dispositivo x-IMU3 en la red (o usa una IP/puerto por defecto si no encuentra ninguno).
2. Recibe en tiempo real los mensajes del sensor: cuaternión de orientación, aceleración lineal, giroscopio y magnetómetro.
3. Convierte el cuaternión a matriz de rotación para pasar la aceleración del marco del sensor al marco del mundo.
4. Resta la gravedad y aplica un filtro de suavizado + detección de reposo (*stationary detection*) para reducir el arrastre (*drift*) típico de la doble integración.
5. Integra la aceleración lineal para obtener **velocidad**, y la velocidad para obtener **posición** estimada.
6. Grafica en tiempo real: orientación 3D (ejes del sensor), aceleración, velocidad y trayectoria de posición.

## Estructura del repositorio

```
ximu3-main/
├── x-imu.py             # Script principal: orientación + velocidad + posición en tiempo real (gráficas 3D)
├── X-imu3_poss.py        # Variante con filtrado más elaborado (umbral de reposo, cuaternión de referencia)
├── testing.py            # Script de prueba mínimo para lectura de aceleración/giroscopio/magnetómetro
├── connection.py          # Ejemplo base de conexión x-IMU3 (impresión de todos los tipos de mensaje del SDK)
├── tcp_connection.py       # Punto de entrada para conectar por TCP usando connection.py
├── helpers.py             # Utilidades varias (prompt de sí/no por consola)
└── Data Logger Example/    # Datos de ejemplo capturados de un dispositivo real (CSV de batería, cuaternión,
                             # magnetómetro, temperatura, acelerómetro de alto rango, datos inerciales)
```

## Requisitos

- Python 3.9+
- SDK oficial de x-IMU3 para Python: [`ximu3`](https://pypi.org/project/ximu3/)
- Dependencias adicionales:
  ```bash
  pip install ximu3 numpy matplotlib keyboard
  ```
- Un dispositivo x-IMU3 accesible por red (TCP) o USB, según el script.

## Uso

Ejecutar el script principal de seguimiento en tiempo real:

```bash
python x-imu.py
```

- El script busca automáticamente dispositivos x-IMU3 en la red durante unos segundos.
- Si no encuentra ninguno, intenta conectarse a `192.168.1.1:7000` por defecto.
- Al conectar, espera 3 segundos y luego abre las ventanas de visualización (velocidad y posición en tiempo real).
- Presiona **`ESC`** para detener la captura y cerrar la conexión de forma segura.

Para probar la conexión y el flujo de mensajes crudos del SDK (sin estimación de posición), usar `tcp_connection.py` / `connection.py`, que imprimen en consola todos los tipos de datos que reporta el sensor (inercial, magnetómetro, batería, RSSI, temperatura, etc.).

## Notas técnicas

- **Estimación de posición por dead reckoning**: al integrar aceleración dos veces sin una referencia externa (GPS, cámara, etc.), el error se acumula rápidamente. Este proyecto mitiga el drift con:
  - Detección de reposo (*stationary threshold*) para anular la velocidad cuando el sensor está quieto.
  - Atenuación exponencial de velocidad y posición (`*= 0.98` / `*= 0.99`) para evitar que el error crezca sin control.
  - Esto la hace útil como **demo/prototipo de tracking a corto plazo**, no como sistema de posicionamiento preciso a largo plazo.
- `X-imu3_poss.py` es una segunda iteración del mismo enfoque, con un cuaternión de referencia inicial (`adjust_to_reference`) para expresar la orientación relativa al arranque, y un umbral de reposo (`REST_THRESHOLD`) más explícito basado en tiempo acumulado en reposo.
- `Data Logger Example/` contiene una captura real (formato CSV) que sirve como referencia de las unidades y el formato de cada tipo de dato (útil para probar análisis offline sin tener el sensor conectado).

## Pendientes / mejoras sugeridas

- `X-imu3_poss.py` tiene actualmente una línea de texto (`You said:`) pegada accidentalmente al inicio del archivo — debe eliminarse para que el script sea válido.
- Unificar `x-imu.py` y `X-imu3_poss.py` en una sola implementación, ya que comparten la mayor parte de la lógica (quedan como versiones duplicadas/alternativas del mismo pipeline).
- Sustituir la dependencia de `keyboard.is_pressed("esc")` (bloqueante y con permisos especiales en algunos sistemas) por un manejo de eventos de matplotlib o `KeyboardInterrupt`.
- Añadir fusión con otra fuente de referencia absoluta (por ejemplo, RTK-GPS o marcadores visuales) si se busca precisión de posición a largo plazo.
