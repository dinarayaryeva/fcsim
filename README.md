# fcsim — заглушка полётного контроллера с инъекцией отказов

Имитатор Pixhawk/PX4 для отладки и автотестов бортового ПО без железа. Говорит на **настоящем MAVLink 2** (`pymavlink`), поэтому бортовое ПО подключается к нему так же, как к реальному ПК.

[Архитектура](docs/architecture.md) · [Перечень отказов](docs/faults.md)

> **Почему Python 3.11+.** Это инструмент тестирования, а не контур реального времени. `pymavlink` и pytest экономят код, а 1 мс достигается абсолютными дедлайнами и активным ожиданием.

## Быстрый старт

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt        # pymavlink, pytest
python -m pytest -q                    # ~10 с (3 теста идут в реальном времени)
```

| Режим | Команда |
|---|---|
| Виртуальное время (быстро, воспроизводимо) | `python -m fcsim run scenarios/default.yaml` |
| Демо отказов | `python -m fcsim run scenarios/faults_demo.yaml` |
| Реальное время + UDP (для бортового ПО) | `python -m fcsim run scenarios/default.yaml --realtime` |

## Что реализовано

| Требование | Где |
|---|---|
| Телеметрия по заданным частотам | `sim.py` |
| Приём команд и смена режима | `sim.py` |
| Режимы `NOT_READY → READY → MANUAL ⇄ EXTERNAL ⇄ HOLD`; **более 500 мс без команд в EXTERNAL → HOLD** + STATUSTEXT с причиной | `sim.py` |
| Динамика: интегратор 1-го порядка по скорости, P-регулятор по положению | `dynamics.py` |
| 6 настраиваемых отказов | `faults.py`, `docs/faults.md` |
| Конфигурация отдельно от кода | `scenarios/*.yaml`, `config.py` |
| Тесты: частоты, таймаут, каждый отказ, воспроизводимость, джиттер, UDP end-to-end | `tests/` |

## Формат сценария (YAML)

```yaml
run:
  seed: <seed>
  duration_s: <duration>

script:
  - t_s: <time>
    do: "<action>"
    <param>: <value>

fault:
  - type: "<fault_type>"
    start_s: <time>
    duration_s: <duration>
    <param>: <value>
```
Пример:\
 boot -> ready -> arm -> external mode -> flight -> setpoint stream loss -> hold

```yaml
run:
  seed: 42
  duration_s: 40

script:
  - t_s: 5.0
    do: "arm"

  # Удержание скорости на 20 Гц с 5.5 по 25 сек
  - t_s: 5.5
    do: "setpoint_stream"
    kind: "vel"
    vx: 5.0
    vz: -1.0
    rate_hz: 20
    until_s: 25.0
    
  - t_s: 6.0
    do: "mode"
    mode: "EXTERNAL"

fault:
  # Ошибка перезагрузки ПК
  - type: "fc_reboot"
    start_s: 20
    duration_s: 6
```

## Воспроизводимость

- **Виртуальное время:** прогон определяется конфигурацией и `seed`, поток кадров **побайтно одинаков** (`test_repro.py`).
- Шум датчиков и случайные отказы берут числа из изолированных ГСЧ, поэтому добавление отказа не сдвигает остальные.
- Метки времени берутся от часов симулятора.

##  Джиттер выдачи (цель ≤ 1 мс)

**Метрика:** отклонение момента отправки от идеального расписания `phase + k·period` (всегда ≥ 0).

Тест проверяет **p99 ≤ 1 мс** (по ~460 кадрам) и **медиану ≤ 0,5 мс**.

 Жёстких гарантий Python не даёт. Для них нужны C++, RT-ядро и изоляция ядра, а это сознательный компромисс. Потребителю телеметрии это безразлично, если он использует `time_boot_ms`, а не время прихода.

## Допущения (исходные данные неполны)

- «GLOBAL_POSITION» = `GLOBAL_POSITION_INT`; система координат команд — LOCAL_NED, курс игнорируется.
- Переход в EXTERNAL возможен только при живом потоке команд (как в PX4); из HOLD в EXTERNAL — только командой.
- **Не реализовано:** `TIMESYNC`, `GPS_RAW_INT`, `SYS_STATUS`, failsafe по батарее (только предупреждение), падение при перезагрузке ПК.
