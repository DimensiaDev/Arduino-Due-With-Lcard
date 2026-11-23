#!/usr/bin/env python3
"""Диагностика LSL на компьютере с устройством.

Запускать на том же компьютере, где работает программа, пока включено
устройство (Neuron Spectrum) и идёт checkup:

    python lsl_diag.py            # ждём потоки до 10 с
    python lsl_diag.py 30         # ждём до 30 с

Скрипт показывает:
  * какие LSL-потоки видны этой машине (в т.ч. поток устройства);
  * IP-адреса компьютера — их нужно сравнить с подсетью устройства;
  * краткий вердикт: программа или сеть/брандмауэр.

Если поток устройства здесь НЕ виден — дело не в программе: проверьте
кабель, подсеть и брандмауэр Windows (UDP 11578 и 16572-16579).
"""

import socket
import sys
import time

try:
    from pylsl import resolve_streams
except Exception as e:  # pragma: no cover
    print(f"pylsl недоступен: {e}")
    print("Установите: pip install pylsl")
    sys.exit(1)

WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
LSL_MULTICAST = "239.255.255.250"
LSL_SDP_PORT = 11578


def local_ipv4():
    out = set()
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            out.add(info[4][0])
    except Exception:
        pass
    out.add("127.0.0.1")
    return sorted(out)


def main():
    print("=" * 68)
    print("Диагностика LSL")
    print("=" * 68)

    addrs = local_ipv4()
    print(f"IP этого компьютера: {', '.join(addrs)}")
    print(f"Группа multicast LSL: {LSL_MULTICAST}:{LSL_SDP_PORT}")
    print(f"Ждём потоки до {WAIT:.0f} с (устройство должно быть включено "
          f"и с запущенным checkup)...")
    print("-" * 68)

    t0 = time.monotonic()
    streams = resolve_streams(wait_time=WAIT)
    waited = time.monotonic() - t0

    if not streams:
        print("ПОТОКОВ НЕТ.")
        print()
        print("Что проверить:")
        print("  1. Устройство включено и на нём идёт checkup (публикация потока).")
        print("  2. Кабель подключён, IP устройства в одной подсети с компьютером.")
        print("  3. Брандмауэр Windows: разрешить Python для частной сети")
        print("     (UDP 11578 — SDP, и 16572-16579 — данные потоков).")
        print("  4. Отключите VPN/прокси: они перехватывают multicast и")
        print("     удалённые потоки становятся не видны (локальные — видны).")
        print("  5. Проверьте тем же способом в LSL Viewer: если там потока")
        print("     тоже нет — проблема точно не в программе.")
        return 1

    print(f"НАЙДЕНО ПОТОКОВ: {len(streams)} (ожидание {waited:.1f} с)")
    print("-" * 68)
    device_seen = False
    for s in streams:
        try:
            name = s.name()
            type_ = s.type()
            uid = s.uid()
            src = s.source_id()
            host = s.hostname()
            ch = s.channel_count()
            fs = s.nominal_srate()
        except Exception as e:
            print(f"  <не удалось прочитать параметры: {e}>")
            continue
        marker = ""
        if "neuron" in (name or "").lower() or (type_ or "").upper() == "EEG":
            device_seen = True
            marker = "   <-- похоже на устройство"
        print(f"  имя:      {name}")
        print(f"  тип:      {type_}   каналов: {ch}   частота: {fs} Гц")
        print(f"  хост:     {host}")
        print(f"  source_id:{src}")
        print(f"  uid:      {uid}{marker}")
        print("-" * 68)

    if device_seen:
        print("Поток устройства виден — с программой всё в порядке.")
        print("Если программа его не показывает: нажмите «Обновить» в разделе")
        print("«Сервис» и выберите поток в списке.")
    else:
        print("Устройство в списке не похоже на EEG-поток — возможно, пришёл")
        print("другой поток (например, локальный генератор). Сравните имена.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
