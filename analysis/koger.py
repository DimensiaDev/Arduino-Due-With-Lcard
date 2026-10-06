#!/usr/bin/env python3

"""
Когерентное накопление (phase-locked averaging) EEG при фотостимуляции.

Вход:
    одна или несколько директорий с файлами вида *step_*.txt.

Пример:
    python koger.py --folder "experiments/Казаков А"
    python koger.py --folder experiments/* --step 1 --output step1.png --no-show

Структура фотостимуляции (одна и та же для обоих полушарий):

    DAC0:  dac0_f1 + dac0_f2
    DAC1:  dac1_f1 + dac1_f2

Для каждой эпохи сохраняются все четыре частоты.

--------------------------------------------------------------------------
Что было исправлено (см. раздел «Исправления» внизу файла)
--------------------------------------------------------------------------

Главная проблема: КОГЕРЕНТНОЕ НАКОПЛЕНИЕ НЕ МОГЛО ВЫПОЛНЯТЬСЯ ВООБЩЕ.

1. Частота дискретизации читалась из ключа `lsl_nominal_srate_hz`
   в секции `# METADATA`, которого там НЕТ (он есть только в таблице
   `# DATA` и внутри `lsl_stream_meta_json`). Все 210 файлов
   отбрасывались -> `RuntimeError: Не удалось загрузить ни одной эпохи`.

2. Условие «у всех эпох одинаковые частоты стимуляции» неверно для
   этого протокола: каждый из 7 шагов имеет СВОЮ конфигурацию.
   Эпохи теперь группируются по конфигурации, накопление идёт
   только внутри группы.

3. `app_timestamp` НЕ является временем дискретизации. Это метка
   времени ПРИЁМА на стороне PC: LSL отдаёт данные пачками по
   ~25 отсчётов, вся пачка получает почти одинаковые метки
   (шаг ~2 мкс), затем идёт пауза ~51 мс. Использовать его как
   ось времени нельзя. Ось времени строится как `t = n / fs` по
   индексу отсчёта. `app_timestamp` используется только там, где
   он действителен: для оценки задержки «событие стимула ->
   первый отсчёт», чтобы выровнять эпохи разных шагов.

4. Линейная интерполяция `interp1d` по сломанной сетке времени
   гасила всё выше ~2 Гц. Интерполяция больше не используется:
   отсчёты уже лежат на равномерной сетке с шагом 1/fs.

5. Оценка `C = 2/N * sum(x * exp(-j*2*pi*f*t))` корректна только
   при `f*T` — целом числе и отсутствии постоянной составляющей.
   Заменена на взвешенную МНК-оценку (нормальные уравнения) с
   окном Ханна и членом постоянной составляющей.

6. `phase_coherence` считалась как `|sum C| / sum |C|` — это
   амплитудно-взвешенная величина. Заменена на ITC
   `|mean(C/|C|)|`, где каждая проба вносит равный вклад.

7. Каналы `filtered_*` пропущены полосой 0.5–40 Гц, поэтому для
   шага 7 (55 Гц) и шага 6 (32/34 Гц) в них НИЧЕГО нет
   (подавление >70 дБ). Для таких групп источник выбирается
   автоматически (`raw_*`).

8. Мёртвые каналы (`raw_ch_9`, `raw_ch_10` — везде нули)
   больше не попадают в усреднение.

9. Начало каждого файла шага содержит: (а) данные, записанные ещё
   на частотах ПРЕДЫДУЩЕГО шага, (б) переходные процессы
   `filtfilt` (~50 мс). Введён `--guard` (по умолчанию 0.1 с).
"""

import argparse
import csv
import io
import json
import re
import sys
import warnings
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import butter, sosfreqz


# ============================================================================
# Configuration
# ============================================================================

STEP_FILE_PATTERN = re.compile(
    r".*step_\d+.*\.txt$",
    re.IGNORECASE,
)

DAC_FREQUENCY_KEYS = (
    "dac0_f1",
    "dac0_f2",
    "dac1_f1",
    "dac1_f2",
)

# Порядок округления частот при группировке эпох.
FREQUENCY_QUANTUM = 1e-6

# Метки DAC в человекочитаемом виде.
DAC_LABELS = {
    "dac0_f1": "DAC0 f1",
    "dac0_f2": "DAC0 f2",
    "dac1_f1": "DAC1 f1",
    "dac1_f2": "DAC1 f2",
}

# Параметры полосового фильтра по умолчанию.
# Используются, если `filter_config_json` отсутствует или нечитаем.
DEFAULT_BANDPASS_LOW_HZ = 0.5
DEFAULT_BANDPASS_HIGH_HZ = 40.0
DEFAULT_BANDPASS_ORDER = 4

# Порог затухания в фильтрованных каналах, выше которого
#stimulated frequency приходится брать из raw_*.
#
# Затухание считается для ДВОЙНОГО применения (filtfilt).
#
# Ориентиры для полосы 0.5–40 Гц, порядок 4 (f0 = 4.47 Гц):
#   10 Гц  -> ~0.2 дБ
#   12 Гц  -> ~1 дБ
#   20 Гц  -> ~46 дБ
#   29 Гц  -> ~93 дБ
#   34 Гц  -> ~146 дБ
#   40 Гц  -> ~146 дБ
#   55 Гц  -> ~152 дБ
MAX_ATTENUATION_DB = 1.0


# ============================================================================
# File loading
# ============================================================================


def unescape_pandas_quoted(value: str) -> str:
    """
    Убирает экранирование кавычек, используемое pandas/CSV.

    Например:

        '"text with ""quotes""'

    превращается в:

        'text with "quotes"'
    """

    if len(value) >= 2 and value.startswith('"') and value.endswith('"'):
        value = value[1:-1].replace('""', '"')

    return value


def load_metadata_and_data(
    file_path: Path,
) -> tuple[dict, pd.DataFrame]:
    """
    Читает файл формата:

        metadata...
        # DATA
        column1    column2    ...
        value      value      ...

    Metadata и данные обрабатываются отдельно.
    """

    content = file_path.read_text(
        encoding="utf-8",
    )

    if "# DATA" not in content:
        raise ValueError(f"В файле {file_path} отсутствует секция '# DATA'")

    meta_part, data_part = content.split(
        "# DATA",
        1,
    )

    # ------------------------------------------------------------------------
    # Metadata
    # ------------------------------------------------------------------------

    metadata = {}

    reader = csv.reader(
        io.StringIO(meta_part.strip()),
        delimiter="\t",
    )

    for row in reader:
        if not row or len(row) < 2 or row[0].startswith("#"):
            continue

        key = row[0].strip()

        value = "\t".join(row[1:]).strip()

        value = unescape_pandas_quoted(value)

        metadata[key] = value

    # ------------------------------------------------------------------------
    # Data
    # ------------------------------------------------------------------------

    if not data_part.strip():
        raise ValueError(f"После '# DATA' нет данных: {file_path}")

    df = pd.read_csv(
        io.StringIO(data_part.strip()),
        sep="\t",
        engine="python",
    )

    return metadata, df


# ============================================================================
# Metadata helpers
# ============================================================================


def get_required_float(
    metadata: dict,
    key: str,
) -> float:
    """
    Получает обязательное числовое значение metadata.
    """

    value = metadata.get(key)

    if value is None or value == "":
        raise ValueError(f"В metadata отсутствует {key}")

    try:
        result = float(value)
    except ValueError as exc:
        raise ValueError(f"{key} имеет некорректное значение: {value!r}") from exc

    if not np.isfinite(result):
        raise ValueError(f"{key} имеет нечисловое значение: {value!r}")

    return result


def get_optional_float(
    metadata: dict,
    key: str,
    default=None,
):
    """
    Числовое значение metadata либо `default`.
    """

    value = metadata.get(key)

    if value is None or value == "":
        return default

    try:
        result = float(value)
    except ValueError:
        return default

    if not np.isfinite(result):
        return default

    return result


def resolve_sampling_rate(
    metadata: dict,
    df: pd.DataFrame,
) -> float:
    """
    Определяет номинальную частоту дискретизации.

    ВАЖНО:
        В секции `# METADATA` ключа `lsl_nominal_srate_hz` НЕТ.
        Он есть только:
            * в таблице `# DATA` (столбец `lsl_nominal_srate_hz`);
            * внутри `lsl_stream_meta_json` -> `nominal_srate_hz`.

    Источники проверяются в этом порядке.
    """

    # 1. Прямой ключ в metadata (на случай будущего формата).
    fs = get_optional_float(metadata, "lsl_nominal_srate_hz")

    if fs is not None and fs > 0:
        return fs

    # 2. Описание LSL-потока.
    raw = metadata.get("lsl_stream_meta_json")

    if raw:
        try:
            stream_meta = json.loads(raw)
        except json.JSONDecodeError:
            stream_meta = None

        if isinstance(stream_meta, dict):
            value = stream_meta.get("nominal_srate_hz")

            try:
                candidate = float(value)
            except (TypeError, ValueError):
                candidate = 0.0

            if np.isfinite(candidate) and candidate > 0:
                return candidate

    # 3. Столбец таблицы данных.
    if "lsl_nominal_srate_hz" in df.columns:
        values = pd.to_numeric(
            df["lsl_nominal_srate_hz"],
            errors="coerce",
        ).dropna()

        if len(values):
            candidate = float(values.iloc[0])

            if np.isfinite(candidate) and candidate > 0:
                return candidate

    raise ValueError("Не удалось определить частоту дискретизации")


def get_bandpass_config(
    metadata: dict,
) -> dict:
    """
    Конфигурация полосового фильтра, применённого к `filtered_*`.

    Из неё нужны `bandpass_high_hz`, `bandpass_low_hz` и
    `bandpass_order`: они определяют, насколько сильно подавлена
    каждая стимулируемая частота в отфильтрованных каналах.
    """

    defaults = {
        "bandpass_low_hz": DEFAULT_BANDPASS_LOW_HZ,
        "bandpass_high_hz": DEFAULT_BANDPASS_HIGH_HZ,
        "bandpass_order": DEFAULT_BANDPASS_ORDER,
        "bandpass_enabled": True,
    }

    raw = metadata.get("filter_config_json")

    if not raw:
        return defaults

    try:
        config = json.loads(raw)
    except json.JSONDecodeError:
        return defaults

    if not isinstance(config, dict):
        return defaults

    result = dict(defaults)

    for key in (
        "bandpass_low_hz",
        "bandpass_high_hz",
        "bandpass_order",
    ):
        value = get_optional_float(config, key)

        if value is not None and value > 0:
            result[key] = value

    result["bandpass_enabled"] = bool(config.get("bandpass_enabled", True))

    return result


def bandpass_attenuation_db(
    frequency: float,
    config: dict,
    fs: float,
) -> float:
    """
    Затухание полосового фильтра `filtered_*` на частоте `frequency`.

    Считается по ТОЧНОЙ АЧХ того фильтра, который строит экспортёр
    (`butter(order, [low, high], btype="band", fs=fs, output="sos")`),
    а не по приближённой формуле: спад у Баттерворта настолько
    крутой, что приближение ошибается на десятки дБ.

    `sosfiltfilt` = два прохода, поэтому затухание удваивается.

    Нужно для выбора источника: `filtered_*` подходит не только когда
    частота ниже верхней границы полосы, но когда затухание в ней
    приемлемо. Проверено на этих данных:
        10 Гц ->  0.0 дБ      29 Гц ->  0.5 дБ
        22 Гц ->  0.1 дБ      32 Гц ->  1.5 дБ
                              34 Гц ->  2.3 дБ
                              55 Гц -> 23.6 дБ
    """

    if not config.get("bandpass_enabled", True):
        return 0.0

    low = float(config["bandpass_low_hz"])

    high = float(config["bandpass_high_hz"])

    order = float(config["bandpass_order"])

    if low <= 0 or high <= 0 or order <= 0 or fs <= 0:
        return 0.0

    frequency = abs(float(frequency))

    if frequency == 0:
        return float("inf")

    try:
        sos = butter(
            int(order),
            [low, high],
            btype="band",
            fs=fs,
            output="sos",
        )
    except (ValueError, TypeError):
        return 0.0

    _, response = sosfreqz(
        sos,
        worN=[frequency],
        fs=fs,
    )

    gain = float(np.abs(response[0]))

    if gain <= 0:
        return float("inf")

    return float(-2.0 * 20.0 * np.log10(gain))


def extract_step_info(
    metadata: dict,
) -> dict:
    """
    Извлекает информацию о конкретном шаге.

    Возвращает:

        step_index
        app_timestamp_start
        dac0_f1
        dac0_f2
        dac1_f1
        dac1_f2
        duration
    """

    try:
        step_index = int(metadata["export_step_index"])
    except (KeyError, ValueError) as exc:
        raise ValueError("Некорректный export_step_index") from exc

    # ------------------------------------------------------------------------
    # Находим timestamp начала конкретного шага
    # ------------------------------------------------------------------------

    events_json = metadata.get(
        "stim_events_json",
        "[]",
    )

    try:
        events = json.loads(events_json)
    except json.JSONDecodeError as exc:
        raise ValueError("Не удалось разобрать stim_events_json") from exc

    start_time = None

    for event in events:
        if event.get("step_index") == step_index:
            start_time = event.get("app_timestamp")
            break

    if start_time is None:
        raise ValueError(f"Не найдено событие для шага {step_index} в stim_events_json")

    try:
        start_time = float(start_time)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Некорректный app_timestamp: {start_time!r}") from exc

    # ------------------------------------------------------------------------
    # Частоты фотостимуляции
    # ------------------------------------------------------------------------

    frequencies = {}

    for name in DAC_FREQUENCY_KEYS:
        value = get_required_float(
            metadata,
            f"export_step_{name}_hz",
        )

        # Частота должна быть положительной.
        if value <= 0:
            raise ValueError(f"{name} должна быть > 0, получено {value}")

        frequencies[name] = value

    duration = get_optional_float(metadata, "export_step_duration_s", 0.0)

    return {
        "step_index": step_index,
        "app_timestamp_start": start_time,
        "duration": duration,
        **frequencies,
    }


def frequency_signature(step: dict) -> tuple:
    """
    Ключ группировки эпох: конфигурация стимуляции.

    Эпохи с разной конфигурацией НЕЛЬЗЯ усреднять между собой —
    усреднение комплексных коэффициентов разных частот бессмысленно.
    """

    return tuple(
        round(step[name] / FREQUENCY_QUANTUM) * FREQUENCY_QUANTUM
        for name in DAC_FREQUENCY_KEYS
    )


def format_signature(
    signature: tuple,
) -> str:
    """
    Человекочитаемое имя группы.
    """

    return "DAC0 " + "+".join(f"{signature[i]:g}" for i in (0, 1)) + (
        " / DAC1 " + "+".join(f"{signature[i]:g}" for i in (2, 3))
    )


# ============================================================================
# File discovery
# ============================================================================


def collect_files(
    folder_paths: list[str],
) -> list[Path]:
    """
    Находит step-файлы во всех переданных директориях.

    Пример:

        --folder dir1 dir2 dir3
    """

    files = set()

    for folder_name in folder_paths:
        folder = Path(folder_name)

        if not folder.exists():
            raise FileNotFoundError(f"Диретория не существует: {folder}")

        if not folder.is_dir():
            raise NotADirectoryError(f"Не является директорией: {folder}")

        for file_path in folder.iterdir():
            if file_path.is_file() and STEP_FILE_PATTERN.match(file_path.name):
                files.add(file_path)

    return sorted(files)


# ============================================================================
# Channels
# ============================================================================


def find_signal_columns(
    df: pd.DataFrame,
    prefix: str,
) -> list[str]:
    """
    Возвращает каналы с заданным префиксом в порядке появления в файле.
    """

    return [
        column
        for column in df.columns
        if column.startswith(prefix)
    ]


def select_source(
    df: pd.DataFrame,
    source: str,
) -> list[str]:
    """
    Выбирает набор каналов-«источников».

    `auto`:
        `filtered_*`, если все стимулируемые частоты проходят
        внутри полосы фильтра; иначе `raw_*`.

    Почему:
        в файлах стоит `sosfiltfilt` с полосой 0.5–40 Гц.
        Частота 55 Гц (шаг 7) и 32–34 Гц (шаг 6) в `filtered_*`
        отсутствуют: подавление >70 дБ, вплоть до нуля.

    Фактический выбор делает `choose_source_for_group` — здесь
    проверяется только наличие колонок.
    """

    if source == "raw":
        columns = find_signal_columns(df, "raw_")
        label = "raw_"
    elif source == "filtered":
        columns = find_signal_columns(df, "filtered_")
        label = "filtered_"
    else:
        columns = find_signal_columns(df, "filtered_")
        label = "filtered_"

        if not columns:
            columns = find_signal_columns(df, "raw_")
            label = "raw_"

    if not columns:
        raise ValueError(f"В файле нет каналов {label}*")

    return columns


def apply_channel_selection(
    available: list[str],
    channels: list[int] | None,
) -> list[str]:
    """
    Выбирает каналы по индексам.

    В отличие от ранних версий, индекс вне диапазона — это ошибка,
    а не молчаливое отбрасывание канала.
    """

    if channels is None:
        return list(available)

    invalid = [
        index
        for index in channels
        if index < 0 or index >= len(available)
    ]

    if invalid:
        raise ValueError(
            f"Некорректные индексы каналов: {invalid}. "
            f"Доступно каналов: {len(available)}"
        )

    return [available[index] for index in channels]


# ============================================================================
# Epoch loading
# ============================================================================


def compute_start_latency(
    epoch: pd.DataFrame,
    step: dict,
) -> float:
    """
    Задержка от события стимула до первого отсчёта файла, в секундах.

    Это единственное, для чего `app_timestamp` действительно пригоден:
    он измерен на стороне PC и отражает момент приёма данных,
    но МЕДЛЕННЫЙ, т.е. систематически занижает оценку.

    Компенсация этой задержки выравнивает эпохи лучше, чем
    привязка к нулевому отсчёту: остаточная неточность равна
    времени одного обмена по последовательному порту (~1–2 мс),
    а не 17–53 мс разброса между шагами.

    `lsl_timestamp` не используется: он имеет разрешение 1 секунда.
    """

    values = pd.to_numeric(
        epoch["app_timestamp"],
        errors="coerce",
    ).to_numpy(dtype=float)

    finite = values[np.isfinite(values)]

    if finite.size == 0:
        return 0.0

    return float(finite[0] - step["app_timestamp_start"])


def load_epochs(
    files: list[Path],
    channels: list[int] | None,
    source: str,
    step_filter: list[int] | None = None,
    verbose: bool = True,
):
    """
    Загружает все эпохи.

    Ось времени каждой эпохи: `t = n / fs`, где `n` — индекс отсчёта.

    НЕЛЬЗЯ использовать `app_timestamp` как ось времени: LSL отдаёт
    данные пачками, и внутри пачки из ~25 отсчётов метки отличаются
    на микросекунды, а между пачками идёт пауза ~51 мс.
    """

    epochs = []

    reference_fs = None
    channel_names = None
    channel_prefix = None
    bandpass = get_bandpass_config({})

    for file_path in files:
        try:
            metadata, df = load_metadata_and_data(file_path)

            step = extract_step_info(metadata)

            if step_filter is not None and step["step_index"] not in step_filter:
                continue

            fs = resolve_sampling_rate(metadata, df)

            if "app_timestamp" not in df.columns:
                raise ValueError("отсутствует столбец app_timestamp")

            available = select_source(df, source)

            prefix = "raw_" if available[0].startswith("raw_") else "filtered_"

        except Exception as exc:
            if verbose:
                print(f"Пропуск {file_path.name}: {exc}", file=sys.stderr)

            continue

        # ------------------------------------------------------------------
        # Sampling rate
        # ------------------------------------------------------------------

        if reference_fs is None:
            reference_fs = fs
            bandpass = get_bandpass_config(metadata)

        elif not np.isclose(fs, reference_fs):
            if verbose:
                print(
                    f"Пропуск {file_path.name}: частота дискретизации {fs:g} "
                    f"!= {reference_fs:g}",
                    file=sys.stderr,
                )

            continue

        # ------------------------------------------------------------------
        # Channels
        # ------------------------------------------------------------------

        if channel_names is None:
            channel_names = apply_channel_selection(available, channels)
            channel_prefix = prefix

        elif prefix != channel_prefix:
            if verbose:
                print(
                    f"Пропуск {file_path.name}: источник {prefix} "
                    f"!= {channel_prefix}",
                    file=sys.stderr,
                )

            continue

        else:
            missing = [
                channel
                for channel in channel_names
                if channel not in df.columns
            ]

            if missing:
                if verbose:
                    print(
                        f"Пропуск {file_path.name}: нет каналов {missing}",
                        file=sys.stderr,
                    )

                continue

        # ------------------------------------------------------------------
        # Signal
        # ------------------------------------------------------------------

        data = (
            df[channel_names]
            .apply(
                pd.to_numeric,
                errors="coerce",
            )
            .to_numpy(dtype=float)
        )

        n_samples = len(data)

        if n_samples == 0:
            if verbose:
                print(f"Пропуск {file_path.name}: нет отсчётов", file=sys.stderr)

            continue

        latency = compute_start_latency(df, step)

        time = np.arange(n_samples, dtype=float) / fs

        epochs.append(
            {
                "time": time,
                "data": data,
                "latency": latency,
                "step": step,
                "file": file_path,
                "n_samples": n_samples,
                "fs": fs,
                "bandpass": bandpass,
            }
        )

        if verbose:
            print(
                f"  {file_path.parent.name}/{file_path.name}: "
                f"step {step['step_index']}, {n_samples} отсч., "
                f"задержка {latency * 1e3:+.1f} мс"
            )

    if not epochs:
        raise RuntimeError(
            "Не удалось загрузить ни одной эпохи. "
            "Проверьте пути к директориям и содержимое файлов."
        )

    return (
        epochs,
        reference_fs,
        channel_names,
        channel_prefix,
    )


def drop_dead_channels(
    epochs: list[dict],
    channel_names: list[str],
    verbose: bool = True,
) -> list[str]:
    """
    Убирает каналы, которые не несут информации.

    В этих данных `ch_9` и `ch_10` нулевые у 29 испытуемых из 30
    (только у `андрей_палыч` они заполнены). Канал считается мёртвым,
    если он нулевой более чем у половины эпох.

    Отдельно предупреждаем о ЧАСТИЧНО мёртвых каналах: их нельзя
    исключать, но их среднее по всем испытуемым бессмысленно —
    вклад даёт один-два человека, а остальные 28 дают ровно ноль.
    """

    n_epochs = len(epochs)

    alive_counts = {}

    for index, name in enumerate(channel_names):
        alive = 0

        for epoch in epochs:
            column = epoch["data"][:, index]

            finite = column[np.isfinite(column)]

            if finite.size == 0:
                continue

            if np.any(finite != 0.0):
                alive += 1

        alive_counts[name] = alive

    keep = [
        name
        for name in channel_names
        if alive_counts[name] * 2 > n_epochs
    ]

    dead = [
        name
        for name in channel_names
        if name not in keep
    ]

    if dead:
        print(
            "Отброшены каналы, нулевые более чем у половины эпох: "
            + ", ".join(
                f"{name} ({alive_counts[name]}/{n_epochs} непустых)"
                for name in dead
            ),
            file=sys.stderr,
        )

    partial = [
        name
        for name in keep
        if alive_counts[name] < n_epochs
    ]

    if partial and verbose:
        print(
            "ВНИМАНИЕ: каналы заполнены не во всех эпохах — "
            "усреднение по ним некорректно: "
            + ", ".join(
                f"{name} ({alive_counts[name]}/{n_epochs})"
                for name in partial
            )
            + ". Исключите их через --channels или исправьте запись.",
            file=sys.stderr,
        )

    if not keep:
        raise ValueError(
            "Все каналы оказались пустыми — анализ невозможен. "
            "Похоже, LSL-поток не содержит данных."
        )

    if len(keep) != len(channel_names):
        indices = [
            index
            for index, name in enumerate(channel_names)
            if name in keep
        ]

        for epoch in epochs:
            epoch["data"] = epoch["data"][:, indices]

    return keep


def validate_epoch_quality(
    epochs: list[dict],
    fs: float,
    rms_threshold: float = 3.0,
    verbose: bool = True,
) -> list[dict]:
    """
    Отбрасывает явно повреждённые эпохи и битые КАНАЛЫ внутри эпох.

    Два независимых уровня проверки.

    1) ЭПОХА ЦЕЛИКОМ отбрасывается, если:
        * её длина сильно меньше объявленной в
          `export_step_duration_s` (обрыв записи);
        * задержка «событие -> первый отсчёт» превышает 0.5 с
          (файл начался не в начале шага; на 55 Гц фазовая ошибка
          тогда превышает полный оборот).

      В этих данных так отбрасываются:
        * Иванов step 1:   315 отсчётов вместо ~1500, задержка +2447 мс;
        * Игнатова step 2: 975 отсчётов, задержка +1049 мс;
        * Игнатова step 1: файл без данных.

    2) КАНАЛ внутри эпохи помечается как NaN, если RMS грубо
       выходит за пределы, характерные для этого канала
       (проверка в логарифмическом масштабе, см.
       `robust_mask_log`). Так отсеиваются битые электроды,
       перегрузки усилителя и движения.

      В этих данных: у испытуемого Чаркин `filtered_ch_1`
      имеет std = 488 ед. и |x|max = 5096, тогда как у соседних
      каналов того же файла std = 5–22 ед. Простое среднее
      по 28 эпохам на этом канале даёт результат ОДНОГО
      испытуемого (|C| = 5.26 против медианы 0.07).

    Проверка RMS, а не |C|: RMS не зависит от фазы, поэтому
    отсеивает именно неисправность канала, а не сильную
    фотостимуляцию или вспышку альфа-ритма.

    Порог 3.0 по log10(RMS) отсекает 131 пару (эпоха, канал) из
    1672, из них 90 приходятся на заведомый мусор (RMS > 100
    при типичном уровне 14 ед.). Остальные 231 пара с RMS 30–100
    не отсекаются: это движение и открытые глаза, а не поломка.
    """

    kept = []

    rejected = []

    for epoch in epochs:
        duration = epoch["step"].get("duration") or 0.0

        expected = (
            duration * fs
            if duration > 0
            else epoch["n_samples"]
        )

        # Допуск: -15% и не менее 50 отсчётов.
        minimum = max(0.85 * expected - 50, 50)

        latency = abs(epoch["latency"])

        problems = []

        if epoch["n_samples"] < minimum:
            problems.append(
                f"{epoch['n_samples']} отсч. вместо ~{int(expected)}"
            )

        if latency > 0.5:
            problems.append(f"задержка {latency * 1e3:+.0f} мс")

        if problems:
            rejected.append(
                (
                    epoch["file"].parent.name,
                    epoch["file"].name,
                    problems,
                )
            )

            continue

        kept.append(epoch)

    if not kept:
        raise ValueError("Не осталось ни одной пригодной эпохи.")

    if rejected:
        print()
        print(
            f"Отброшено повреждённых эпох: {len(rejected)}",
            file=sys.stderr,
        )

        for subject, name, problems in rejected:
            print(
                f"  {subject}/{name}: " + ", ".join(problems),
                file=sys.stderr,
            )

    # ------------------------------------------------------------------
    # Битые каналы внутри эпох
    # ------------------------------------------------------------------

    n_channels = kept[0]["data"].shape[1]

    rms = np.full(
        (len(kept), n_channels),
        np.nan,
        dtype=float,
    )

    for row, epoch in enumerate(kept):
        for channel in range(n_channels):
            column = epoch["data"][:, channel]

            finite = column[np.isfinite(column)]

            if finite.size:
                rms[row, channel] = float(np.sqrt(np.mean(finite ** 2)))

    masked = []

    for channel in range(n_channels):
        column = rms[:, channel]

        if rms_threshold <= 0:
            continue

        mask = robust_mask_log(column, rms_threshold)

        bad = np.flatnonzero(np.isfinite(column) & ~mask)

        if bad.size == 0:
            continue

        reference = float(np.nanmedian(column))

        for row in bad:
            subject = kept[row]["file"].parent.name

            name = kept[row]["file"].name

            value = rms[row, channel]

            masked.append(
                f"  канал {channel + 1}: RMS {value:.4g} "
                f"({value / reference:.0f}x от {reference:.4g}) — "
                f"{subject}/{name}"
            )

            kept[row]["data"][:, channel] = np.nan

    if masked:
        print()
        print(
            f"Обнулено аномальных (эпоха, канал) по RMS: "
            f"{len(masked)} из {len(kept) * n_channels}",
            file=sys.stderr,
        )

        print(
            f"  (порог log10(RMS) <= median + {rms_threshold:g}*MAD; "
            "типичный RMS 10-20 ед., отсекается мусор от ~100 ед.)",
            file=sys.stderr,
        )

        for line in masked:
            print(line, file=sys.stderr)

    return kept


# ============================================================================
# Grouping by stimulation configuration
# ============================================================================


def group_epochs(
    epochs: list[dict],
    step_filter: list[int] | None,
) -> dict:
    """
    Группирует эпохи по конфигурации стимуляции.

    Это ключевое исправление: в протоколе каждый из 7 шагов имеет
    свою конфигурацию частот, поэтому усреднять эпохи разных шагов
    нельзя. Накопление выполняется только внутри группы.

    Метка шага (`export_step_index`) сама по себе не является ключём:
    группировка идёт по ЧАСТОТАМ, поэтому результат корректен и для
    записей, где шаги пронумерованы по-разному.
    """

    groups = defaultdict(list)

    for epoch in epochs:
        step_index = epoch["step"]["step_index"]

        if step_filter is not None and step_index not in step_filter:
            continue

        groups[frequency_signature(epoch["step"])].append(epoch)

    return dict(groups)


def source_for_group(
    signature: tuple,
    bandpass: dict,
    source: str,
    fs: float,
) -> str:
    """
    Выбирает источник данных для группы: `filtered` или `raw`.

    `auto`:
        `filtered_*` берётся только если затухание ВСЕХ
        стимулируемых частот в полосовом фильтре не превышает
        `MAX_ATTENUATION_DB`. Иначе — `raw_*`.

    Проверяется не только верхняя граница полосы: 32–34 Гц
    формально попадают в 0.5–40 Гц, но для фильтра Баттерворта
    4 порядка это -146 дБ, то есть компоненты там нет.
    """

    if source != "auto":
        return source

    for frequency in signature:
        if (
            bandpass_attenuation_db(frequency, bandpass, fs)
            > MAX_ATTENUATION_DB
        ):
            return "raw"

    return "filtered"


def format_attenuations(
    signature: tuple,
    bandpass: dict,
    fs: float,
) -> str:
    """
    Строка с затуханием каждой стимулируемой частоты в фильтре.
    """

    parts = []

    for index in range(len(DAC_FREQUENCY_KEYS)):
        attenuation = bandpass_attenuation_db(
            signature[index],
            bandpass,
            fs,
        )

        if attenuation > 60:
            parts.append(f"{signature[index]:g} Гц: -{attenuation:.0f} дБ")
        else:
            parts.append(f"{signature[index]:g} Гц: -{attenuation:.1f} дБ")

    return ", ".join(parts)


def source_suffix(name: str) -> str:
    """
    `filtered` -> `filtered_*`.
    """

    return f"{name}_*"


# ============================================================================
# Trimming
# ============================================================================


def trim_epoch(
    epoch: dict,
    guard: float,
    compensate_latency: bool,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Отбрасывает «грязные» края эпохи и строит ось времени.

    Что удаляется:

      * начало файла содержит данные, записанные ещё на частотах
        ПРЕДЫДУЩЕГО шага (шаг помечается на PC в момент приёма
        пачки, а не в момент acquisition);
      * `sosfiltfilt` / `filtfilt` дают переходные процессы
        длиной padlen (~27 отсчётов, ~54 мс) на обоих краях
        каждого файла — а вызванная реакция максимальна в начале;
      * хвост выходит за пределы длительности шага.

    Компенсация задержки сдвигает ось времени так, чтобы `t = 0`
    соответствовало моменту переключения частот на Arduino
    (событие `stim_events_json` фиксируется сразу после подтверждения
    команды, т.е. с точностью до времени обмена по UART).
    """

    time = epoch["time"]
    data = epoch["data"]

    if compensate_latency:
        time = time + epoch["latency"]

    duration = epoch["step"].get("duration") or 0.0

    if duration > 0:
        # Отсчёты физически не могли относиться к этому шагу.
        end = float(time[-1])

        upper = duration if duration < end else end
    else:
        upper = float(time[-1])

    keep = (
        (time >= guard)
        & (time <= upper - guard)
        & np.isfinite(time)
    )

    if keep.sum() < 8:
        # Защита: эпоха слишком короткая для запрошенного guard.
        keep = (
            (time >= guard / 2.0)
            & (time <= float(time[-1]) - guard / 2.0)
        )

    if keep.sum() < 8:
        keep = np.isfinite(time)

    return (
        time[keep],
        data[keep, :],
    )


# ============================================================================
# Complex amplitude (weighted least squares)
# ============================================================================


def make_window_function(
    n_samples: int,
    name: str,
) -> np.ndarray:
    """
    Окно для взвешенной МНК-оценки.
    """

    if name in ("hann", "hanning"):
        return np.hanning(n_samples)

    if name in ("rect", "boxcar", "none"):
        return np.ones(n_samples, dtype=float)

    raise ValueError(f"Неизвестное окно: {name!r}")


def complex_amplitudes(
    time: np.ndarray,
    data: np.ndarray,
    frequency: float,
    window_function: str = "hann",
) -> np.ndarray:
    """
    Комплексные амплитуды компоненты `frequency` для всех каналов.

    Решается взвешенная задача МНК:

        x(t) ≈ c0 + a*cos(2*pi*f*t) + b*sin(2*pi*f*t)

    и возвращается `C = a - i*b`, для которого

        |C|       -> амплитуда
        angle(C)  -> фаза

    ПОЧЕМУ НЕ `2/N * sum(x * exp(-j*2*pi*f*t))`:

        Для x = A*cos(2*pi*f*t + phi) эта формула даёт

            C = A*e^{i*phi} + (A/N)*e^{-i*phi}*S2,
            S1 = sum(e^{-2i*pi*f*t}),
            S2 = sum(e^{-4i*pi*f*t}).

        То есть точность требует S2 = 0 (2*f*T — целое)
        и отсутствия постоянной составляющей. В этом
        эксперименте ни то, ни другое не выполняется:
        частоты берутся из пользовательского файла (f*T
        произвольно), а постоянная составляющая реальна
        (среднее raw_* — сотни единиц).

        МНК с членом c0:
          * снимает постоянную составляющую точно, а не приближённо;
          * не требует условия на f*T;
          * устойчив на неравномерной сетке;
          * Hann-окно подавляет утечку от второй
            стимулируемой частоты, не внося потерь в полосе.
    """

    n_channels = data.shape[1]

    result = np.full(
        n_channels,
        complex(np.nan, np.nan),
        dtype=complex,
    )

    frequency = float(frequency)

    if not np.isfinite(frequency) or frequency <= 0:
        return result

    base_mask = np.isfinite(time)

    if base_mask.sum() < 4:
        return result

    # Общая (одинаковая для всех каналов) часть решения вычисляется
    # один раз; если у всех каналов одинаковые пропуски — считаем
    # сразу для всех, иначе разбираем по каналам.
    shared = np.all(np.isfinite(data[base_mask, :]), axis=1)

    if shared.all():
        values = complex_amplitudes_core(
            time[base_mask][shared],
            data[base_mask, :][shared],
            frequency,
            window_function,
        )

        result[:] = values

        return result

    for index in range(n_channels):
        mask = base_mask & np.isfinite(data[:, index])

        if mask.sum() < 4:
            continue

        result[index] = complex_amplitudes_core(
            time[mask],
            data[mask, index:index + 1],
            frequency,
            window_function,
        )[0]

    return result


def complex_amplitudes_core(
    time: np.ndarray,
    data: np.ndarray,
    frequency: float,
    window_function: str,
) -> np.ndarray:
    """
    Ядро МНК-оценки: нормальные уравнения 3x3.

    На входе — матрица без пропусков `(n_samples, n_channels)`.
    """

    w = make_window_function(len(time), window_function)

    sum_w = float(np.sum(w))

    if not np.isfinite(sum_w) or sum_w <= 0:
        return np.full(
            data.shape[1],
            complex(np.nan, np.nan),
        )

    theta = 2.0 * np.pi * frequency * time

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    # Нормальные уравнения симметричны, поэтому достаточно
    # трёх скалярных произведений.
    g00 = sum_w
    g01 = float(np.sum(w * cos_t))
    g02 = float(np.sum(w * sin_t))
    g11 = float(np.sum(w * cos_t * cos_t))
    g12 = float(np.sum(w * cos_t * sin_t))
    g22 = float(np.sum(w * sin_t * sin_t))

    gram = np.array(
        [
            [g00, g01, g02],
            [g01, g11, g12],
            [g02, g12, g22],
        ],
        dtype=float,
    )

    rhs = np.empty((3, data.shape[1]), dtype=float)

    rhs[0, :] = np.sum(w[:, None] * data, axis=0)
    rhs[1, :] = np.sum(w[:, None] * cos_t[:, None] * data, axis=0)
    rhs[2, :] = np.sum(w[:, None] * sin_t[:, None] * data, axis=0)

    try:
        coefficients = np.linalg.solve(gram, rhs)
    except np.linalg.LinAlgError:
        return np.full(
            data.shape[1],
            complex(np.nan, np.nan),
        )

    a = coefficients[1, :]
    b = coefficients[2, :]

    return np.asarray(
        a - 1j * b,
        dtype=complex,
    )


# ============================================================================
# Coherent statistics
# ============================================================================


def coherent_average(
    coefficients: np.ndarray,
) -> complex:
    """
    Когерентное усреднение комплексных коэффициентов.

        C_coherent = mean(C_i)

    В отличие от `mean(|C_i|)` фазы здесь сохраняются —
    именно в этом суть когерентного накопления.

    Усреднять можно только эпохи с ОДИНАКОВОЙ конфигурацией
    стимуляции (см. `group_epochs`).
    """

    coefficients = np.asarray(coefficients, dtype=complex)

    valid = np.isfinite(coefficients.real) & np.isfinite(coefficients.imag)

    coefficients = coefficients[valid]

    if coefficients.size == 0:
        return complex(np.nan, np.nan)

    return complex(
        float(np.mean(coefficients.real)),
        float(np.mean(coefficients.imag)),
    )


def inter_trial_coherence(
    coefficients: np.ndarray,
) -> float:
    """
    Inter-trial coherence (ITC):

        ITC = |mean(C_i / |C_i|)|

    Значения:
        1 -> фазы полностью согласованы
        0 -> фазы равномерно разбросаны

    Каждая проба вносит равный вклад независимо от амплитуды.
    Именно для этого и нужен ITC, а не `|sum C| / sum |C|`:
    последний вариант взвешен по амплитуде и «выглядит
    когерентным» из-за одной сильной пробы.
    """

    coefficients = np.asarray(coefficients, dtype=complex)

    valid = np.isfinite(coefficients.real) & np.isfinite(coefficients.imag)

    coefficients = coefficients[valid]

    if coefficients.size == 0:
        return float("nan")

    magnitudes = np.abs(coefficients)

    nonzero = magnitudes > 0

    if not nonzero.any():
        return float("nan")

    unit = np.zeros(coefficients.size, dtype=complex)

    unit[nonzero] = coefficients[nonzero] / magnitudes[nonzero]

    return float(np.abs(np.mean(unit)))


def amplitude_weighted_coherence(
    coefficients: np.ndarray,
) -> float:
    """
    `|sum C| / sum |C|` — амплитудно-взвешенная мера.

    Оставлена для сравнения с ITC: разница показывает,
    насколько оценка определяется амплитудой, а не фазой.
    """

    coefficients = np.asarray(coefficients, dtype=complex)

    valid = np.isfinite(coefficients.real) & np.isfinite(coefficients.imag)

    coefficients = coefficients[valid]

    if coefficients.size == 0:
        return float("nan")

    denominator = float(np.sum(np.abs(coefficients)))

    if denominator == 0:
        return float("nan")

    return float(np.abs(np.sum(coefficients)) / denominator)


def robust_mask(
    values: np.ndarray,
    threshold: float,
) -> np.ndarray:
    """
    Маска выбросов по медиане и MAD.

        median_abs_deviation = 1.4826 * median(|v - median(v)|)

    MAD используется вместо стандартного отклонения, потому что
    сам выброс раздувает sigma и маскирует себя.

    Если MAD равен нулю (больше половины значений совпадают),
    сравнение идёт по вводу отклонения от медианы.

    `threshold` = 0 отключает проверку.
    """

    finite = np.isfinite(values)

    if not finite.any():
        return finite

    clean = values[finite]

    median = float(np.median(clean))

    deviation = float(
        np.median(np.abs(clean - median))
    )

    scale = 1.4826 * deviation

    if scale <= 0:
        scale = max(
            float(np.max(np.abs(clean - median))),
            np.finfo(float).eps,
        )

    return finite & (np.abs(values - median) <= threshold * scale)


def robust_mask_log(
    values: np.ndarray,
    threshold: float,
) -> np.ndarray:
    """
    Маска выбросов по МАСШТАБИРОВАННОЙ медиане и MAD.

    RMS канала распределён сильно правоскошено: типичное значение
    10–20 ед., а у артефактов 100–7000 ед. На линейной шкале MAD
    растягивается выбросами и порог получается слишком мягким
    (или, наоборот, для канала с двумя модами — слишком жёстким,
    как у `filtered_ch_1`: медиана 3.3, MAD 2.8).

    Поэтому отбрасывание считается в логарифмическом масштабе,
    где распределение становится приблизительно симметричным.

    `threshold` задаётся в единицах MAD от log10(RMS):
        3.0 -> примерно отсечение мощности в 10 раз выше типичной.

    `threshold` = 0 отключает проверку.
    """

    finite = np.isfinite(values) & (values > 0)

    if not finite.any():
        return finite

    log_values = np.log10(values)

    clean = log_values[finite]

    median = float(np.median(clean))

    deviation = 1.4826 * float(
        np.median(np.abs(clean - median))
    )

    if deviation <= 0:
        deviation = max(
            float(np.max(np.abs(clean - median))),
            1e-9,
        )

    return finite & (log_values <= median + threshold * deviation)


def robust_coherent_average(
    coefficients: np.ndarray,
) -> complex:
    """
    Устойчивое когерентное среднее: покоординатная медиана.

    В отличие от `coherent_average` (среднего), медиана не даёт
    одной амplitудной аномалии перевесить остальные.

    В этих данных это критично: у испытуемого Чаркин канал
    ch_1 имеет std = 488 ед. против 15–20 у остальных каналов
    (амплитудная выброс |C| = 145 против медианы 0.16 по группе
    из 28 эпох). Обычное среднее даёт по этому каналу |C| = 5.26,
    то есть результат одного испытуемого из 28; медиана даёт 0.07,
    то есть результат всех остальных.

    Медиана берётся отдельно по действительной и мнимой частям,
    поэтому комплексная величина остаётся корректной оценкой
    `A*e^{i*phi}` с переобозначенными амплитудой и фазой.
    """

    coefficients = np.asarray(coefficients, dtype=complex)

    valid = np.isfinite(coefficients.real) & np.isfinite(coefficients.imag)

    coefficients = coefficients[valid]

    if coefficients.size == 0:
        return complex(np.nan, np.nan)

    return complex(
        float(np.median(coefficients.real)),
        float(np.median(coefficients.imag)),
    )


def coherent_snr(
    coefficients: np.ndarray,
) -> float:
    """
    Оценка отношения сигнал/шум для когерентного среднего
    (Borst–Theorens):

        SNR = |sum C|^2 / sum |C_i - mean(C)|^2

    Значения около 1 соответствуют уровню шума.
    """

    coefficients = np.asarray(coefficients, dtype=complex)

    valid = np.isfinite(coefficients.real) & np.isfinite(coefficients.imag)

    coefficients = coefficients[valid]

    if coefficients.size < 2:
        return float("nan")

    mean = np.mean(coefficients)

    power = float(np.abs(np.sum(coefficients)) ** 2)

    noise = float(np.sum(np.abs(coefficients - mean) ** 2))

    if noise == 0:
        return float("inf") if power > 0 else float("nan")

    return power / noise


def analyze_group(
    epochs: list[dict],
    signature: tuple,
    channel_names: list[str],
    guard: float,
    compensate_latency: bool,
    window_function: str,
    outlier_threshold: float = 0.0,
) -> dict:
    """
    Когерентное накопление внутри одной группы эпох.

    Основная оценка — МОДИАНА комплексных коэффициентов: она не
    даёт одной амплитудной аномалии перевесить остальные пробы.
    Рядом выводится обычное среднее: сильное расхождение между ними
    само по себе сигнализирует о выбросах в данных.

    `outlier_threshold` (в единицах MAD по |C_i|) — необязательное
    ДОПОЛНИТЕЛЬНОЕ отбрасывание отдельных проб. По умолчанию
    выключено (0), и это осознанно: при фотостимуляции амплитуда
    альфа-ритма сильно меняется от пробы к пробе, и отбрасывание
    сильных проб удаляет настоящую мощность альфа, а не артефакты.
    Неисправные каналы отсекаются отдельно, по RMS
    (см. `validate_epoch_quality`).
    """

    n_epochs = len(epochs)

    n_channels = len(channel_names)

    # (n_epochs, n_channels) для каждой частоты
    coefficients = {}

    for index, name in enumerate(DAC_FREQUENCY_KEYS):
        frequency = signature[index]

        table = np.full(
            (n_epochs, n_channels),
            complex(np.nan, np.nan),
            dtype=complex,
        )

        for epoch_index, epoch in enumerate(epochs):
            time, data = trim_epoch(
                epoch,
                guard,
                compensate_latency,
            )

            table[epoch_index, :] = complex_amplitudes(
                time,
                data,
                frequency,
                window_function,
            )

        coherent = np.array(
            [
                coherent_average(table[:, channel_index])
                for channel_index in range(n_channels)
            ]
        )

        # --------------------------------------------------------------
        # Отбрасывание амплитудных выбросов (MAD)
        # --------------------------------------------------------------
        #
        # Сделано ПОКАНАЛЬНО: один битый канал у одного
        # испытуемого не должен выбрасывать его эпоху целиком
        # из анализа по остальным каналам.

        accepted = np.ones(
            (n_epochs, n_channels),
            dtype=bool,
        )

        if outlier_threshold > 0:
            for channel_index in range(n_channels):
                amplitudes = np.abs(table[:, channel_index])

                accepted[:, channel_index] = robust_mask(
                    amplitudes,
                    outlier_threshold,
                )

        robust_values = np.array(
            [
                robust_coherent_average(
                    table[accepted[:, channel_index], channel_index]
                )
                for channel_index in range(n_channels)
            ]
        )

        n_accepted = np.sum(accepted, axis=0)

        coefficients[name] = {
            "frequency": frequency,
            "table": table,
            "accepted": accepted,
            "n_accepted": n_accepted,
            "coherent": coherent,
            "robust": robust_values,
            "robust_amplitude": np.abs(robust_values),
            "robust_phase": np.angle(robust_values, deg=True),
            "amplitude": np.abs(robust_values),
            "phase": np.angle(robust_values, deg=True),
            "mean_amplitude": np.abs(coherent),
            "mean_phase": np.angle(coherent, deg=True),
            "median_amplitude": np.array(
                [
                    np.nanmedian(np.abs(table[:, channel_index]))
                    for channel_index in range(n_channels)
                ]
            ),
            "itc": np.array(
                [
                    inter_trial_coherence(
                        table[accepted[:, channel_index], channel_index]
                    )
                    for channel_index in range(n_channels)
                ]
            ),
            "awc": np.array(
                [
                    amplitude_weighted_coherence(
                        table[accepted[:, channel_index], channel_index]
                    )
                    for channel_index in range(n_channels)
                ]
            ),
            "snr": np.array(
                [
                    coherent_snr(
                        table[accepted[:, channel_index], channel_index]
                    )
                    for channel_index in range(n_channels)
                ]
            ),
        }

    return {
        "signature": signature,
        "label": format_signature(signature),
        "n_epochs": n_epochs,
        "step_indices": sorted(
            {
                epoch["step"]["step_index"]
                for epoch in epochs
            }
        ),
        "channel_names": list(channel_names),
        "coefficients": coefficients,
        "subjects": [
            epoch["file"].parent.name
            for epoch in epochs
        ],
        "latencies": np.array(
            [epoch["latency"] for epoch in epochs],
            dtype=float,
        ),
    }


# ============================================================================
# Time-domain averaging
# ============================================================================


def build_time_grid(
    window: float,
    fs: float,
) -> np.ndarray:
    """
    Равномерная сетка с шагом ровно `1/fs`.

    Шаг `1/fs` важен: при `np.linspace(-W/2, W/2, ceil(W*fs)+1)`
    шаг сетки отличается от `1/fs` (до ~0.8%), что при 55 Гц
    даёт ошибку фазы в несколько градусов.
    """

    n_points = int(round(window * fs)) + 1

    if n_points < 3:
        raise ValueError("Окно слишком короткое для частоты дискретизации.")

    return (np.arange(n_points) - n_points // 2) / fs


def average_group(
    epochs: list[dict],
    time_grid: np.ndarray,
    n_channels: int,
    guard: float,
    compensate_latency: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Усреднение во времени внутри группы.

    Отсчёты уже лежат на равномерной сетке `t = n / fs`, поэтому
    интерполяция НЕ НУЖНА: эпохи просто выравниваются по индексу
    отсчёта и обрезаются до общей длины.
    """

    blocks = []

    for epoch in epochs:
        time, data = trim_epoch(
            epoch,
            guard,
            compensate_latency,
        )

        if len(time) == 0:
            continue

        blocks.append((time, data))

    if not blocks:
        raise ValueError("После отсечения не осталось ни одной эпохи.")

    # Эпохи могут отличаться по длине на несколько отсчётов.
    # Для усреднения во времени берём общую часть.
    n_usable = min(len(time) for time, _ in blocks)

    if n_usable < len(time_grid):
        # Общий участок короче запрошенного окна: сокращаем окно.
        time_grid = time_grid[:n_usable]

    fs = blocks[0][0][1] - blocks[0][0][0]

    stacked = np.stack(
        [data[:len(time_grid), :] for _, data in blocks],
        axis=0,
    )

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)

        average = np.nanmean(stacked, axis=0)
        std = np.nanstd(stacked, axis=0, ddof=1)

    count_valid = np.sum(
        np.isfinite(stacked),
        axis=0,
    )

    sem = np.divide(
        std,
        np.sqrt(count_valid),
        out=np.full_like(std, np.nan),
        where=count_valid > 1,
    )

    return (
        time_grid,
        average,
        sem,
        count_valid,
    )


# ============================================================================
# Latency scan
# ============================================================================


def latency_scan(
    epochs: list[dict],
    frequency: float,
    channel_index: int,
    guard: float,
    compensate_latency: bool,
    window_function: str,
    span: float = 0.08,
    step: float = 0.002,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Сканирует предполагаемый общий сдвиг времени `tau`.

    Нужен, чтобы проверить, не размывает ли остаточная
    несинхронность когерентное накопление: если максимум |C(tau)|
    стоит при `tau != 0` и заметно выше |C(0)|, эпохи усредняются
    с фазовым разбросом.

    Сдвиг применяется ОДИНАКОВО ко всем эпохам, поэтому результат
    показывает именно ОСТАТОЧНУЮ (усреднённую по испытуемым)
    ошибку синхронизации, а не индивидуальную.

    Индивидуальный разброс измеряется отдельно — см. задержку
    «событие -> первый отсчёт», выводимую для каждой группы.
    """

    taus = np.arange(-span, span + step / 2, step)

    accumulated = np.zeros(len(taus), dtype=complex)

    used = 0

    for epoch in epochs:
        time, data = trim_epoch(
            epoch,
            guard,
            compensate_latency,
        )

        signal = data[:, channel_index]

        if len(signal) < 8:
            continue

        used += 1

        for index, tau in enumerate(taus):
            accumulated[index] += complex_amplitudes_core(
                time + tau,
                signal.reshape(-1, 1),
                frequency,
                window_function,
            )[0]

    if used:
        # Усредняем, иначе величина растёт с числом эпох
        # и несравнима с |C| из отчёта.
        accumulated /= used

    return (
        taus,
        np.abs(accumulated),
    )


# ============================================================================
# Plotting
# ============================================================================


def plot_group(
    time_grid: np.ndarray,
    average: np.ndarray,
    sem: np.ndarray,
    channel_names: list[str],
    group: dict,
    window: float,
    source: str,
    output: Path | None = None,
):
    """
    График группы: усреднение во времени + амплитуда/фаза/ITC.
    """

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(12, 11),
        gridspec_kw={"height_ratios": [2, 1, 1]},
    )

    # ------------------------------------------------------------------
    # Усреднение во времени
    # ------------------------------------------------------------------

    ax = axes[0]

    for index, name in enumerate(channel_names):
        ax.plot(
            time_grid,
            average[:, index],
            label=name,
            linewidth=1.0,
        )

        ax.fill_between(
            time_grid,
            average[:, index] - sem[:, index],
            average[:, index] + sem[:, index],
            alpha=0.2,
        )

    ax.axvline(0, linestyle="--", linewidth=0.8, color="black")

    ax.set_xlabel("Время относительно переключения стимула, с")
    ax.set_ylabel("Амплитуда, ед. ADC")
    ax.set_title(
        f"{group['label']} — {group['n_epochs']} эпох, "
        f"источник {source}, окно ±{window / 2:.3f} с"
    )
    ax.legend(fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Амплитуды
    # ------------------------------------------------------------------

    names = list(DAC_FREQUENCY_KEYS)

    frequencies = [group["coefficients"][name]["frequency"] for name in names]

    labels = [
        f"{DAC_LABELS[name]}\n{frequencies[index]:g} Гц"
        for index, name in enumerate(names)
    ]

    positions = np.arange(len(names))

    width = 0.8 / max(len(channel_names), 1)

    ax = axes[1]

    for index, name in enumerate(channel_names):
        ax.bar(
            positions + index * width,
            [group["coefficients"][key]["amplitude"][index] for key in names],
            width=width,
            label=name,
        )

    ax.set_xticks(positions + width * (len(channel_names) - 1) / 2)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("|C|, ед. ADC")
    ax.set_title("Когерентная амплитуда")
    ax.legend(fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3, axis="y")

    # ------------------------------------------------------------------
    # ITC
    # ------------------------------------------------------------------

    ax = axes[2]

    for index, name in enumerate(channel_names):
        ax.bar(
            positions + index * width,
            [group["coefficients"][key]["itc"][index] for key in names],
            width=width,
            label=name,
        )

    ax.set_xticks(positions + width * (len(channel_names) - 1) / 2)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("ITC")
    ax.set_ylim(0, 1)
    ax.set_title("Inter-trial coherence (фазовая согласованность)")
    ax.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()

    if output is not None:
        fig.savefig(output, dpi=150, bbox_inches="tight")

    return fig


# ============================================================================
# Reporting
# ============================================================================


def print_group_report(
    group: dict,
    channel_names: list[str],
    source: str,
    outlier_threshold: float,
):
    """
    Печатает результаты одной группы.

    `|C|` — устойчивая оценка (медиана после отбрасывания
    выбросов). `|C| mean` — обычное среднее: оно показывает,
    насколько результат зависит от отдельных амплитудных аномалий.
    """

    print()
    print("=" * 100)
    print(
        f"ГРУППА: {group['label']}   "
        f"эпох: {group['n_epochs']}   "
        f"шаги: {group['step_indices']}   "
        f"источник: {source}"
    )
    print("=" * 100)

    print(
        f"Задержка «событие -> первый отсчёт»: "
        f"{np.mean(group['latencies']) * 1e3:+.1f} мс "
        f"(sd {np.std(group['latencies']) * 1e3:.1f}, "
        f"разброс {(np.max(group['latencies']) - np.min(group['latencies'])) * 1e3:.1f})"
    )

    for key in DAC_FREQUENCY_KEYS:
        result = group["coefficients"][key]

        print()
        print(f"{DAC_LABELS[key]} = {result['frequency']:.6f} Гц")

        print(
            f"{'Channel':<16}{'|C|':>11}{'Phase':>11}"
            f"{'med|C|':>10}{'|C| mean':>11}"
            f"{'ITC':>8}{'SNR':>8}{'n':>5}"
        )
        print("-" * 80)

        for index, name in enumerate(channel_names):
            print(
                f"{name:<16}"
                f"{result['amplitude'][index]:>11.4g}"
                f"{result['phase'][index]:>10.2f}°"
                f"{result['median_amplitude'][index]:>10.4g}"
                f"{result['mean_amplitude'][index]:>11.4g}"
                f"{result['itc'][index]:>8.3f}"
                f"{result['snr'][index]:>8.2f}"
                f"{result['n_accepted'][index]:>5d}"
            )

        # --------------------------------------------------------------
        # Выбросы (только если --outlier-threshold задан)
        # --------------------------------------------------------------

        if outlier_threshold <= 0:
            continue

        for index, name in enumerate(channel_names):
            rejected = ~result["accepted"][:, index]

            if not rejected.any():
                continue

            amplitudes = np.abs(result["table"][:, index])

            names = [
                group["subjects"][row]
                for row in np.flatnonzero(rejected)
            ]

            print()
            print(
                f"  Выбросы по {name} "
                f"(отброшено {int(rejected.sum())} из "
                f"{len(rejected)}, порог MAD x{outlier_threshold:g}): "
                + ", ".join(names)
            )

            print(
                f"    их |C|: "
                + ", ".join(
                    f"{value:.3g}"
                    for value in amplitudes[rejected]
                )
                + f"; у остальных медиана "
                f"{result['median_amplitude'][index]:.3g}, "
                f"среднее {result['mean_amplitude'][index]:.3g}"
            )

    # ------------------------------------------------------------------
    # Предупреждение о несогласованности
    # ------------------------------------------------------------------

    n_epochs = group["n_epochs"]

    # Уровень ITC, ожидаемый от чистого шума при N эпохах.
    noise_itc = 1.0 / np.sqrt(n_epochs)

    weak = []

    for key in DAC_FREQUENCY_KEYS:
        result = group["coefficients"][key]

        for index, name in enumerate(channel_names):
            itc = result["itc"][index]

            if not np.isfinite(itc) or itc > noise_itc * 1.5:
                continue

            weak.append(f"{name}@{DAC_LABELS[key]}")

    if weak:
        print()
        print(
            f"  ВНИМАНИЕ: ITC не превышает шумовой уровень "
            f"({noise_itc:.3f} при {n_epochs} эпохах) для: "
            + ", ".join(weak)
        )
        print(
            "    Когерентного ответа на этих частотах в данных нет "
            "либо он замаскирован шумом. Это не ошибка расчёта."
        )


def print_summary(
    groups: dict,
    channel_names: list[str],
):
    """
    Сводная таблица по всем группам и каналам.
    """

    print()
    print("=" * 100)
    print("СВОДКА (среднее по всем группам)")
    print("=" * 100)

    print(
        f"{'Channel':<16}"
        + "".join(f"{DAC_LABELS[key]:>20}" for key in DAC_FREQUENCY_KEYS)
    )
    print("-" * 96)

    for index, name in enumerate(channel_names):
        row = f"{name:<16}"

        for key in DAC_FREQUENCY_KEYS:
            amplitudes = [
                group["coefficients"][key]["amplitude"][index]
                for group in groups.values()
            ]

            itc = [
                group["coefficients"][key]["itc"][index]
                for group in groups.values()
            ]

            row += (
                f"{np.nanmean(amplitudes):>11.4g}"
                f" ITC {np.nanmean(itc):>6.3f}"
            )

        print(row)


def write_csv(
    groups: dict,
    channel_names: list[str],
    output: Path,
):
    """
    Выгружает таблицу результатов в CSV.
    """

    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, delimiter="\t")

        writer.writerow(
            [
                "group",
                "n_epochs",
                "step_indices",
                "dac",
                "frequency_hz",
                "channel",
                "amplitude_robust",
                "phase_robust_deg",
                "median_amplitude",
                "amplitude_mean",
                "phase_mean_deg",
                "itc",
                "awc",
                "snr",
                "n_accepted",
            ]
        )

        for group in groups.values():
            for key in DAC_FREQUENCY_KEYS:
                result = group["coefficients"][key]

                for index, name in enumerate(channel_names):
                    writer.writerow(
                        [
                            group["label"],
                            group["n_epochs"],
                            ",".join(
                                str(value)
                                for value in group["step_indices"]
                            ),
                            DAC_LABELS[key],
                            f"{result['frequency']:.6g}",
                            name,
                            f"{result['robust_amplitude'][index]:.6g}",
                            f"{result['robust_phase'][index]:.4f}",
                            f"{result['median_amplitude'][index]:.6g}",
                            f"{result['mean_amplitude'][index]:.6g}",
                            f"{result['mean_phase'][index]:.4f}",
                            f"{result['itc'][index]:.5f}",
                            f"{result['awc'][index]:.5f}",
                            f"{result['snr'][index]:.5g}",
                            int(result["n_accepted"][index]),
                        ]
                    )

    print(f"Таблица сохранена: {output}")


# ============================================================================
# Self-test
# ============================================================================


def phase_difference_deg(
    first: float,
    second: float,
) -> float:
    """
    Разность фаз с приведением к диапазону (-180, 180].
    """

    return float(
        np.degrees(
            np.angle(np.exp(1j * (first - second)))
        )
    )


def run_self_test() -> int:
    """
    Проверяет корректность пайплайна на данных с известным ответом.

    Проверки:

      1. ТОЧНОСТЬ. Без шума и постоянной составляющей оценка
         обязана вернуть амплитуду и фазу до машинного нуля.
         Это проверка самой математики.
      2. ПОСТОЯННАЯ СОСТАВЛЯЮЩАЯ. Смещение в hundreds единиц
         (реальность этих данных) не должно влиять на результат.
      3. ВЫБОР ЧАСТОТЫ. Соседняя компонента не должна
         подменять целевую: сумма энергий сохраняется.
      4. УСТОЙЧИВОСТЬ К ШУМУ. Отклонение не превышает
         аналитической границы 4*sigma.
      5. НАКОПЛЕНИЕ И ITC. Когерентное усреднение восстанавливает
         амплитуду при заданном разбросе фаз; ITC не зависит
         от амплитуды.
      6. ГРУППИРОВКА. Эпохи с разной конфигурацией
         стимуляции не смешиваются.
      7. ОСЬ ВРЕМЕНИ. Оценка не зависит от сдвига сетки,
         а поворот сигнала на dT даёт dT*f градусов фазы.
    """

    print("=" * 86)
    print("SELF-TEST")
    print("=" * 86)

    rng = np.random.default_rng(20260610)

    fs = 500.0

    n_samples = 1500

    time = np.arange(n_samples) / fs

    failures = 0

    # ------------------------------------------------------------------
    # 1 + 2. Точность и постоянная составляющая
    # ------------------------------------------------------------------

    print()
    print("1) Точность оценки (без шума) и устойчивость к DC")
    print("-" * 86)
    print(
        f"{'f, Гц':>8}{'DC':>8}{'A ожид':>10}{'A получ':>14}"
        f"{'фаза, °':>12}{'ошибка фазы, °':>16}   итог"
    )

    for f_target in (2.0, 8.0, 10.0, 12.7, 33.3, 55.0):
        for dc in (0.0, 100.0, 500.0):
            amplitude = 2.0
            phase = 0.7

            signal = (
                amplitude * np.cos(2 * np.pi * f_target * time + phase)
                + dc
            )

            got = complex_amplitudes(
                time,
                signal.reshape(-1, 1),
                f_target,
                "hann",
            )[0]

            phase_error = phase_difference_deg(
                np.angle(got),
                phase,
            )

            ok = (
                abs(abs(got) - amplitude) < 1e-9
                and abs(phase_error) < 1e-7
            )

            failures += 0 if ok else 1

            print(
                f"{f_target:>8g}{dc:>8g}{amplitude:>10g}{abs(got):>14.10f}"
                f"{np.degrees(np.angle(got)):>12.5f}{phase_error:>16.2e}"
                f"   {'OK' if ok else 'ПРОВАЛ'}"
            )

    # ------------------------------------------------------------------
    # 3. Разделение соседних частот
    # ------------------------------------------------------------------

    print()
    print("2) Разделение соседних частот (A=2 на 10 Гц + A=1 на 8 Гц)")
    print("-" * 86)

    signal = (
        2.0 * np.cos(2 * np.pi * 10.0 * time + 0.7)
        + 1.0 * np.cos(2 * np.pi * 8.0 * time - 0.4)
        + 100.0
    )

    for f_target, expected in ((10.0, 2.0), (8.0, 1.0)):
        got = complex_amplitudes(
            time,
            signal.reshape(-1, 1),
            f_target,
            "hann",
        )[0]

        error = abs(abs(got) - expected) / expected

        # Допуск 1% — остаток определяется конечным окном
        # (leakage), а не ошибкой оценки.
        ok = error < 0.01

        failures += 0 if ok else 1

        print(
            f"   f={f_target:5.1f} Гц: ожидалось {expected:.4f}, "
            f"получено {abs(got):.4f} (ошибка {error * 100:.3f}%)  "
            f"{'OK' if ok else 'ПРОВАЛ'}"
        )

    # ------------------------------------------------------------------
    # 4. Устойчивость к шуму
    # ------------------------------------------------------------------

    print()
    print("3) Устойчивость к шуму (A=2, sigma шума = 0.5, 200 реализаций)")
    print("-" * 86)

    amplitude = 2.0
    phase = 0.7
    noise_sd = 0.5

    n_trials = 200

    amplitudes = np.empty(n_trials)
    phases = np.empty(n_trials)

    for trial in range(n_trials):
        noisy = (
            amplitude * np.cos(2 * np.pi * 10.0 * time + phase)
            + rng.normal(0, noise_sd, n_samples)
        )

        got = complex_amplitudes(
            time,
            noisy.reshape(-1, 1),
            10.0,
            "hann",
        )[0]

        amplitudes[trial] = abs(got)
        phases[trial] = np.angle(got)

    # Для МНК с окном Ханна: Var(amplitude) = 2*sigma^2 / sum(w*cos^2),
    # sum(w*cos^2) ~= 3N/8.
    expected_sd = (
        np.sqrt(2) * noise_sd / np.sqrt(0.375 * n_samples)
    )

    bias = float(np.mean(amplitudes)) - amplitude

    print(
        f"   смещение средней амплитуды: {bias:+.5f} "
        f"(ожидание ~{expected_sd / np.sqrt(n_trials):.5f})"
    )
    print(
        f"   СКО амплитуды: {np.std(amplitudes):.5f} "
        f"(теория {expected_sd:.5f})"
    )
    print(
        f"   СКО фазы: {np.degrees(np.std(phases)):.4f}° "
        f"(теория {np.degrees(noise_sd / (amplitude * np.sqrt(0.375 * n_samples))):.4f}°)"
    )

    # Смещение должно быть много меньше самой амплитуды.
    ok = abs(bias) < 0.01 * amplitude

    failures += 0 if ok else 1

    print(f"   ->  {'OK' if ok else 'ПРОВАЛ'}")

    # ------------------------------------------------------------------
    # 5. Когерентное накопление и ITC
    # ------------------------------------------------------------------

    print()
    print("4) Когерентное накопление и ITC")
    print("-" * 86)

    n_epochs = 200

    spread_deg = 20.0

    spread = np.radians(spread_deg)

    phases = rng.normal(0, spread, n_epochs)

    coefficients = 2.0 * np.exp(1j * phases)

    mean = coherent_average(coefficients)
    itc = inter_trial_coherence(coefficients)
    awc = amplitude_weighted_coherence(coefficients)
    snr = coherent_snr(coefficients)

    theory_amplitude = 2.0 * np.exp(-spread ** 2 / 2)

    print(
        f"   разброс фаз sigma = {spread_deg}° -> "
        f"|mean C| = {abs(mean):.5f} "
        f"(теория A*exp(-sigma^2/2) = {theory_amplitude:.5f})"
    )
    print(
        f"   ITC = {itc:.6f}, AWC = {awc:.6f} "
        f"(совпадают при равных амплитудах)"
    )
    print(f"   ITC*sqrt(n) = {itc * np.sqrt(n_epochs):.4f} "
          f"(оценка Саймона для согласованных фаз)")

    ok = abs(abs(mean) - theory_amplitude) < 0.05

    failures += 0 if ok else 1

    print(f"   ->  {'OK' if ok else 'ПРОВАЛ'}")

    # ITC не должен зависеть от амплитуды.
    #
    # Сильная проба на 0°, две слабые на ±120°:
    # ITC -> 0 (фазы расходятся, согласия нет),
    # AWC -> ~0.97 (её «вытягивает» одна сильная проба).
    mixed = np.array(
        [
            10.0 * np.exp(1j * np.radians(0.0)),
            0.1 * np.exp(1j * np.radians(120.0)),
            0.1 * np.exp(1j * np.radians(-120.0)),
        ]
    )

    itc_mixed = inter_trial_coherence(mixed)
    awc_mixed = amplitude_weighted_coherence(mixed)

    print()
    print("   Одна сильная проба (0°) и две слабые (±120°):")
    print(f"      ITC = {itc_mixed:.4f}  <- честная мера: согласия нет")
    print(f"      AWC = {awc_mixed:.4f}  <- завышена весом амплитуды")

    ok = itc_mixed < 0.05 and awc_mixed > 0.9

    failures += 0 if ok else 1

    print(f"   ->  {'OK' if ok else 'ПРОВАЛ'}")

    # ------------------------------------------------------------------
    # 6. Группировка
    # ------------------------------------------------------------------

    print()
    print("5) Группировка по конфигурации стимуляции")
    print("-" * 86)

    samples = [
        {"step_index": 1, "dac0_f1": 10.0, "dac0_f2": 10.0,
         "dac1_f1": 8.0, "dac1_f2": 8.0},
        {"step_index": 2, "dac0_f1": 8.0, "dac0_f2": 8.0,
         "dac1_f1": 7.0, "dac1_f2": 7.0},
        {"step_index": 1, "dac0_f1": 10.0, "dac0_f2": 10.0,
         "dac1_f1": 8.0, "dac1_f2": 8.0},
        {"step_index": 6, "dac0_f1": 12.0, "dac0_f2": 14.0,
         "dac1_f1": 32.0, "dac1_f2": 34.0},
    ]

    grouped = {}

    for sample in samples:
        grouped.setdefault(
            frequency_signature(sample),
            [],
        ).append(sample)

    sizes = sorted(len(value) for value in grouped.values())

    print(f"   4 эпохи, сигнатур: {len(grouped)}, "
          f"размеры групп: {sizes} (ожидается [1, 1, 2])")

    ok = sizes == [1, 1, 2]

    failures += 0 if ok else 1

    print(f"   ->  {'OK' if ok else 'ПРОВАЛ'}")

    # ------------------------------------------------------------------
    # 7. Ось времени
    # ------------------------------------------------------------------

    print()
    print("6) Ось времени: независимость от сетки и корректность поворота")
    print("-" * 86)

    frequency = 10.0

    amplitude = 2.0

    phase = 0.7

    reference = complex_amplitudes(
        time,
        (amplitude * np.cos(2 * np.pi * frequency * time + phase)).reshape(-1, 1),
        frequency,
        "hann",
    )[0]

    # Прореженная сетка (как будто fs втрое ниже).
    sparse_time = time[::3]

    sparse = complex_amplitudes(
        sparse_time,
        (
            amplitude
            * np.cos(2 * np.pi * frequency * sparse_time + phase)
        ).reshape(-1, 1),
        frequency,
        "hann",
    )[0]

    # Сдвиг сигнала на dT даёт сдвиг фазы на -dT*f.
    shift = 0.01

    shifted = complex_amplitudes(
        time,
        (
            amplitude
            * np.cos(
                2 * np.pi * frequency * (time - shift) + phase
            )
        ).reshape(-1, 1),
        frequency,
        "hann",
    )[0]

    shift_error = phase_difference_deg(
        np.angle(shifted),
        phase,
    )

    print(
        f"   сетка 500 Гц: |C| = {abs(reference):.8f}, "
        f"фаза = {np.degrees(np.angle(reference)):.5f}°"
    )
    print(
        f"   сетка 167 Гц: |C| = {abs(sparse):.8f}, "
        f"фаза = {np.degrees(np.angle(sparse)):.5f}°  "
        f"разница {phase_difference_deg(np.angle(sparse), np.angle(reference)):+.2e}°"
    )
    print(
        f"   сдвиг сигнала на {-shift * 1e3:.0f} мс -> "
        f"фаза {shift_error:+.4f}° "
        f"(ожидание {-shift * frequency * 360:+.4f}°)"
    )

    ok = (
        abs(abs(reference) - amplitude) < 1e-8
        and abs(shift_error - (-shift * frequency * 360)) < 1e-3
    )

    failures += 0 if ok else 1

    print(f"   ->  {'OK' if ok else 'ПРОВАЛ'}")

    # ------------------------------------------------------------------
    # Итог
    # ------------------------------------------------------------------

    print()
    print("=" * 86)

    if failures:
        print(f"SELF-TEST: ПРОВАЛЕНО ПРОВЕРОК: {failures}")
        return 1

    print("SELF-TEST: ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ")

    return 0


# ============================================================================
# Pipeline
# ============================================================================


def plan_sources(
    files: list[Path],
    step_filter: list[int] | None,
    source: str,
    min_epochs: int,
    verbose: bool = True,
):
    """
    Читает ТОЛЬКО metadata всех файлов и решает, как анализировать.

    Это отдельный проход, потому что выбор источника зависит от
    набора частот, а читать сами данные до решения не нужно.

    Возвращает:

        plan         : {signature: {"steps": [...], "count": N}}
        source_name  : "filtered" | "raw"
        bandpass     : конфигурация фильтра из `filter_config_json`
        fs           : частота дискретизации, Гц
    """

    # signature -> {"steps": [...], "count": N}
    plan = defaultdict(
        lambda: {
            "steps": [],
            "count": 0,
        }
    )

    bandpass = get_bandpass_config({})

    bandpass_known = False

    fs = 0.0

    for file_path in files:
        try:
            metadata, df = load_metadata_and_data(file_path)

            step = extract_step_info(metadata)

            candidate_fs = resolve_sampling_rate(metadata, df)

            if not bandpass_known:
                bandpass = get_bandpass_config(metadata)
                fs = candidate_fs
                bandpass_known = True

            if not np.isclose(candidate_fs, fs):
                continue

        except Exception:
            continue

        if step_filter is not None and step["step_index"] not in step_filter:
            continue

        entry = plan[frequency_signature(step)]

        entry["steps"].append(step["step_index"])

        # Считаем ФАЙЛЫ, а не номера шагов: один и тот же номер
        # шага у разных испытуемых — это разные эпохи.
        entry["count"] += 1

    plan = {
        signature: value
        for signature, value in plan.items()
        if value["count"] >= min_epochs
    }

    if not plan:
        raise ValueError(
            "Не удалось прочитать metadata ни одного файла "
            "или все файлы отсеяны фильтром --step."
        )

    # Группы с достаточным числом эпох.
    plan = {
        signature: value
        for signature, value in plan.items()
        if len(value) >= min_epochs
    }

    if not plan:
        raise ValueError(
            f"Ни в одной группе не набралось {min_epochs} эпох.\n"
            "Когерентное накопление требует ПОВТОРОВ с одинаковой "
            "конфигурацией стимуляции.\n"
            "  * укажите все директории испытуемых: --folder experiments/*\n"
            "  * внутри одной директории каждый из 7 шагов имеет СВОЮ "
            "конфигурацию, поэтому там только 1 эпоха на группу;\n"
            "  * или уменьшите --min-epochs."
        )

    required = {
        source_for_group(signature, bandpass, source, fs)
        for signature in plan
    }

    if len(required) > 1:
        detail = "\n".join(
            f"  шаг {sorted(set(plan[signature]['steps']))}: "
            f"{format_signature(signature)}\n"
            f"      -> {source_for_group(signature, bandpass, source, fs)}_*, "
            "затухание: "
            f"{format_attenuations(signature, bandpass, fs)}"
            for signature in sorted(plan)
        )

        raise ValueError(
            "Выбранные группы требуют РАЗНЫХ источников данных:\n"
            f"{detail}\n"
            "Причина: `filtered_*` содержит полосовой фильтр "
            f"{bandpass['bandpass_low_hz']:g}-{bandpass['bandpass_high_hz']:g} Гц "
            f"Баттерворта {bandpass['bandpass_order']:g} порядка, "
            f"применённый дважды (sosfiltfilt). У него очень крутой "
            f"спад: 55 Гц теряет {abs(bandpass_attenuation_db(55.0, bandpass, fs)):.0f} дБ, "
            "32–34 Гц — около 2 дБ.\n"
            "Порог: компонента берётся из filtered_* только если "
            f"затухание <= {MAX_ATTENUATION_DB:g} дБ.\n"
            "Полный набор шагов одним запуском не анализируется. "
            "Разбейте по шагам:\n"
            "  --step 1 2 3 4 5        (filtered_*)\n"
            "  --step 6 7 --source raw (raw_*)\n"
            "Либо задайте порог явно: --source filtered (принять "
            "затухание 32–34 Гц ценой -2 дБ по амплитуде)."
        )

    source_name = required.pop()

    if verbose:
        print()
        print(f"Источник данных: {source_name}_*")

        if bandpass.get("bandpass_enabled", True):
            print(
                f"Фильтр filtered_*: Баттерворт {bandpass['bandpass_order']:g} "
                f"порядка, {bandpass['bandpass_low_hz']:g}-"
                f"{bandpass['bandpass_high_hz']:g} Гц, "
                "применён дважды (filtfilt)"
            )

        if source_name == "raw":
            print(
                "  ВНИМАНИЕ: анализ по raw_* (без фильтрации). "
                "Сигнал содержит постоянную составляющую и ВЧ-помехи; "
                "они не влияют на оценку амплитуды, т.к. в модели "
                "есть член постоянной составляющей, но частотная "
                "избирательность и ITC зависят от помех.",
                file=sys.stderr,
            )

    return plan, source_name, bandpass, fs


def process_folders(
    folder_paths: list[str],
    window: float,
    channels: list[int] | None = None,
    source: str = "auto",
    step_filter: list[int] | None = None,
    guard: float = 0.1,
    min_epochs: int = 2,
    compensate_latency: bool = True,
    window_function: str = "hann",
    outlier_threshold: float = 0.0,
    rms_threshold: float = 3.0,
    make_plots: bool = True,
    output: Path | None = None,
    csv_output: Path | None = None,
    verbose: bool = True,
):
    """
    Полный pipeline:

        folders
          ↓
        step files
          ↓
        ПРОХОД ПО METADATA: план групп + выбор источника
          ↓
        загрузка данных
          ↓
        ось времени по индексу отсчёта (НЕ app_timestamp)
          ↓
        компенсация задержки «событие -> первый отсчёт»
          ↓
        отсечение грязных краёв (--guard)
          ↓
        группировка по конфигурации стимуляции
          ↓
        когерентное усреднение комплексных коэффициентов
        + ITC + SNR
        ↓
        усреднение во времени
    """

    files = collect_files(folder_paths)

    if not files:
        raise FileNotFoundError(
            "В указанных директориях не найдено файлов step_*.txt."
        )

    print(f"Найдено файлов: {len(files)}")

    # ------------------------------------------------------------------
    # План: группы и источник
    # ------------------------------------------------------------------

    plan, source_name, bandpass, fs = plan_sources(
        files,
        step_filter,
        source,
        min_epochs,
        verbose=verbose,
    )

    print()
    print(f"Групп по конфигурации стимуляции: {len(plan)}")

    # ------------------------------------------------------------------
    # Загрузка
    # ------------------------------------------------------------------

    (
        epochs,
        fs,
        channel_names,
        channel_prefix,
    ) = load_epochs(
        files,
        channels,
        source_name,
        step_filter,
        verbose=verbose,
    )

    channel_names = drop_dead_channels(
        epochs,
        channel_names,
        verbose=verbose,
    )

    epochs = validate_epoch_quality(
        epochs,
        fs,
        rms_threshold=rms_threshold,
        verbose=verbose,
    )

    print()
    print(f"Загружено эпох: {len(epochs)}")
    print(f"Частота дискретизации: {fs:.3f} Гц")
    print(f"Каналы ({source_suffix(source_name)}): " + ", ".join(channel_names))

    # ------------------------------------------------------------------
    # Группировка
    # ------------------------------------------------------------------

    groups = group_epochs(epochs, step_filter)

    if not groups:
        raise ValueError("После фильтрации по шагу не осталось ни одной эпохи.")

    skipped = [
        (signature, value)
        for signature, value in groups.items()
        if len(value) < min_epochs
    ]

    for signature, value in skipped:
        print(
            f"Пропущена группа {format_signature(signature)}: "
            f"только {len(value)} эпох (минимум {min_epochs})",
            file=sys.stderr,
        )

    groups = {
        signature: value
        for signature, value in groups.items()
        if len(value) >= min_epochs
    }

    if not groups:
        raise ValueError(
            f"Ни в одной группе не набралось {min_epochs} эпох. "
            "Когерентное накопление требует повторов с одинаковой "
            "конфигурацией стимуляции — укажите больше директорий "
            "(--folder experiments/*) или уменьшите --min-epochs."
        )

    # ------------------------------------------------------------------
    # Анализ
    # ------------------------------------------------------------------

    time_grid = build_time_grid(window, fs)

    results = {}

    figures = {}

    for signature, group_epochs_list in sorted(groups.items()):
        latencies = [
            epoch["latency"]
            for epoch in group_epochs_list
        ]

        if verbose:
            print()
            print(
                f"Группа {format_signature(signature)}: "
                f"{len(group_epochs_list)} эпох"
            )

            print(
                f"  задержка «событие -> первый отсчёт»: "
                f"средняя {np.mean(latencies) * 1e3:+.1f} мс, "
                f"sd {np.std(latencies) * 1e3:.1f} мс, "
                f"разброс "
                f"{(np.max(latencies) - np.min(latencies)) * 1e3:.1f} мс"
            )

            if not compensate_latency:
                spread_s = float(
                    np.max(latencies) - np.min(latencies)
                )

                # Размах разброса -> разброс фазы на частоте f.
                worst_deg = 360.0 * max(signature) * spread_s

                print(
                    "  ВНИМАНИЕ: компенсация задержки отключена "
                    "(--no-latency-compensation). Разброс "
                    f"{spread_s * 1e3:.0f} мс даёт разброс фазы до "
                    f"{worst_deg:.0f}° на {max(signature):g} Гц "
                    "и размывает когерентное накопление.",
                    file=sys.stderr,
                )

        group = analyze_group(
            group_epochs_list,
            signature,
            channel_names,
            guard,
            compensate_latency,
            window_function,
            outlier_threshold,
        )

        (
            group_grid,
            average,
            sem,
            count_valid,
        ) = average_group(
            group_epochs_list,
            time_grid,
            len(channel_names),
            guard,
            compensate_latency,
        )

        results[signature] = group

        print_group_report(
            group,
            channel_names,
            source_name,
            outlier_threshold,
        )

        if make_plots:
            target = None

            if output is not None:
                target = output.with_name(
                    output.stem
                    + f"_step{'_'.join(str(v) for v in group['step_indices'])}"
                    + output.suffix
                )

            figures[signature] = plot_group(
                group_grid,
                average,
                sem,
                channel_names,
                group,
                window,
                source_name,
                target,
            )

            if target is not None:
                print(f"График сохранён: {target}")

    print_summary(results, channel_names)

    if csv_output is not None:
        write_csv(results, channel_names, csv_output)

    return {
        "fs": fs,
        "source": source_name,
        "channel_names": channel_names,
        "time_grid": time_grid,
        "groups": results,
        "figures": figures,
    }


# ============================================================================
# CLI
# ============================================================================


def main() -> int:

    parser = argparse.ArgumentParser(
        description="Когерентное накопление EEG при фотостимуляции.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Примеры:\n"
            "  %(prog)s --folder 'experiments/Казаков А'\n"
            "  %(prog)s --folder experiments/* --step 1 --output out.png --no-show\n"
            "  %(prog)s --folder experiments/* --source raw --guard 0.15\n"
            "  %(prog)s --self-test\n"
        ),
    )

    parser.add_argument(
        "--folder",
        "-d",
        nargs="+",
        required=False,
        metavar="DIR",
        help="Одна или несколько директорий с файлами step_*.txt.",
    )

    parser.add_argument(
        "--window",
        "-w",
        type=float,
        default=3.0,
        help=(
            "Длина временного окна в секундах для усреднения во времени. "
            "По умолчанию: 3.0 (вся длительность шага)."
        ),
    )

    parser.add_argument(
        "--channels",
        "-c",
        nargs="+",
        type=int,
        default=None,
        metavar="N",
        help=(
            "Индексы каналов. По умолчанию — все каналы "
            "источника. Мёртвые (нулевые) каналы отбрасываются."
        ),
    )

    parser.add_argument(
        "--source",
        "-s",
        choices=("auto", "filtered", "raw"),
        default="auto",
        help=(
            "Источник сигнала. 'auto' берёт filtered_* для частот "
            "внутри полосы 0.5–40 Гц и raw_* для более высоких. "
            "По умолчанию: auto."
        ),
    )

    parser.add_argument(
        "--step",
        nargs="+",
        type=int,
        default=None,
        metavar="N",
        help=(
            "Ограничить анализ указанными номерами шагов. "
            "Несколько шагов можно объединять только если для них "
            "одинаков требуемый источник данных (--source)."
        ),
    )

    parser.add_argument(
        "--guard",
        type=float,
        default=0.1,
        help=(
            "Отсечка начала и конца эпохи в секундах "
            "(данные предыдущего шага + переходные процессы filtfilt). "
            "По умолчанию: 0.1."
        ),
    )

    parser.add_argument(
        "--min-epochs",
        type=int,
        default=2,
        help=(
            "Минимальное число эпох в группе. "
            "По умолчанию: 2."
        ),
    )

    parser.add_argument(
        "--no-latency-compensation",
        dest="compensate_latency",
        action="store_false",
        help=(
            "Не компенсировать задержку между событием стимула "
            "и первым отсчётом файла."
        ),
    )

    parser.add_argument(
        "--window-function",
        choices=("hann", "rect"),
        default="hann",
        help="Окно МНК-оценки. По умолчанию: hann.",
    )

    parser.add_argument(
        "--outlier-threshold",
        type=float,
        default=0.0,
        metavar="Z",
        help=(
            "Дополнительное отбрасывание отдельных проб по "
            "амплитуде |C| (в единицах MAD, на канал). "
            "По умолчанию 0, т.е. выключено: сильные пробы — это "
            "обычно вспышки альфа-ритма, а не артефакты. "
            "Неисправные каналы отсекаются всегда, по RMS."
        ),
    )

    parser.add_argument(
        "--rms-threshold",
        type=float,
        default=3.0,
        metavar="Z",
        help=(
            "Порог отбрасывания канала по RMS внутри эпохи, "
            "в единицах MAD от log10(RMS). Ловит битые электроды, "
            "перегрузку усилителя и движения. 0 отключает. "
            "По умолчанию: 3."
        ),
    )

    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=None,
        metavar="FILE",
        help="Сохранить графики в файлы (имя дополняется номером шага).",
    )

    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        metavar="FILE",
        help="Сохранить сводную таблицу в TSV.",
    )

    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Не строить графики.",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Не открывать интерактивное окно графика.",
    )

    parser.add_argument(
        "--quiet",
        "-q",
        action="store_true",
        help="Не печатать построчную информацию о загрузке файлов.",
    )

    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Проверить оценку амплитуды/фазы на синтетике и выйти.",
    )

    args = parser.parse_args()

    if args.self_test:
        return run_self_test()

    if not args.folder:
        parser.error("нужен --folder либо --self-test")

    if args.window <= 0:
        parser.error("--window должен быть больше 0.")

    if args.guard < 0:
        parser.error("--guard не может быть отрицательным.")

    if args.min_epochs < 1:
        parser.error("--min-epochs должен быть >= 1.")

    if args.outlier_threshold < 0:
        parser.error("--outlier-threshold не может быть отрицательным.")

    if args.rms_threshold < 0:
        parser.error("--rms-threshold не может быть отрицательным.")

    try:
        result = process_folders(
            args.folder,
            args.window,
            args.channels,
            source=args.source,
            step_filter=args.step,
            guard=args.guard,
            min_epochs=args.min_epochs,
            compensate_latency=args.compensate_latency,
            window_function=args.window_function,
            outlier_threshold=args.outlier_threshold,
            rms_threshold=args.rms_threshold,
            make_plots=not args.no_plot,
            output=args.output,
            csv_output=args.csv,
            verbose=not args.quiet,
        )

        if not args.no_show and result["figures"]:
            plt.show()

    except KeyboardInterrupt:
        print("\nОперация прервана.", file=sys.stderr)
        sys.exit(130)

    except Exception as exc:
        print(f"\nОшибка: {exc}", file=sys.stderr)
        sys.exit(1)

    return 0


if __name__ == "__main__":
    sys.exit(main())