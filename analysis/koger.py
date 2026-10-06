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

# Минимальное число отсчётов, при котором оценка амплитуды имеет
# смысл. Ниже МНК на 3 неизвестных (DC + cos + sin) вырождается.
MIN_EPOCH_SAMPLES = 20

# Если после отсечения осталось меньше этой доли шага,
# выводится предупреждение: на шаге 3 с при guard = 1.4 с
# остаётся 63 отсчёта, то есть 4.7% записи, и без
# предупреждения это выглядит как полноценный анализ.
MIN_WINDOW_FRACTION_WARN = 0.25

# Минимальный множитель RMS относительно медианы канала, при
# котором (эпоха, канал) считается аномалией. Не даёт порогу
# «схлопнуться» на каналах с малым разбросом RMS.
MIN_RMS_RATIO = 3.0

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


def parse_metadata(meta_part: str) -> dict:
    """
    Разбирает секцию `# METADATA`.
    """

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

    return metadata


def load_metadata(
    file_path: Path,
) -> dict:
    """
    Читает ТОЛЬКО metadata файла.

    Дешёвый проход: план групп и выбор источника строятся по
    metadata, а разбирать таблицу `# DATA` (это 3-12 с на 210
    файлов) для этого не нужно.
    """

    content = file_path.read_text(encoding="utf-8")

    if "# DATA" not in content:
        raise ValueError(
            f"В файле {file_path} отсутствует секция '# DATA'"
        )

    meta_part, _ = content.split("# DATA", 1)

    return parse_metadata(meta_part)


def parse_data_table(
    data_part: str,
    file_path: Path,
) -> pd.DataFrame:
    """
    Разбирает таблицу `# DATA`.

    Сначала C-движок (15.7 мс/файл), при ошибке — python-движок
    (59.2 мс/файл), который терпимее к кривым строкам.
    """

    payload = data_part.strip()

    if not payload:
        raise ValueError(f"После '# DATA' нет данных: {file_path}")

    try:
        return pd.read_csv(
            io.StringIO(payload),
            sep="\t",
        )
    except pd.errors.ParserError:
        return pd.read_csv(
            io.StringIO(payload),
            sep="\t",
            engine="python",
            on_bad_lines="warn",
        )


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

    content = file_path.read_text(encoding="utf-8")

    if "# DATA" not in content:
        raise ValueError(
            f"В файле {file_path} отсутствует секция '# DATA'"
        )

    meta_part, data_part = content.split("# DATA", 1)

    return (
        parse_metadata(meta_part),
        parse_data_table(data_part, file_path),
    )


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
    df: pd.DataFrame | None = None,
) -> float:
    """
    Определяет номинальную частоту дискретизации.

    ВАЖНО:
        В секции `# METADATA` ключа `lsl_nominal_srate_hz` НЕТ.
        Он есть только:
            * внутри `lsl_stream_meta_json` -> `nominal_srate_hz`;
            * в таблице `# DATA` (столбец `lsl_nominal_srate_hz`).

    Источники проверяются в этом порядке. `df` может быть None —
    тогда третий источник недоступен, но первые два покрывают
    все реальные файлы.
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
    if df is not None and "lsl_nominal_srate_hz" in df.columns:
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
    приемлемо. Измерено на этих данных (fs = 500 Гц, 0.5-40 Гц,
    Баттерворт 4 порядка, два прохода):
         4 Гц ->  0.0 дБ     22 Гц ->  0.1 дБ
        12 Гц ->  0.0 дБ     25 Гц ->  0.2 дБ
        14 Гц ->  0.0 дБ     29 Гц ->  0.5 дБ
                              32 Гц ->  1.2 дБ
                              34 Гц ->  1.9 дБ
                              55 Гц -> 24.4 дБ
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
    Находит step-файлы во всех переданных директориях, включая
    вложенные.

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

        # Рекурсивно: структура `experiments/<испытуемый>/*.txt`
        # очень распространена, и молчаливый ноль найденных файлов
        # при указании на `experiments` сбивал с толку.
        for file_path in folder.rglob("*"):
            if file_path.is_file() and STEP_FILE_PATTERN.match(
                file_path.name
            ):
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
    Возвращает каналы выбранного источника данных.

    Выбор между `filtered_*` и `raw_*` делает `plan_sources`
    (по частотам стимуляции), здесь — только проверка наличия
    колонок. `auto` сюда не доходит.
    """

    prefix = f"{source}_"

    columns = find_signal_columns(df, prefix)

    if not columns:
        raise ValueError(f"В файле нет каналов {prefix}*")

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
            # .copy() ОБЯЗАТЕЛЕН: под copy-on-write (pandas 3.x)
            # to_numpy() возвращает представление только для
            # чтения, и последующая запись NaN в
            # `validate_epoch_quality` падала бы с
            # "assignment destination is read-only".
            # Без --channels этого не проявлялось только потому,
            # что мёртвые каналы заставляли drop_dead_channels
            # пересобрать массив.
            .copy()
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
       `robust_mask_log`). Так отсеиваются битые электроды и
       перегрузки усилителя.

      В этих данных: у испытуемого Чаркин `filtered_ch_1`
      имеет std = 488 ед. и |x|max = 5096, тогда как у соседних
      каналов того же файла std = 5–22 ед. При отключённой
      этой проверке (--rms-threshold 0) обычное среднее коэффициентов
      на 10 Гц по 28 эпохам даёт |C| = 5.36 — практически целиком
      вклад ОДНОГО испытуемого (его |C| = 148 при медиане 0.16 по
      группе). Медиана даёт 0.043. С проверкой по умолчанию этот
      испытуемый отбрасывается раньше, и среднее даёт 0.083.

    Проверка RMS, а не |C|: RMS не зависит от фазы, поэтому
    отсеивает именно неисправность канала, а не сильную
    фотостимуляцию или вспышку альфа-ритма.

    Порог строгий намеренно. База (медиана и MAD) считается
    отдельно для каждого шага: разные шаги имеют разные частоты
    и разный фоновый уровень. Типичный порог по MAD получается
    10-180x медианы, так что движение и открытые глаза
    (RMS 30-100 при типичном 25-43) остаются нетронутыми, а
    отсекаются только поломки (RMS 300-7000).
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
    #
    # База (медиана и MAD) считается ОТДЕЛЬНО для каждого шага:
    # разные шаги имеют разные частоты стимуляции и разный
    # фоновый уровень, и общий MAD смешивал бы их. Проверено:
    # при объединении шагов 6 и 7 порог по DAC0 уходил на 50
    # отбрасываний вместо 24 + 26 по отдельности.

    n_channels = kept[0]["data"].shape[1]

    if rms_threshold <= 0:
        return kept

    by_step = defaultdict(list)

    for row, epoch in enumerate(kept):
        by_step[epoch["step"]["step_index"]].append(row)

    masked = []

    for step_index, rows in sorted(by_step.items()):
        rows = sorted(rows)

        block = np.full(
            (len(rows), n_channels),
            np.nan,
            dtype=float,
        )

        for position, row in enumerate(rows):
            for channel in range(n_channels):
                column = kept[row]["data"][:, channel]

                finite = column[np.isfinite(column)]

                if finite.size:
                    block[position, channel] = float(
                        np.sqrt(np.mean(finite ** 2))
                    )

        for channel in range(n_channels):
            column = block[:, channel]

            mask = robust_mask_log(column, rms_threshold)

            bad = np.flatnonzero(np.isfinite(column) & ~mask)

            if bad.size == 0:
                continue

            reference = float(np.nanmedian(column))

            for position in bad:
                row = rows[position]

                value = block[position, channel]

                masked.append(
                    f"  шаг {step_index}, канал {channel + 1}: "
                    f"RMS {value:.4g} ({value / reference:.0f}x от "
                    f"{reference:.4g}) — "
                    f"{kept[row]['file'].parent.name}/"
                    f"{kept[row]['file'].name}"
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
            f"  (порог: RMS > median * max({MIN_RMS_RATIO:g}, "
            f"10^({rms_threshold:g}*MAD) от log10 RMS), база по шагам; "
            "ловит битые электроды и перегрузку, не трогая движение "
            "и альфа)",
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
    формально попадают в 0.5–40 Гц, но спад Баттерворта 4 порядка,
    применённый дважды, даёт на 34 Гц -2.3 дБ, а на 55 Гц -23.6 дБ.
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


def attenuated_frequencies(
    signature: tuple,
    bandpass: dict,
    fs: float,
) -> list:
    """
    Стимулируемые частоты, которые `filtered_*` гасит
    сильнее `MAX_ATTENUATION_DB`.
    """

    return [
        frequency
        for frequency in signature
        if (
            bandpass_attenuation_db(frequency, bandpass, fs)
            > MAX_ATTENUATION_DB
        )
    ]


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


def epoch_time_offset(
    epoch: dict,
    compensate_latency: bool,
) -> float:
    """
    Сдвиг оси времени эпохи относительно момента переключения
    частот на Arduino.

    Отсчёты приходят с шагом ровно 1/fs, поэтому ось эпохи — это
    равномерная сетка `n/fs`, сдвинутая на `latency`:

        t_n = n / fs + offset

    где `offset = latency` при компенсации и 0 без неё.
    """

    if compensate_latency:
        return float(epoch["latency"])

    return 0.0


def epoch_time_bounds(
    epoch: dict,
    guard: float,
    compensate_latency: bool,
) -> tuple[float, float, float]:
    """
    Границы участка эпохи, пригодного для анализа.

    Возвращает:

        offset : сдвиг сетки `n/fs`
        lower  : нижняя граница времени
        upper  : верхняя граница времени

    Что исключается:

      * начало файла содержит данные, записанные ещё на частотах
        ПРЕДЫДУЩЕГО шага (шаг помечается на PC в момент приёма
        пачки, а не в момент acquisition);
      * `sosfiltfilt` / `filtfilt` дают переходные процессы
        длиной padlen (~27 отсчётов, ~54 мс) на обоих краях
        каждого файла, а вызванная реакция максимальна в начале;
      * всё, что позже конца шага `export_step_duration_s`.

    Компенсация задержки сдвигает ось так, чтобы `t = 0`
    соответствовало моменту переключения частот. Событие
    `stim_events_json` фиксируется сразу после подтверждения
    команды, т.е. с точностью до времени обмена по UART (~1-2 мс),
    что на порядок лучше разброса 17-53 мс между шагами.
    """

    offset = epoch_time_offset(epoch, compensate_latency)

    fs = epoch["fs"]

    n_samples = epoch["data"].shape[0]

    last = (n_samples - 1) / fs + offset

    duration = epoch["step"].get("duration") or 0.0

    if duration > 0:
        last = duration if duration < last else last

    return (
        offset,
        float(offset + guard),
        float(last - guard),
    )


def trim_epoch(
    epoch: dict,
    guard: float,
    compensate_latency: bool,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Отбрасывает «грязные» края эпохи и строит ось времени.
    """

    time = epoch["time"] + epoch_time_offset(epoch, compensate_latency)

    data = epoch["data"]

    _, lower, upper = epoch_time_bounds(
        epoch,
        guard,
        compensate_latency,
    )

    keep = (
        (time >= lower)
        & (time <= upper)
        & np.isfinite(time)
    )

    if keep.sum() < MIN_EPOCH_SAMPLES:
        # Запрошенный guard не помещается в эпоху.
        half = guard / 2.0

        print(
            f"ВНИМАНИЕ: guard={guard:g} с почти равен длине шага "
            f"({epoch['file'].name}); guard уменьшен до {half:g} с",
            file=sys.stderr,
        )

        keep = (
            (time >= lower - half)
            & (time <= upper + half)
            & np.isfinite(time)
        )

    if keep.sum() < MIN_EPOCH_SAMPLES:
        print(
            f"ВНИМАНИЕ: после отсечения в {epoch['file'].name} "
            "осталось меньше "
            f"{MIN_EPOCH_SAMPLES} отсчётов; отбрасываем эпоху",
            file=sys.stderr,
        )

        return (
            np.empty(0),
            np.empty((0, data.shape[1])),
        )

    # Слишком короткое окно анализа стоит отметить: при guard
    # чуть меньше половины шага `trim_epoch` молча оставляет
    # несколько процентов записи (guard = 1.4 с на шаге 3 с ->
    # 63 отсчёта, то есть 4.7% длительности, БЕЗ предупреждения).
    span = float(time[keep][-1] - time[keep][0])

    duration = float(epoch["step"].get("duration") or 0.0)

    if duration > 0 and span < MIN_WINDOW_FRACTION_WARN * duration:
        print(
            f"ВНИМАНИЕ: в {epoch['file'].name} осталось только "
            f"{span:.3f} с из {duration:.3f} с "
            f"({100 * span / duration:.0f}%). Проверьте --guard.",
            file=sys.stderr,
        )

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

    Пробы с ровно нулевой амплитудой ИСКЛЮЧАЮТСЯ, а не считаются
    за фазовые 0: у нулевой величины фазы нет.
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

    return float(
        np.abs(
            np.mean(
                coefficients[nonzero] / magnitudes[nonzero]
            )
        )
    )


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


def _nan_median_or_nan(values: np.ndarray) -> float:
    """
    Медиана без предупреждения о пустом срезе.
    """

    values = np.asarray(values, dtype=float)

    values = values[np.isfinite(values)]

    if values.size == 0:
        return float("nan")

    return float(np.median(values))


# Относительная точность, ниже которой отклонение считается нулём.
EPSILON = 1e-9


def mad_scale(
    deviations: np.ndarray,
    level: float = 1.0,
) -> float:
    """
    Масштаб устойчивой оценки разброса.

        MAD-sigma = 1.4826 * median(|v - median(v)|)

    ВАЖНО, что происходит при MAD = 0 (больше половины значений
    совпадают, типично для «9 значений 5.0 и одно 1e6»).
    Ни минимальный, ни максимальный непустой разброс здесь не
    годятся:

      * максимальный = собственный разброс выброса, и выброс
        задаёт себе порог, из-за чего ВСЕГДА проходит проверку;
      * минимальный непустой = тот же самый разброс, если
        неповторяющихся значений ровно одно.

    Поэтому при MAD = 0 масштаб берётся от МАСШТАБА САМИХ
    ЗНАЧЕНИЙ: `level * EPSILON`, где `level` — их медиана.
    Численно это соответствует различию на последних разрядах
    double, то есть любой реальный выброс отсекается.

    `level` = 0 или NaN (все значения нулевые) даёт абсолютный
    минимум `np.finfo(float).tiny`.
    """

    if deviations.size == 0:
        return np.finfo(float).tiny

    median = float(np.median(deviations))

    scale = 1.4826 * median

    if scale > 0:
        return scale

    if np.isfinite(level):
        scale = abs(float(level)) * EPSILON

        if scale > 0:
            return scale

    return np.finfo(float).tiny


def robust_mask(
    values: np.ndarray,
    threshold: float,
) -> np.ndarray:
    """
    Маска выбросов по медиане и MAD.

    MAD используется вместо стандартного отклонения, потому что
    сам выброс раздувает sigma и маскирует себя.

    `threshold` = 0 отключает проверку.
    """

    finite = np.isfinite(values)

    if not finite.any() or threshold <= 0:
        return finite

    clean = values[finite]

    median = float(np.median(clean))

    scale = mad_scale(
        np.abs(clean - median),
        level=median,
    )

    return finite & (np.abs(values - median) <= threshold * scale)


def robust_mask_log(
    values: np.ndarray,
    threshold: float,
    min_ratio: float = 3.0,
) -> np.ndarray:
    """
    Маска выбросов по медиане и MAD в логарифмическом масштабе.

    RMS распределён сильно правоскошено: типичное значение
    10–20 ед., а у артефактов 100–7000 ед. На линейной шкале MAD
    растягивается выбросами.

    В логарифмическом масштабе распределение становится
    приблизительно симметричным, но ТОЛЬКО при условии, что порог
    не «схлопывается» на каналах с малым разбросом:

        эффективный множитель = 10 ** (threshold * MAD_log10)

    У `filtered_ch_8` MAD_log10 = 0.088, поэтому при threshold = 3
    множитель равен всего 1.83, и обычная 1.5-кратная вариация
    RMS удалялась бы как «аномалия».

    Поэтому итоговый порог — МАКСИМУМ из двух ОТНОСИТЕЛЬНЫХ оценок:

        factor = max(min_ratio, 10 ** (threshold * MAD_log10))

    и отбрасывается проба, только если RMS превышает
    `median * factor`, т.е. она заведомо велика по обоим критериям
    сразу. Пересечение критериев (AND на отбрасывание) давало бы
    54 отбрасывания из 224 на шаге 1 вместо 12 и вырезало бы пробы
    с сильным альфа-ритмом — то есть удаляло бы именно исследуемый
    сигнал.

    Типичный порог по MAD получается 9-39x медианы. На каналах с
    малым MAD_log10 (порог по MAD < 3x) срабатывает floor
    `min_ratio`, и там отсекается всё выше 3x от медианы: на шаге 1
    это канал 8 (RMS 5.5, порог 16.5), где отсекаются в том числе
    пробы с RMS 17.9-25.5.
    """

    finite = np.isfinite(values) & (values > 0)

    if not finite.any() or threshold <= 0:
        return finite

    log_values = np.full(values.shape, np.nan, dtype=float)

    log_values[finite] = np.log10(values[finite])

    clean = log_values[finite]

    median = float(np.median(clean))

    linear_median = float(np.median(values[finite]))

    # При MAD_log = 0 масштаб берётся от масштаба самих значений
    # (`level * EPSILON` в ЛИНЕЙНЫХ единицах), а не от log10:
    # иначе 14.0 и 2000.0 отличаются на 2.15 десятичных порядка,
    # и порог схлопывается до 10^(3*2.15) = 2.9e6 от медианы.
    deviation = mad_scale(
        np.abs(clean - median),
        level=linear_median,
    )

    # Оба порога — ОТНОСИТЕЛЬНЫ к медиане, иначе на канале с
    # малым разбросом min_ratio сравнивался бы с абсолютной
    # величиной и не срабатывал бы.
    factor = max(
        float(min_ratio),
        10.0 ** (threshold * deviation),
    )

    return finite & (values <= linear_median * factor)


def robust_coherent_average(
    coefficients: np.ndarray,
) -> complex:
    """
    Устойчивое когерентное среднее: покоординатная медиана.

    В отличие от `coherent_average` (среднего), медиана не даёт
    одной амплитудной аномалии перевесить остальные.

    В этих данных это критично: у испытуемого Чаркин канал
    `filtered_ch_1` имеет RMS 488 ед. против 3–37 у остальных
    каналов, и его |C| на 10 Гц равен 148 при медиане 0.16 по
    группе из 28 эпох. Обычное среднее даёт по этому каналу
    |C| = 5.36 на 10 Гц, то есть результат одного испытуемого из
    28; медиана даёт 0.043 на 10 Гц и 0.070 на 8 Гц, то есть
    результат всех остальных.

    Медиана нужна и там, где отсечения по RMS не было: на шаге 7
    из `raw_*` среднее по 30 эпохам даёт 1.08 при медиане 0.0051,
    потому что одна проба имеет |C| = 22.4.

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

    # ------------------------------------------------------------------
    # Отсечение выполняется ОДИН раз на эпоху и переиспользуется
    # для всех четырёх частот: входные данные у них одинаковы.
    # ------------------------------------------------------------------

    prepared = []

    for epoch in epochs:
        time, data = trim_epoch(
            epoch,
            guard,
            compensate_latency,
        )

        if len(time) < MIN_EPOCH_SAMPLES:
            prepared.append((np.empty(0), data[:0, :]))

            continue

        prepared.append((time, data))

    # (n_epochs, n_channels) для каждой частоты
    coefficients = {}

    for index, name in enumerate(DAC_FREQUENCY_KEYS):
        frequency = signature[index]

        table = np.full(
            (n_epochs, n_channels),
            complex(np.nan, np.nan),
            dtype=complex,
        )

        for epoch_index, (time, data) in enumerate(prepared):
            if len(time) == 0:
                continue

            table[epoch_index, :] = complex_amplitudes(
                time,
                data,
                frequency,
                window_function,
            )

        finite = (
            np.isfinite(table.real)
            & np.isfinite(table.imag)
        )

        # --------------------------------------------------------------
        # Маска принятых проб
        # --------------------------------------------------------------
        #
        # ПокАНАЛЬНО: один битый канал у одного испытуемого
        # не должен выбрасывать его эпоху целиком из анализа
        # по остальным каналам.
        #
        # Уже включает пробы, помеченные NaN в
        # `validate_epoch_quality` (битые электроды).

        accepted = finite.copy()

        if outlier_threshold > 0:
            for channel_index in range(n_channels):
                amplitudes = np.abs(table[:, channel_index])

                accepted[:, channel_index] &= robust_mask(
                    amplitudes,
                    outlier_threshold,
                )

        n_accepted = np.sum(accepted, axis=0)

        mean_values = np.array(
            [
                coherent_average(table[accepted[:, column], column])
                for column in range(n_channels)
            ]
        )

        robust_values = np.array(
            [
                robust_coherent_average(
                    table[accepted[:, column], column]
                )
                for column in range(n_channels)
            ]
        )

        coefficients[name] = {
            "frequency": frequency,
            "table": table,
            "accepted": accepted,
            "n_accepted": n_accepted,
            "coherent": mean_values,
            "robust": robust_values,
            "amplitude": np.abs(robust_values),
            "phase": np.angle(robust_values, deg=True),
            "mean_amplitude": np.abs(mean_values),
            "mean_phase": np.angle(mean_values, deg=True),
            # Медиана |C_i| по принятым пробам: типичная (не
            # когерентная) амплитуда одиночной пробы на этой частоте.
            "median_amplitude": np.array(
                [
                    _nan_median_or_nan(
                        np.abs(table[accepted[:, column], column])
                    )
                    for column in range(n_channels)
                ]
            ),
            "itc": np.array(
                [
                    inter_trial_coherence(
                        table[accepted[:, column], column]
                    )
                    for column in range(n_channels)
                ]
            ),
            "awc": np.array(
                [
                    amplitude_weighted_coherence(
                        table[accepted[:, column], column]
                    )
                    for column in range(n_channels)
                ]
            ),
            "snr": np.array(
                [
                    coherent_snr(table[accepted[:, column], column])
                    for column in range(n_channels)
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
    t_start: float,
    t_stop: float,
    fs: float,
) -> np.ndarray:
    """
    Равномерная сетка времени с шагом РОВНО `1/fs`.

    Время отсчитывается от момента переключения частот на Arduino
    (событие `stim_events_json`).

    Шаг `1/fs` важен: при `np.linspace(a, b, ceil(W*fs)+1)` шаг
    отличается от `1/fs` (до ~0.8%), что на 55 Гц даёт ошибку
    фазы в несколько градусов.
    """

    if t_stop <= t_start:
        raise ValueError(
            f"Пустой интервал анализа: [{t_start:g}, {t_stop:g}] с."
        )

    # FLOOR, а не round: при round последняя точка сетки могла
    # выйти за t_stop на выборку (t = 2.8509 при t_stop = 2.8500),
    # и на ней count_valid = 0, average = NaN.
    n_points = int(np.floor((t_stop - t_start) * fs)) + 1

    if n_points < 3:
        raise ValueError(
            f"Интервал {t_stop - t_start:g} с слишком короткий "
            f"для частоты дискретизации {fs:g} Гц."
        )

    return t_start + np.arange(n_points) / fs


def resample_epoch(
    epoch: dict,
    time_grid: np.ndarray,
    guard: float,
    compensate_latency: bool,
) -> np.ndarray:
    """
    Точное вынесение эпохи на абсолютную сетку времени.

    ИНТЕРПОЛЯЦИИ НЕТ и она не нужна: ось эпохи `t = n/fs + offset`
    уже равномерна с шагом 1/fs, поэтому отсчёту сетки
    `time_grid[k]` соответствует ровно один индекс

        n_k = round((time_grid[k] - offset) * fs)

    Так каждая эпоха попадает на сетку по СВОЕМУ времени, а не по
    индексу: разброс задержки между шагами (до 45 мс, т.е. до 23
    отсчётов) компенсируется, иначе усреднение размывалось бы на
    2.5 оборота при 55 Гц.

    Вне сетки времени и в границах `epoch_time_bounds` и `guard`
    возвращается NaN — такие точки не усредняются.

    Интерполяции нет и она не нужна: ось эпохи уже равномерна,
    отсчёту сетки соответствует ровно один индекс. Проверено на
    данных: вынесенные значения побитово совпадают с отсчётами
    (максимальное отклонение 0.000e+00 на шагах 1, 3, 6, 7).

    ОДНОКО: округление до целого индекса вносит до половины
    отсчёта (1 мс) остаточного разброса между эпохами, если
    начало сетки не кратно шагу с учётом задержки. На 55 Гц это
    до ~40° фазы, на 12 Гц до ~9°. Частотная оценка этого не
    касается: `complex_amplitudes` работает на точной оси
    `n/fs + latency` каждой эпохи и округления не использует.
    """

    data = epoch["data"]

    fs = epoch["fs"]

    offset, lower, upper = epoch_time_bounds(
        epoch,
        guard,
        compensate_latency,
    )

    indices = np.rint((time_grid - offset) * fs).astype(np.int64)

    n_samples = data.shape[0]

    valid = (
        (indices >= 0)
        & (indices < n_samples)
        & (time_grid >= lower)
        & (time_grid <= upper)
    )

    values = np.full(
        (len(time_grid), data.shape[1]),
        np.nan,
        dtype=float,
    )

    if valid.any():
        values[valid, :] = data[indices[valid], :]

    return values


def trimmed_time_average(
    stacked: np.ndarray,
    count_valid: np.ndarray,
    fraction: float,
) -> np.ndarray:
    """
    Усечённое среднее по оси эпох, покАНАЛЬНО.

    Обычное среднее по 30 эпохам определяется двумя-тремя пробами
    с артефактами: на шаге 3 график усреднения во времени
    показывал красивую 12 Гц-волну на `filtered_ch_1`, в то время
    как ITC на этой частоте и канале равнялся 0.17, то есть ниже
    шумового уровня 0.183. Частотная часть считалась устойчиво
    (медиана), а график — нет, и они противоречили друг другу.

    Отбрасывается `fraction` самых больших и столько же самых
    маленьких значений в каждой точке времени. Медиана не подходит:
    усреднённый сигнал должен остаться непрерывным.

    `stacked` имеет форму (n_epochs, n_time, n_channels); NaN
    сортируются numpy в конец, поэтому срез по верхней границе
    заведомо не содержит пропусков.

    Число отбрасываемых проб берётся от МИНИМАЛЬНОГО числа
    валидных проб по времени внутри канала: иначе в точках, где
    часть эпох дала NaN, в срез попали бы пропуски.
    """

    total = stacked.shape[0]

    keep = int(np.floor(total * fraction))

    if total - 2 * keep < 3:
        keep = max(0, (total - 3) // 2)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)

        if keep <= 0:
            return np.nanmean(stacked, axis=0)

        ordered = np.sort(stacked, axis=0)

        result = np.full(
            (stacked.shape[1], stacked.shape[2]),
            np.nan,
            dtype=float,
        )

        # Минимальное число валидных проб по времени, на канал.
        minimum = np.min(count_valid, axis=0)

        for channel in range(stacked.shape[2]):
            high = int(minimum[channel]) - keep

            if high <= keep:
                result[:, channel] = np.nanmean(
                    stacked[:, :, channel],
                    axis=0,
                )

                continue

            result[:, channel] = np.mean(
                ordered[keep:high, :, channel],
                axis=0,
            )

    return result


def average_group(
    epochs: list[dict],
    time_grid: np.ndarray,
    guard: float,
    compensate_latency: bool,
    trim_fraction: float = 0.1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Усреднение во времени внутри группы.

    Эпохи выносятся на ОБЩУЮ абсолютную сетку времени, поэтому
    усреднение идёт по одному и тому же моменту времени, а не по
    одному и тому же индексу отсчёта: разброс задержки между
    шагами (до 45 мс = 23 отсчёта) компенсируется, иначе волна
    размывалась бы на 2.5 оборота при 55 Гц.

    Усреднение — УСЕЧЁННОЕ (см. `trimmed_time_average`), чтобы
    график и частотные числа не противоречили друг другу.

    Возвращает `time_grid`, `average`, `sem`, `count_valid`.
    Формы всех четырёх совпадают.
    """

    blocks = [
        resample_epoch(
            epoch,
            time_grid,
            guard,
            compensate_latency,
        )
        for epoch in epochs
    ]

    usable = [
        block
        for block in blocks
        if np.isfinite(block).any()
    ]

    if not usable:
        raise ValueError(
            "После отсечения не осталось ни одного отсчёта. "
            "Уменьшите --guard."
        )

    stacked = np.stack(usable, axis=0)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)

        count_valid = np.sum(
            np.isfinite(stacked),
            axis=0,
        )

        average = trimmed_time_average(
            stacked,
            count_valid,
            trim_fraction,
        )

        std = np.nanstd(stacked, axis=0, ddof=1)

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
# Plotting
# ============================================================================


def plot_group(
    time_grid: np.ndarray,
    average: np.ndarray,
    sem: np.ndarray,
    channel_names: list[str],
    group: dict,
    source: str,
    output: Path | None = None,
):
    """
    График группы: усреднение во времени + амплитуда + ITC.
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

    ax.set_xlabel(
        "Время относительно переключения частот на Arduino, с"
    )
    ax.set_ylabel("Амплитуда, ед. ADC")
    ax.set_title(
        f"{group['label']} — {group['n_epochs']} эпох, "
        f"источник {source}_*, "
        f"t ∈ [{time_grid[0]:.3f}, {time_grid[-1]:.3f}] с"
    )
    ax.legend(fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Когерентная амплитуда
    # ------------------------------------------------------------------

    names = list(DAC_FREQUENCY_KEYS)

    frequencies = [group["coefficients"][key]["frequency"] for key in names]

    labels = [
        f"{DAC_LABELS[name]}\n{frequencies[index]:g} Гц"
        for index, name in enumerate(names)
    ]

    positions = np.arange(len(names))

    width = 0.8 / max(len(channel_names), 1)

    centres = positions + width * (len(channel_names) - 1) / 2

    ax = axes[1]

    for index, name in enumerate(channel_names):
        ax.bar(
            positions + index * width,
            [
                group["coefficients"][key]["amplitude"][index]
                for key in names
            ],
            width=width,
            label=name,
        )

    ax.set_xticks(centres)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("|C|, ед. ADC")
    ax.set_title("Когерентная амплитуда (медиана по пробам)")
    ax.legend(fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3, axis="y")

    # ------------------------------------------------------------------
    # ITC
    # ------------------------------------------------------------------

    ax = axes[2]

    for index, name in enumerate(channel_names):
        ax.bar(
            positions + index * width,
            [
                group["coefficients"][key]["itc"][index]
                for key in names
            ],
            width=width,
            label=name,
        )

    # Уровень шума при N эпохах: 1/sqrt(N).
    noise_itc = 1.0 / np.sqrt(group["n_epochs"])

    ax.axhline(
        noise_itc,
        linestyle=":",
        linewidth=1.0,
        color="black",
        label=f"шум ({noise_itc:.3f})",
    )

    ax.set_xticks(centres)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("ITC")
    ax.set_ylim(0, 1)
    ax.set_title("Inter-trial coherence (фазовая согласованность)")
    ax.legend(fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()

    if output is not None:
        fig.savefig(output, dpi=150, bbox_inches="tight")

    return fig


# ============================================================================
# Reporting
# ============================================================================


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
                            f"{result['amplitude'][index]:.6g}",
                            f"{result['phase'][index]:.4f}",
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


def print_group_report(
    group: dict,
    channel_names: list[str],
    source: str,
    outlier_threshold: float,
    bandpass: dict,
    fs: float,
):
    """
    Печатает результаты одной группы.

    `|C|` — устойчивая оценка (медиана по принятым пробам).
    `|C| mean` — обычное среднее: оно показывает, насколько
    результат зависит от отдельных амплитудных аномалий.
    `n` — число реально использованных проб на этом канале.
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

    latencies = group["latencies"]

    print(
        f"Задержка «событие -> первый отсчёт»: "
        f"{np.mean(latencies) * 1e3:+.1f} мс "
        f"(sd {np.std(latencies) * 1e3:.1f}, "
        f"разброс {(np.max(latencies) - np.min(latencies)) * 1e3:.1f})"
    )

    # ------------------------------------------------------------------
    # Что сделал фильтр с частотой стимуляции
    # ------------------------------------------------------------------

    lost = attenuated_frequencies(group["signature"], bandpass, fs)

    if source == "filtered":
        print(
            "Затухание в фильтре: "
            + format_attenuations(group["signature"], bandpass, fs)
        )

        # Выбор filtered_* здесь задан пользователем принудительно,
        # поэтому потери НЕ предупреждались: на шаге 7 амплитуда
        # 55 Гц занижалась в 21 раз молча.
        if lost:
            print(
                "ВНИМАНИЕ: filtered_* гасит "
                + ", ".join(f"{value:g} Гц" for value in lost)
                + f" сильнее {MAX_ATTENUATION_DB:g} дБ, поэтому "
                "амплитуда на этих частотах занижена. "
                "Для корректных значений используйте --source raw.",
                file=sys.stderr,
            )

    elif lost:
        print(
            "Источник raw_* выбран, потому что filtered_* гасит "
            + ", ".join(f"{value:g} Гц" for value in lost)
            + f" сильнее {MAX_ATTENUATION_DB:g} дБ.",
            file=sys.stderr,
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
        # Выбросы
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
                f"  Отброшено по {name}: {int(rejected.sum())} из "
                f"{len(rejected)} (порог MAD x{outlier_threshold:g} "
                "по |C|, либо NaN канала)"
            )
            print(f"    испытуемые: " + ", ".join(names))
            print(
                f"    их |C|: "
                + ", ".join(
                    f"{value:.3g}"
                    for value in amplitudes[rejected]
                )
                + "; у принятых медиана "
                f"{result['median_amplitude'][index]:.3g}"
            )

    # ------------------------------------------------------------------
    # Согласованность с уровнем шума
    # ------------------------------------------------------------------

    n_epochs = group["n_epochs"]

    noise_itc = 1.0 / np.sqrt(n_epochs)

    weak = {}

    for key in DAC_FREQUENCY_KEYS:
        result = group["coefficients"][key]

        names = []

        for index, name in enumerate(channel_names):
            itc = result["itc"][index]

            if not np.isfinite(itc) or itc > noise_itc * 1.5:
                continue

            names.append(name)

        if names:
            weak[f"{result['frequency']:g} Гц"] = names

    if weak:
        print()
        print(
            f"  ВНИМАНИЕ: ITC не превышает шумовой уровень "
            f"{noise_itc:.3f} ({n_epochs} эпох):"
        )

        for frequency, names in weak.items():
            print(
                f"    {frequency}: "
                + ", ".join(names)
                + f"  ({len(names)} из {len(channel_names)} каналов)"
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
    Сводка по группам.

    Строки ключуются КОНКРЕТНОЙ ЧАСТОТОЙ, а не именем слота DAC:
    усреднять 12 Гц (шаг 3) с 55 Гц (шаг 7) в один столбец
    «DAC0 f1» бессмысленно. В шаге 1 f1 == f2, поэтому без
    агрегации по частоте строки дублировались бы.
    """

    if not groups:
        return

    rows = {}

    for signature, group in groups.items():
        for name in DAC_FREQUENCY_KEYS:
            result = group["coefficients"][name]

            frequency = result["frequency"]

            entry = rows.setdefault(
                frequency,
                {
                    "frequency": frequency,
                    "dacs": set(),
                    "steps": set(),
                    "n_epochs": group["n_epochs"],
                    "amplitude": result["amplitude"],
                    "itc": result["itc"],
                },
            )

            entry["dacs"].add(DAC_LABELS[name].split()[0])

            entry["steps"].update(group["step_indices"])

    print()
    print("=" * 100)
    print("СВОДКА ПО ЧАСТОТАМ")
    print("=" * 100)

    header = (
        f"{'Частота':<10}{'DAC':<12}{'Шаги':<10}{'n':>4}"
    )

    header += "".join(
        f"{name:>16}" for name in channel_names
    )

    print(header)
    print("-" * len(header))

    for frequency in sorted(rows):
        row = rows[frequency]

        line = (
            f"{f'{frequency:g} Гц':<10}"
            f"{'/'.join(sorted(row['dacs'])):<12}"
            f"{','.join(str(value) for value in sorted(row['steps'])):<10}"
            f"{row['n_epochs']:>4}"
        )

        for index in range(len(channel_names)):
            amplitude = row["amplitude"][index]

            itc = row["itc"][index]

            if not np.isfinite(amplitude) or not np.isfinite(itc):
                line += f"{'н/д':>16}"

                continue

            line += f"{amplitude:>10.3g} I{itc:>4.2f}"

        print(line)

    print()
    print(
        "Формат ячейки:  амплитуда |C| и ITC. "
        "Сравнивать амплитуды можно только в строке одной частоты."
    )


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
    print("6) Маски выбросов")
    print("-" * 86)

    # При MAD = 0 нельзя брать максимальный выброс как масштаб:
    # тогда единственный выброс сам себе задаёт порог и проходит.
    spikes = np.array([5.0] * 9 + [1e6])

    kept = int(np.sum(robust_mask(spikes, 4.0)))

    ok = kept == 9

    failures += 0 if ok else 1

    print(
        f"   9 значений 5.0 и одно 1e6 (MAD = 0): "
        f"оставлено {kept} из 10  ->  {'OK' if ok else 'ПРОВАЛ'}"
    )

    # Нормальный разброс не должен попадать под порог.
    smooth = 20.0 * (1.0 + 0.01 * np.arange(30))

    kept_smooth = int(np.sum(robust_mask_log(smooth, 3.0)))

    ok = kept_smooth == 30

    failures += 0 if ok else 1

    print(
        f"   плавный ряд 20.0…20.29 (разброс 1%): "
        f"оставлено {kept_smooth} из 30  ->  {'OK' if ok else 'ПРОВАЛ'}"
    )

    # На канале с малым разбросом порог не должен схлопываться.
    tight = 5.0 * (1.0 + 0.05 * np.array([-1.0, 1.0] + [0.0] * 28))

    kept_tight = int(np.sum(robust_mask_log(tight, 3.0)))

    ok = kept_tight == 30

    failures += 0 if ok else 1

    print(
        f"   узкий канал ±5% (MAD_log10 мал, порог схлопывается): "
        f"оставлено {kept_tight} из 30  ->  {'OK' if ok else 'ПРОВАЛ'}"
    )

    # Явный мусор должен отсекаться.
    noisy = np.array([14.0] * 28 + [2000.0])

    kept_noisy = int(np.sum(robust_mask_log(noisy, 3.0)))

    ok = kept_noisy == 28

    failures += 0 if ok else 1

    print(
        f"   28 значений 14.0 и одно 2000.0: "
        f"оставлено {kept_noisy} из 29  ->  {'OK' if ok else 'ПРОВАЛ'}"
    )

    print()
    print("7) Ось времени: независимость от сетки и корректность поворота")
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
            metadata = load_metadata(file_path)

            step = extract_step_info(metadata)

            candidate_fs = resolve_sampling_rate(
                metadata,
                None,
            )

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
    #
    # Считаются ФАЙЛЫ (`value["count"]`), а не номера шагов:
    # один и тот же номер шага у разных испытуемых — разные эпохи.
    plan = {
        signature: value
        for signature, value in plan.items()
        if value["count"] >= min_epochs
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
    t_start: float | None = None,
    t_stop: float | None = None,
    channels: list[int] | None = None,
    source: str = "auto",
    step_filter: list[int] | None = None,
    guard: float = 0.15,
    min_epochs: int = 2,
    compensate_latency: bool = True,
    window_function: str = "hann",
    outlier_threshold: float = 0.0,
    trim_fraction: float = 0.1,
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
          + компенсация задержки «событие -> первый отсчёт»
          ↓
        отбрасывание битых каналов (RMS) и обрывов записи
          ↓
        группировка по конфигурации стимуляции
          ↓
        когерентное усреднение комплексных коэффициентов
        + ITC + SNR
          ↓
        усреднение во времени на общей абсолютной сетке
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

        group["source"] = source_name

        # Сетка времени строится из СОБСТВЕННЫХ границ каждой группы,
        # а не одна на весь запуск: у разных шагов разные
        # длительности и разные задержки.
        #
        # Берётся ПЕРЕСЕЧЕНИЕ допустимых интервалов всех эпох
        # группы, чтобы на краях сетки не было участков без данных
        # (иначе count_valid = 0, average = NaN, и график начинался
        # бы пустым отрезком).
        bounds = [
            epoch_time_bounds(epoch, guard, compensate_latency)
            for epoch in group_epochs_list
        ]

        group_start = max(item[1] for item in bounds)

        group_stop = min(item[2] for item in bounds)

        if t_start is not None:
            group_start = max(group_start, t_start)

        if t_stop is not None:
            group_stop = min(group_stop, t_stop)

        if group_stop <= group_start:
            raise ValueError(
                f"Интервал анализа пуст: доступно "
                f"[{max(i[1] for i in bounds):.3f}, "
                f"{min(i[2] for i in bounds):.3f}] с "
                f"(запрошено [{t_start}, {t_stop}]). "
                "Уменьшите --guard."
            )

        group_grid = build_time_grid(group_start, group_stop, fs)

        (
            group_grid,
            average,
            sem,
            count_valid,
        ) = average_group(
            group_epochs_list,
            group_grid,
            guard,
            compensate_latency,
            trim_fraction,
        )

        group["time_grid"] = group_grid
        group["average"] = average
        group["sem"] = sem
        group["count_valid"] = count_valid

        results[signature] = group

        print_group_report(
            group,
            channel_names,
            source_name,
            outlier_threshold,
            bandpass,
            fs,
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
        "bandpass": bandpass,
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
        "--range",
        nargs=2,
        type=float,
        default=None,
        metavar=("T0", "T1"),
        help=(
            "Интервал усреднения во времени в секундах "
            "относительно переключения частот. "
            "По умолчанию: [0, конец шага]."
        ),
    )

    parser.add_argument(
        "--window",
        "-w",
        type=float,
        default=None,
        help=(
            "Длина интервала от t=0 в секундах; сокращение для "
            "--range 0 W."
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
            "Индексы каналов, НУМЕРАЦИЯ С НУЛЯ: -c 0 - это первый "
            "канал источника (ch_1). По умолчанию — все каналы, "
            "кроме нулевых (мёртвые отбрасываются автоматически)."
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
        default=0.15,
        help=(
            "Отсечка начала и конца эпохи в секундах, "
            "отсчитываемая от переключения частот на Arduino. "
            "Убирает данные, записанные ещё на частотах "
            "предыдущего шага, и переходные процессы filtfilt "
            "(~54 мс). По умолчанию: 0.15."
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
        "--trim",
        type=float,
        default=0.1,
        metavar="F",
        help=(
            "Доля отбрасываемых проб с каждой стороны при усреднении "
            "во времени (усечённое среднее). Без этого график "
            "определяется 2-3 артефактными пробами. "
            "0 отключает усечение. По умолчанию: 0.1."
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

    if args.range is not None and args.window is not None:
        parser.error("задайте либо --range, либо --window, не оба.")

    if args.range is not None:
        t_start, t_stop = args.range

    elif args.window is not None:
        t_start, t_stop = 0.0, args.window

    else:
        t_start, t_stop = None, None

    if t_start is not None and t_start < 0:
        parser.error("--range: начало не может быть отрицательным.")

    if t_stop is not None and t_stop <= 0:
        parser.error("--range: конец должен быть больше 0.")

    if args.guard < 0:
        parser.error("--guard не может быть отрицательным.")

    if args.min_epochs < 1:
        parser.error("--min-epochs должен быть >= 1.")

    if args.outlier_threshold < 0:
        parser.error("--outlier-threshold не может быть отрицательным.")

    if not 0 <= args.trim < 0.5:
        parser.error("--trim должен быть в [0, 0.5).")

    if args.rms_threshold < 0:
        parser.error("--rms-threshold не может быть отрицательным.")

    try:
        result = process_folders(
            args.folder,
            t_start,
            t_stop,
            args.channels,
            source=args.source,
            step_filter=args.step,
            guard=args.guard,
            min_epochs=args.min_epochs,
            compensate_latency=args.compensate_latency,
            window_function=args.window_function,
            outlier_threshold=args.outlier_threshold,
            trim_fraction=args.trim,
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