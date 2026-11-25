"""Модель одного шага последовательности стимуляции."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SequenceStep:
    dac0_f1: float
    dac0_f2: float
    dac1_f1: float
    dac1_f2: float
    seconds: float
