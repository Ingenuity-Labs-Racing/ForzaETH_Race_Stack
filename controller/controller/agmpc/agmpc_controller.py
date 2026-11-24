"""Geometric tracker; shared reference mechanics live in core.reference."""
from core.reference import PredictiveTracker, PredictiveResult, validate_config
from .solver import AGMPCSolver


class AGMPCController(PredictiveTracker):
    def __init__(self, config, solver_factory=AGMPCSolver):
        super().__init__(config, solver_factory)
