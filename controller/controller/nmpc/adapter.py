"""Nonlinear MPC registry adapter."""
from core.controller_interface import PredictiveAdapter
from .nmpc_controller import NMPCController, validate_config


def load_config(document):
    validate_config(document)
    return dict(document)


class NMPCAdapter(PredictiveAdapter):
    def __init__(self, config, context):
        super().__init__(NMPCController(config))
