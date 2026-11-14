"""AGMPC registry adapter."""
from core.controller_interface import PredictiveAdapter
from .agmpc_controller import AGMPCController, validate_config


def load_config(document):
    validate_config(document)
    return dict(document)


class AGMPCAdapter(PredictiveAdapter):
    def __init__(self, config, context):
        super().__init__(AGMPCController(config))
