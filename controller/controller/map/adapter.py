"""MAP pursuit adapter."""
from controller.pursuit import PursuitAdapter
from .controller import MAP_Controller


class MAPAdapter(PursuitAdapter):
    def create_engine(self):
        return MAP_Controller(**self.engine_parameters(), loop_rate=self.context.rate,
                              LUT_name=self.context.lookup_table,
                              logger_info=self.context.logger_info, logger_warn=self.context.logger_warn)
