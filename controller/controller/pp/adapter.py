"""Pure-pursuit adapter."""
from controller.pursuit import PursuitAdapter
from .controller import PP_Controller


class PPAdapter(PursuitAdapter):
    def create_engine(self):
        return PP_Controller(**self.engine_parameters(), loop_rate=self.context.rate,
                             wheelbase=self.context.wheelbase,
                             logger_info=self.context.logger_info, logger_warn=self.context.logger_warn)
