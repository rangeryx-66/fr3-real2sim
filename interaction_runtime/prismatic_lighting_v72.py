"""V7.1 manipulation unchanged; explicit versioned render-only lighting."""
import json
from .prismatic_benchmark_v71 import RegisteredPrismaticRuntimeV71
from .sensor_lighting_v1 import install


class LitRegisteredPrismaticRuntimeV72(RegisteredPrismaticRuntimeV71):
    def __init__(self,job):
        super().__init__(job)
        audit=install(self.scene['stage'],self.root/'configs/sensor_lighting_v1.json')
        audit.update(manipulation_runtime='RegisteredPrismaticRuntimeV71 unchanged',
            observation_distribution_changed=True,previous_dark_results_preserved=True)
        (self.output/'LIGHTING_PROVENANCE.json').write_text(json.dumps(audit,indent=2))
