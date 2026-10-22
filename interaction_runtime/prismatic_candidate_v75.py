"""Frozen ranking with whole declared motion-chain feasibility and fallback."""
import json,hashlib
from agent_command_protocol_v12 import atomic_json
from .prismatic_planning_v73 import PrismaticCandidateCheck
from .prismatic_chain_v75 import evaluate_chain


class PrismaticCandidateCheckV75(PrismaticCandidateCheck):
    def __call__(self,candidate):
        result=super().__call__(candidate)
        if result['valid']:
            geometry=self.g['a'].output/'agent_geometry/observed_geometry.json'
            data=json.loads(geometry.read_text())
            context=dict(self.g,normal=data['normal_world'])
            chain=evaluate_chain(context,candidate,self.g.get('v75_declared_chain'),self.policy,self.config,self.solver)
            chain['observed_normal_source_sha256']=hashlib.sha256(geometry.read_bytes()).hexdigest()
            result['v75_whole_chain']=chain
            if not chain['valid']:result.update(valid=False,reason=chain['reason'])
            self.audit[-1].update(result)
            atomic_json(self.g['a'].output/'EXECUTION_CANDIDATE_AUDIT.json',dict(candidates=self.audit,
                weights_changed=False,whole_declared_chain_checked=True,
                native_use='collision safety only, never observed contact scoring'))
        return result
