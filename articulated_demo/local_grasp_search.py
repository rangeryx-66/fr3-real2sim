"""Joint feasibility gates for bounded handle poses; no opening arc search."""
import json
import numpy as np

from .backend import Failure
from .preflight import probe_ik
from .redundant_path import values,with_q,margins
from .isolation_diagnostics import checked_command,run_contact_model_test
from r1a7_backend import JOINTS


def validate_trajectory(node,start,trajectory):
    names=list(trajectory.joint_trajectory.joint_names);previous=values(start);minimum=1.
    for point in trajectory.joint_trajectory.points:
        endpoint=np.array([point.positions[names.index(n)] for n in JOINTS])
        for q in np.linspace(previous,endpoint,max(2,int(np.ceil(np.max(np.abs(endpoint-previous))/.01))+1)):
            detail=margins(node,q);minimum=min(minimum,detail['all_joint_min_rad'])
            if minimum<=.05:raise Failure('LOW_JOINT_MARGIN','trajectory outside unchanged 0.05 rad gate')
            node.validate(with_q(start,q))
        previous=endpoint
    return minimum


def run_search(node,items,output,report,save):
    home=node.measured()
    search={'rows':[],'required_load_distance_m':None,'closure_diagnostic_only':True,'selected':None,
            'ik_random_seeds':8,'ik_timeout_s':.25,'minimum_joint_margin_rad':.05}
    report['local_search']=search
    def checkpoint():
        (output/'local_search.json').write_text(json.dumps(search,indent=2,default=str));save()
    for item,T in items:
        row={'variant':item['variant'],'T_B_TCP':T.tolist()};search['rows'].append(row)
        try:
            solutions=[]
            outcome,grasp=probe_ik(node,T,home,random_seeds=8,timeout_s=.25,solution_sink=solutions)
            safe=[state for state in solutions if margins(node,values(state))['all_joint_min_rad']>.05]
            if safe and (grasp is None or margins(node,values(grasp))['all_joint_min_rad']<=.05):
                grasp=max(safe,key=lambda state:margins(node,values(state))['all_joint_min_rad'])
            row['grasp_ik']=outcome
            if grasp is None:raise Failure('NO_IK' if not outcome['kinematic_ik'] else 'COLLISION','grasp exact IK gate')
            row['grasp_margin']=margins(node,values(grasp))
            if row['grasp_margin']['all_joint_min_rad']<=.05:raise Failure('LOW_JOINT_MARGIN','grasp gate')
            # Closure endpoint cannot be inferred from the point cloud.
            # Plan the OPEN approach first; actual closure is the next gate.
            row['estimated_contact_width_m']=item.get('closure',{}).get('estimated_contact_width_m')
            row['closure_endpoint_source']='actual Isaac closure'
            prestate=None;row['pregrasp_trials']=[]
            for retreat in (.08,.06,.04,.02):
                pre=T.copy();pre[:3,3]-=retreat*T[:3,2]
                pre_solutions=[]
                outcome,trial=probe_ik(node,pre,grasp,random_seeds=8,timeout_s=.25,solution_sink=pre_solutions)
                detail={'retreat_m':retreat,'ik':outcome,'plan_trials':[]};row['pregrasp_trials'].append(detail)
                safe_pre=[state for state in pre_solutions if margins(node,values(state))['all_joint_min_rad']>.05]
                safe_pre.sort(key=lambda state:np.linalg.norm(values(state)-values(grasp)))
                unique=[]
                for candidate in safe_pre:
                    if not any(np.linalg.norm(values(candidate)-values(old))<.01 for old in unique):unique.append(candidate)
                for trial in unique[:3]:
                    try:
                        approach=node.cartesian(trial,T)
                        detail['approach_margin']=validate_trajectory(node,trial,approach)
                        plan=node.plan(home,trial)
                        detail['preplan_margin']=validate_trajectory(node,home,plan)
                        prestate=trial;row['pregrasp_retreat_m']=retreat;break
                    except Failure as error:detail['plan_trials'].append({'failure':error.category,'detail':str(error)})
                if prestate is not None:break
            if prestate is None:raise Failure('NO_PREGRASP_PLAN','no safe approach on the checked retreat line')
            row['status']='OPEN_APPROACH_PLANNED_CLOSURE_UNVERIFIED';checkpoint()
            # Diagnose real closure before authorizing any loading.
            run_contact_model_test(node,T,output,report,save,load_limit_m=0.,
                                   initial_margin_gate=.05,use_cartesian=True,keep_locked=True,
                                   pregrasp_retreat_m=row['pregrasp_retreat_m'],
                                   prepared_plan={'grasp':grasp,'prestate':prestate,'approach':approach,'preplan':plan})
            physical=report['isolation']['isolated_grasp'];row['physical_probe']=physical
            row['actual_aperture_error_m']=(physical.get('actual_aperture_m',0)-row['estimated_contact_width_m']) if row['estimated_contact_width_m'] is not None else None
            checkpoint()
            row['status']='ACTUAL_CLOSURE_PADS_ONLY' if physical['status']=='NO_FAILURE_WITHIN_TEST_RANGE' else 'PHYSICAL_'+physical['status']
            report['status']='CLOSURE_DIAGNOSTIC_COMPLETE'
            # No load path is accepted until checked from the measured endpoint.
            row['actual_load_path_checked']=False
            checkpoint();return
        except Failure as error:
            row['status']=error.category;row['detail']=str(error);checkpoint()
        print(item['variant'],row['status'],flush=True)
    report['status']='NO_LOCAL_FEASIBLE_GRASP';checkpoint()
