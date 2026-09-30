"""Joint feasibility gates for bounded handle poses; no opening arc search."""
import copy
import json
import numpy as np

from .backend import Failure
from .preflight import probe_ik,trajectory_end
from .redundant_path import values,with_q,margins
from .isolation_diagnostics import checked_command,run_contact_model_test
from r1a7_backend import JOINTS
from r1a7_calibration import width_to_finger_q
import r1a7_plant as plant


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
    audit=checked_command({'op':'diagnostic_audit'});home=node.measured()
    search={'rows':[],'required_load_distance_m':.002,'selected':None,
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
            # Validate the official finger closure against the full door mesh.
            closed=copy.deepcopy(grasp)
            index=[i for i,n in enumerate(closed.joint_state.name) if n in ('dex1_Joint1_1','dex1_Joint2_1')]
            for width in item['closure']['width_samples_m']:
                q=width_to_finger_q(float(width))
                for i in index:closed.joint_state.position[i]=float(q)
                node.validate(closed)
            row['closure_moveit_valid']=True
            tangent=np.cross(np.array(audit['hinge_axis_world']),T[:3,3]-np.array(audit['hinge_origin_world']))
            tangent/=np.linalg.norm(tangent)
            current=closed;load_rows=[];row['load_path']=load_rows
            for distance in np.arange(.00025,.00201,.00025):
                target=T.copy();target[:3,3]+=distance*tangent
                # Existing Cartesian planner preserves the previous state;
                # no solver, seed policy or planner parameter is changed.
                trajectory=node.cartesian(current,target)
                margin=validate_trajectory(node,current,trajectory)
                load_rows.append({'distance_m':float(distance),'margin_rad':margin})
                current=trajectory_end(current,trajectory)
            row['load_path_collision_free']=True
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
            row['status']='ALL_PREFLIGHT_GATES_PASSED';checkpoint()
            # Real closure and 2 mm loading must pass before a capacity sweep.
            run_contact_model_test(node,T,output,report,save,load_limit_m=.002,
                                   initial_margin_gate=.05,use_cartesian=True,keep_locked=True,
                                   pregrasp_retreat_m=row['pregrasp_retreat_m'],
                                   prepared_plan={'grasp':grasp,'prestate':prestate,'approach':approach,'preplan':plan})
            physical=report['isolation']['isolated_grasp'];row['physical_probe']=physical
            actual=np.dot(np.array(plant.state()['tcp'])-np.array(physical.get('reference_T_B_TCP',T))[:3,3],
                          physical.get('tangent_world',[0.,0.,0.]))
            row['actual_loaded_tangential_displacement_m']=float(actual)
            if physical['status']=='NO_FAILURE_WITHIN_TEST_RANGE' and actual>=.001:
                search['selected']=item;checkpoint()
                # Continue the same locked grasp; no regrasp or parameter tuning.
                from .isolation_diagnostics import continue_tangential_capacity
                report['capacity_test']=continue_tangential_capacity(node,physical,output)
                report['status']='LOCAL_GRASP_SEARCH_FOUND';checkpoint();return
            if physical['status']=='NO_FAILURE_WITHIN_TEST_RANGE':
                checked_command({'op':'diagnostic_lock_door','locked':False})
                physical['status']='INSUFFICIENT_ACTUAL_LOAD_MOTION'
            row['status']='PHYSICAL_'+physical['status']
            # A failed real grasp is not followed by another automated grasp
            # without restoring home and opening the hand under collision checks.
            report['status']='LOCAL_SEARCH_PHYSICAL_BLOCKER';checkpoint();return
        except Failure as error:
            row['status']=error.category;row['detail']=str(error);checkpoint()
        print(item['variant'],row['status'],flush=True)
    report['status']='NO_LOCAL_FEASIBLE_GRASP';checkpoint()
