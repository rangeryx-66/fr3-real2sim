"""Post-selection reporting; never influences a controller or fitted parameter."""
import csv,json,time
from pathlib import Path
import numpy as np

def read(p):return json.loads(Path(p).read_text())
def finalize(e):
    from interactive_twin_conditional.workflow import summarize
    from interactive_twin_refinement.assembly import compare
    allrows=[];failures=[];invariance=[];episodes={}
    for folder in e.out.iterdir():
        if not folder.is_dir() or not folder.name.startswith(('dev_','test_')):continue
        rows=summarize(e,folder.name);allrows.extend([{'episode':folder.name,**r} for r in rows]);episodes[folder.name]=read(folder/'summary.json')
        if episodes[folder.name].get('stop'):
            failures.append({'episode':folder.name,'path':str(folder/'stop.json'),'preserved':True,**episodes[folder.name]['stop']})
        ref=folder/'reference/original/native_authored_assembly_private.json'
        if ref.exists():
            base=read(ref)
            for p in folder.glob('conditional_grid/*/native_authored_assembly_private.json'):
                invariance.append({'episode':folder.name,'candidate':p.parent.name,**compare(base,read(p))})
        for p in folder.glob('**/report.json'):
            r=read(p)
            if r['status'] not in ('SUCCESS','PHYSICS_PROTOCOL_COMPLETE','REPLAY_COMPLETE'):
                failures.append({'episode':folder.name,'path':str(p),'status':r['status'],'phase':r.get('failure_phase'),'preserved':True})
        selection=folder/'selection_frozen.json'
        if selection.exists():
            import matplotlib;matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            for condition,s in read(selection)['conditions'].items():
                versions={}
                for name,label in [('T0','fixed_refined_wrong_prior'),('T1','uncertain_structure_same_prior'),('T2','uncertain_structure_calibrated')]:
                    chosen=s.get('methods',{}).get(label)
                    path=folder/'updated_twins'/condition/label/'twin_versions.json'
                    if chosen and path.exists():
                        native=read(path)['versions']['T1']
                        versions[name]={'comparison':label,'native_asset':native['asset_root'],'urdf':native['urdf'],'effective_parameters':{k:chosen[k] for k in ('tau_c','b')},'structure_id':chosen['structure_id'],'parameter_intervals':s['parameter_intervals'],'identifiability':s['status'],'prediction_selection_only':True,'full_task_validated_separately':True}
                dest=folder/'updated_twins'/condition;dest.mkdir(parents=True,exist_ok=True)
                (dest/'conditional_twins.json').write_text(json.dumps({'versions':versions,'T0_scope':'fixed refined structure and wrong physics prior; not an oracle or raw visual prior','parameter_recovery_claimed':False,'baseline_replaced':False},indent=2))
                losses=s.get('candidate_losses',[])
                if not losses:continue
                structures=sorted({r['structure_id'] for r in losses});tau=e.c['physics_grid']['tau_c'];b=e.c['physics_grid']['b']
                fig,axs=plt.subplots(1,len(structures),figsize=(3.2*len(structures),3.3),squeeze=False)
                for ax,structure in zip(axs[0],structures):
                    matrix=np.full((len(tau),len(b)),np.nan)
                    for r in losses:
                        if r['structure_id']==structure:matrix[tau.index(r['tau_c']),b.index(r['b'])]=np.sqrt(r['train_loss'])
                    im=ax.imshow(matrix,origin='lower',aspect='auto');ax.set_xticks(range(len(b)),b);ax.set_yticks(range(len(tau)),tau);ax.set_xlabel('b (effective sim)');ax.set_ylabel('tau_c (effective sim)');ax.set_title(structure);fig.colorbar(im,ax=ax)
                fig.suptitle(condition+' / training normalized RMS; test excluded');fig.tight_layout();fig.savefig(folder/f'physics_profiles_{condition}.png');plt.close(fig)
    if allrows:
        with (e.out/'comparison.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(allrows[0]),lineterminator='\n');w.writeheader();w.writerows(allrows)
    result={'mode':'SIM_TO_SIM_BLIND_SYSID','conditional_A_distinct_from_full_task_B':True,'episodes':episodes,'failures':failures,'initial_assembly_consistent':bool(invariance and all(r['passed'] for r in invariance)),'assembly_comparisons':invariance,'no_baseline_promoted':True,'hardware_friction_measurement':False,'old_12_scene_outcomes_retained':True,'created_unix_s':time.time()}
    (e.out/'summary.json').write_text(json.dumps(result,indent=2))
    write_report(e,allrows,result)
    return result


def write_report(e, rows, result):
    """Keep the two experiments and action initialization semantics visible."""
    def value(x, digits=4):
        return '—' if x is None else f'{x:.{digits}f}'
    labels={'fixed_refined_wrong_prior':'固定精化结构＋错误 prior',
            'uncertain_structure_same_prior':'结构候选＋相同 prior',
            'uncertain_structure_calibrated':'结构候选＋校准物理'}
    lines=['# 结构不确定性下的条件动力学校准', '',
      '**SIM_TO_SIM_BLIND_SYSID；条件预测 A 和完整任务 B 分开验收。**', '',
      '本轮不修改成功抓取、接触判据、proxy、base、控制器、安全上限或 mobile recovery。', '',
      '## 主表', '',
      'P4 使用预先冻结的双脉冲波形。主表为从相同抓持快照单独运行 P4 的结果；前序 P1–P3 自主演化后的 P4 结果另行保留。RMSE 是逐坐标 RMS。', '',
      '|条件|模型|同初态 P4 RMSE mm|增量 RMSE mm|停驱动漂移 mm|漂移误差 mm|tau_c / b|完整任务 B|实测开门 °|',
      '|---|---|---:|---:|---:|---:|---|---|---:|']
    for r in rows:
        task=('成功' if r['full_task_success'] else r['full_task_status'])
        lines.append('|'+ '|'.join([r['condition'],labels.get(r['method'],r['method']),value(r.get('common_start_RMSE_mm')),value(r.get('common_start_relative_RMSE_mm')),value(r.get('common_start_held_drift_mm')),value(r.get('common_start_held_drift_error_mm')),f"{r['tau_c']} / {r['b']}",task,value(r['full_task_angle_deg'],3)])+'|')
    lines += ['', '## 可辨识性与限制', '',
      'tau_c / b 是所测试网格中由训练与验证选出的有效模拟参数，不等于精确测量的真实铰链摩擦。',
      '停驱动片段仍保持真实抓持及机器人阻尼/重力补偿，不是完全自由的门。接触冲量没有复制；每次仿真仅开始时初始化，随后由物理自主演化。', '']
    for r in rows:
        if r['method']=='uncertain_structure_calibrated':
            lines.append(f"- {r['condition']}: {r['identifiability']}; unresolved tested ranges = {r['parameter_intervals']}. 这些范围不是统计置信区间。")
    lines += ['', '## 前序动作累计误差诊断', '',
      '|条件|模型|P1–P3 后 P4 绝对 RMSE mm|同初态 P4 最终位移误差 mm|启动误差 s|速度 RMSE mm/s|',
      '|---|---|---:|---:|---:|---:|']
    for r in rows:
        lines.append('|'+ '|'.join([r['condition'],labels.get(r['method'],r['method']),value(r['heldout_RMSE_mm']),value(r.get('common_start_final_displacement_error_mm')),value(r.get('common_start_delay_error_s')),value(r.get('common_start_velocity_RMSE_mm_s'))])+'|')
    lines += ['', '同初态 P4 是模型选择冻结后的初始化复核；没有利用它重新选择结构、物理参数或阈值。前序累计状态误差减小不能独自证明单动作动力学预测改善。', '',
      '## 完整任务与失败', '',
      '完整任务保持原始 home/warmup → approach → closure → manipulation。逐模型保留失败，未用 A 的初始化绕过 B。', '']
    for r in rows:
        lines.append(f"- {r['condition']} / {labels.get(r['method'],r['method'])}: {r['full_task_status']}; bilateral={r['full_task_grasp']}; min margin={value(r['full_task_min_margin_rad'])} rad; final slip={value(None if r['full_task_final_slip_m'] is None else r['full_task_final_slip_m']*1000)} mm.")
    for name,episode in result['episodes'].items():
        if episode.get('stop'):
            lines += ['', f"### {name} 保留的停止结果", '', json.dumps(episode['stop'],ensure_ascii=False)]
    lines += ['', f"初始 COM / 惯量装配一致性：{result['initial_assembly_consistent']}，共 {len(result['assembly_comparisons'])} 个候选检查。", '',
      '数据：comparison.csv、summary.json、每个 episode 下的 selection_frozen.json、common_start_heldout/results.json、updated_twins/*/conditional_twins.json 与连续 contact_baseline.mp4。', '',
      '单一复现命令见 docs/interactive_twin_conditional.md。']
    (e.out/'REPORT.md').write_text('\n'.join(lines)+'\n')
