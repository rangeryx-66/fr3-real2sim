"""Summarize measured recovery/decomposition results without launching rollouts."""
import argparse
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text()) if path.exists() else None


def number(value, scale=1., precision=3):
    return '—' if value is None else f'{value*scale:.{precision}f}'


def summarize(root):
    mobile = read(root/'mobile_summary.json')
    physics = read(root/'physics_oracle/physics_decomposition.json')
    gate = read(root/'physics_oracle/stop_gate.json')
    prior = read(root/'prior_comparison.json')
    lines = ['# PiPER mobile recovery / physics decomposition', '',
             '**SIM_TO_SIM_BLIND_SYSID**；未验证真机 Real2Sim。平台为 kinematic SE(2)，未验证轮式导航动力学。', '']
    if mobile:
        rows = mobile['rows']
        successful_assets = sorted({r['asset'] for r in rows if r['mobile_5deg_success']})
        lines += ['## 1. 冻结 unseen 场景', '',
                  f"完成 {mobile['completed']}/12；固定底座成功 {mobile['fixed_5deg_success']}/12；"
                  f"含底盘恢复成功 {mobile['mobile_5deg_success']}/12；新增成功 {mobile['recovered_manipulation']}。", '',
                  f"成功涉及 {len(successful_assets)}/4 个资产：{', '.join(successful_assets) or '无'}。所有失败保留在分母中。", '',
                  '| 配置 | fixed IK/path | mobile IK/path | 真实抓持 | ID | ≥5° | 最小 margin (rad) | 终止状态 |',
                  '|---|---:|---:|---:|---:|---:|---:|---|']
        for row in rows:
            lines.append('| '+ ' | '.join([row['config'],
                *[str(int(bool(row[k]))) for k in ('fixed_grasp_feasible','mobile_grasp_feasible',
                   'mobile_grasp_success','mobile_ID_success','mobile_5deg_success')],
                number(row.get('minimum_joint_margin_rad')),row['failure_reason']])+' |')
        lines += ['', 'IK/path 仅表示静态规划可行；不能替代真实抓持或开门。详细底盘位姿与行程见 `cross_object_mobile.csv`。', '']
    else:
        lines += ['Mobile：尚无汇总结果。', '']
    lines += ['## 2. DEV 7320 physics 分解', '']
    if physics and physics.get('groups'):
        lines += ['| 组 | 状态 | train EE RMSE (mm) | held-out EE RMSE (mm) | onset error (s) | velocity RMSE (mm/s) | final displacement error (mm) | tau_c / b |',
                  '|---|---|---:|---:|---:|---:|---:|---|']
        for label in ('P0','P1_wrong_prior','P1','P2','P3'):
            row = physics['groups'].get(label, {'status':'NOT_RUN'})
            h = row.get('heldout',{}).get('metrics',{})
            lines.append('| '+' | '.join([label,row['status'],
                number(row.get('train',{}).get('ee_position_rmse_m'),1000),
                number(h.get('ee_position_rmse_m'),1000),number(h.get('start_time_error_s')),
                number(h.get('ee_velocity_rmse_m_s'),1000),number(h.get('final_displacement_error_m'),1000),
                number(row.get('tau_c'))+' / '+number(row.get('b'))])+' |')
        groups = physics['groups']
        def rmse(label):
            return groups.get(label,{}).get('heldout',{}).get('metrics',{}).get('ee_position_rmse_m')
        p1, wrong, p2, p3 = [rmse(label) for label in ('P1','P1_wrong_prior','P2','P3')]
        lines += ['']
        if p1 is not None and wrong is not None:
            lines.append('P1 held-out '+('改善' if p1 < wrong else '**未改善**')+'错误 physics prior。')
        if None not in (p1,p2,p3):
            lines.append(f"纯 estimated-kinematics 组 P2 的误差为 {number(p2,1000)} mm，完整 P3 为 {number(p3,1000)} mm；"
                         f"隔离 physics 的 P1 为 {number(p1,1000)} mm。")
        lines += ['', 'tau_c / b 是 effective simulator resistance parameters；不是经真机 torque 标定的铰链摩擦。', '']
    else:
        lines += ['尚无完整分解结果；不把此前 T0 接近 GT 的结果称为 Real2Sim prior 优势。', '']
    if gate:
        lines += [f"Physics stop gate：`{gate['status']}`；45621 optimizer resumed：`{gate.get('45621_optimizer_resumed',False)}`。", '']
    lines += ['## 3. Imperfect visual prior', '']
    if prior:
        lines += [f"状态：`{prior['status']}`。", '']
        for label in ('T_prior','T_updated'):
            if prior.get(label):
                lines.append(f"- {label} held-out EE RMSE：{number(prior[label]['metrics']['ee_position_rmse_m'],1000)} mm。")
        if 'heldout_improved' in prior:
            lines += ['', f"更新后 held-out 改善：`{prior['heldout_improved']}`。"]
    else:
        lines.append('尚未执行。')
    lines += ['', '## 4. 证据边界', '',
              '- f5dcc6f / cd61660 抓取、接触、collision、proxy、力和 0.05 rad 安全余量保持不变。',
              '- P0/P2 的 GT 仅用于诊断环境构建；拟合不接收 GT，held-out 不参与参数选择。',
              '- 旧训练数据复用带 provenance，不称为新执行。',
              '- 当前接触安全 supervisor 含 PhysX 信号；未证明仅依靠原厂 PiPER SDK 即可完整迁移。',
              '- 事后 final relative slip 不是全程最大滑移；未证明完整在线 slip observability。',
              '- approximate interaction proxy 未证明与真实几何完全一致。', '']
    (root/'report.md').write_text('\n'.join(lines))
    return root/'report.md'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    print(summarize(parser.parse_args().output))
