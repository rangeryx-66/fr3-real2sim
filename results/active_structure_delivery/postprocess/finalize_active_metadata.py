"""Post-run bookkeeping; no simulator/control imports or fitting."""
from pathlib import Path
import json,csv,datetime
root=Path('/data1/home/rangeryx/fr3_real2sim_piper_mobile/results/active_structure_20261005_v1')
def read(p):return json.loads(p.read_text())
attempts=[]
for p in sorted(root.glob('episodes/*/candidate_*/report.json')):
 r=read(p);s=r.get('first_failure_state') or {}
 attempts.append({'episode':p.parent.parent.name,'candidate':p.parent.name,'status':r['status'],'failure_phase':r.get('failure_phase'),'bilateral_hold':r.get('bilateral_hold_established',False),'min_margin_rad':r.get('minimum_joint_margin_rad'),'stop_force_n':json.dumps(s.get('forces_n')),'stop_aperture_m':s.get('aperture_m'),'report':str(p.relative_to(root))})
with (root/'candidate_attempts.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(attempts[0]));w.writeheader();w.writerows(attempts)
episodes=list(csv.DictReader((root/'per_episode.csv').open()))
typed=[r for r in episodes if r['joint_type'] in ('revolute','prismatic')]
counts={'bilateral_grasps':sum(r['bilateral_grasp']=='True' for r in episodes),'posthoc_type_correct':sum(r['joint_type_correct']=='True' for r in typed),'posthoc_type_evaluated':len(typed),'completed_candidates':len(attempts),'candidate_collision_stops':sum('COLLISION' in r['status'] for r in attempts),'closed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'no_new_simulations_after_primary_batch':True,'physics_fitting':False}
(root/'completion_metadata.json').write_text(json.dumps(counts,indent=2))
report=root/'REPORT.md'
prefix='''# 本轮结论

**已恢复短弧 discovery → bounded refinement，但尚未证明 fresh TEST 上的结构重建泛化。**

冻结的新集合为 6 个 PhysX-Mobility 资产 × 2 个配置，共 12 场景。代码/config/预算在 TEST 前冻结，旧暴露资产没有进入主分母。

- Deployment：4/12；fixed-base 失败中的 mobile recovery 为 0/8。
- 真实双侧抓持：3 个配置；抓持并获得 ≥5 mm 有效运动：2/4 reachable。
- Discovery：2/2 useful interaction 进入 PROVISIONAL；均来自 45134。
- 最终 refinement acceptance：0/2；端到端：0/12；至少一个/两个配置成功的资产均为 0/6。
- GT 事后类型正确率：2/2 已产生可辨识运动的轨迹；不代表最终结构验收成功。
- 7320、45621 原成功回归：2/2，通过约 5.62° / 5.58° 实际开门。

## 首个阻塞

1. 8 个配置未通过 fixed + SE(2) 部署；其中两台洗碗机在 setup settle 阶段已被动打开约 39–41°，没有用 GT reset 或增大阻力压住运动。
2. 47419：一个配置稳定双侧抓持失败；另一个配置成功抓持但四次 probe 累计 EE 运动只有约 0.257 mm，判为 UNOBSERVABLE。它在机器人运动前已被动回到闭门限位，不能将这段被动变化记为交互成果。
3. 45134：两个配置分别完成 28 / 8 个 refinement 小步；实际开门约 2.53° / 1.58° 后，原模型一致性误差约 1.005 / 1.003 mm 触发安全停机。对应接触面漂移仅约 0.106 / 0.018 mm，GT 事后真实相对位移约 0.814 / 1.004 mm，不能仅凭旧 `SLIP` 标签断言脱手。

首个配置的在线轴方向误差为约 0.20°，但轴线误差约 60 mm；第二个配置方向误差约 9.16°。最新在线估计没有通过独立最终验收，也没有展示一致的结构误差改善。当前证据显示短弧结构不确定性与接触顺应仍然混杂；停止继续调 fitter。

**最终 0.30 mm、接触/力、关节余量等门槛均未放宽。Held-out 结构预测为 NOT REACHED，没有用训练残差冒充测试 RMSE。**

运行命令、主表、事后轴/轴线诊断和图表见下文。连续视频在服务器原运行目录，交付目录另附代表性视频。grasp/contact/mobile baseline 文件未修改；实现提交为 `b339503`。

---

'''
body=report.read_text()
if not body.startswith('# 本轮结论'):report.write_text(prefix+body)
with report.open('a') as f:
 f.write(f'\n## Candidate-level safety provenance\n\n`candidate_attempts.csv` retains all {len(attempts)} actual candidate executions, including {counts["candidate_collision_stops"]} collision stops. A later selected candidate does not erase earlier unsafe-approach stops.\n')
print(json.dumps(counts,indent=2))
