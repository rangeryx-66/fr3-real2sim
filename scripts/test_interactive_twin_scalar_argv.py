"""Pure bootstrap/argparse regression; no Isaac launch, repo writes or scene edits."""
import argparse,ast,contextlib,hashlib,io,json,math,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
SOURCE=ROOT/'interactive_twin/plant.py';LEGACY=ROOT/'scripts/piper_mobile_execute.py'
from interactive_twin import plant as MOD
SOURCE_SHA_BEFORE=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
LEGACY_SHA_BEFORE=hashlib.sha256(LEGACY.read_bytes()).hexdigest()
TREE=ast.parse(LEGACY.read_text())
BOOT=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='bootstrap')
ASSIGN=next(n for n in BOOT.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='sys.argv')
ARGV_SOURCE=ast.get_source_segment(LEGACY.read_text(),ASSIGN)
PARSER_SOURCE='''import argparse,sys
parser=argparse.ArgumentParser()
parser.add_argument('--gpu',type=int)
parser.add_argument('--asset-root')
for option in ('asset-x','asset-y','asset-yaw-deg','fixture-height-m'):
 parser.add_argument('--'+option,type=float)
parser.add_argument('--record-overview',action='store_true')
parser.add_argument('--enable-isolation-diagnostics',action='store_true')
parsed=parser.parse_args()
received_argv=list(sys.argv)
class Vec:
 def __init__(self,v):self.v=v
 def tolist(self):return self.v
baked_mass_audit={}
plant_resistance_audit={}
fixture={}
asset_xyz=Vec([parsed.asset_x,parsed.asset_y,0.])
asset_rotation=Vec([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
'''
FIXTURE='''def bootstrap(a,base):
    legacy=Path('loader_fixture.py')
    source=a.parser_source
    place=a.place;old=sys.argv
    %s
    scene={}
    exec(compile(source,str(legacy),'exec'),scene);sys.argv=old;return scene
''' % ARGV_SOURCE
OPTIONS={'asset-x':'x_m','asset-y':'y_m','asset-yaw-deg':'yaw_deg','fixture-height-m':'fixture_height_m'}

class ScalarArgv(unittest.TestCase):
 def run_bootstrap(self,place,source=FIXTURE):
  original=types.ModuleType('piper_mobile_execute');original.bootstrap=lambda *a:None;original.sys=sys;original.Path=Path
  with tempfile.TemporaryDirectory() as directory:
   args=types.SimpleNamespace(place=place,parser_source=PARSER_SOURCE,gpu=3,asset_root=Path('/tmp/an asset'),output=Path(directory))
   with patch.dict(sys.modules,{'piper_mobile_execute':original}),patch.object(MOD.inspect,'getsource',return_value=source),patch.object(MOD,'adapt_loader_source',side_effect=lambda s,j:s),patch.object(sys,'argv',['unchanged-outer-argv']):
    result=MOD.bootstrap_job(args,[.5,-.55,-.1,150],{})
    self.assertEqual(sys.argv,['unchanged-outer-argv'])
    self.assertTrue((args.output/'initial_scene_private.json').is_file())
    return result

 def test_legacy_reproduces_negative_scientific_notation_error(self):
  parser=argparse.ArgumentParser();parser.add_argument('--asset-y',type=float)
  with contextlib.redirect_stderr(io.StringIO()) as errors:
   with self.assertRaises(SystemExit):parser.parse_args(['--asset-y','-2.7755575615628914e-17'])
  self.assertIn('expected one argument',errors.getvalue())

 def test_all_scalar_tokens_and_float_values_preserved_exactly(self):
  rows=[{'x_m':.275,'y_m':-2.7755575615628914e-17,'yaw_deg':-120.,'fixture_height_m':.237},
        {field:-1.25e-17 for field in OPTIONS.values()},
        {field:-0.0 for field in OPTIONS.values()},
        {field:-5e-324 for field in OPTIONS.values()},
        {field:1.25e-17 for field in OPTIONS.values()}]
  for values in rows:
   with self.subTest(values=values):
    result=self.run_bootstrap(values);argv=result['received_argv'];parsed=result['parsed']
    for option,field in OPTIONS.items():
     self.assertIn('--'+option+'='+str(values[field]),argv)
     self.assertNotIn('--'+option,argv)
     actual=getattr(parsed,option.replace('-','_'))
     self.assertEqual(actual.hex(),float(values[field]).hex())
    self.assertEqual(parsed.gpu,3);self.assertEqual(parsed.asset_root,'/tmp/an asset')
    self.assertTrue(parsed.record_overview);self.assertTrue(parsed.enable_isolation_diagnostics)

 def test_actual_legacy_assignment_has_each_guarded_scalar_once(self):
  for option,field in OPTIONS.items():
   self.assertEqual(ARGV_SOURCE.count(f"'--{option}',str(place['{field}'])"),1)
  self.assertEqual(LEGACY.read_text().count("    exec(compile(source,str(legacy),'exec'),scene);sys.argv=old;"),1)

 def test_changed_legacy_scalar_format_is_not_silently_patched(self):
  with self.assertRaisesRegex(RuntimeError,'FROZEN_BOOTSTRAP_SCALAR_ARG_CHANGED:asset-y'):
   self.run_bootstrap({v:0. for v in OPTIONS.values()},FIXTURE.replace("'--asset-y'","'--different-asset-y'"))

 def test_source_and_baseline_remain_untouched(self):
  self.assertEqual(hashlib.sha256(SOURCE.read_bytes()).hexdigest(),SOURCE_SHA_BEFORE)
  self.assertEqual(hashlib.sha256(LEGACY.read_bytes()).hexdigest(),LEGACY_SHA_BEFORE)

if __name__=='__main__':unittest.main(verbosity=2)
