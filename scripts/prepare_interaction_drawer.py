"""Extract dataset single-prismatic drawers; reuse established preparation/proxy."""
import argparse,json,sys,zipfile,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))

def main():
 p=argparse.ArgumentParser();p.add_argument('--zip',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--limit',type=int,default=3);p.add_argument('--include-multi',action='store_true');p.add_argument('--max-height',type=float,default=10.);p.add_argument('--require-handle-semantics',action='store_true');p.add_argument('--part-keywords',nargs='+',default=['drawer']);p.add_argument('--moving-handle',action='store_true',help='Require metadata handle inside selected prismatic subtree');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 raw=a.output/'source';raw.mkdir(exist_ok=True);inventory=[]
 with zipfile.ZipFile(a.zip) as z:
  names=set(z.namelist())
  for n in sorted(names):
   if not n.endswith('.urdf'):continue
   root=ET.fromstring(z.read(n));js=[j for j in root.findall('joint') if j.get('type')!='fixed']
   if not any(j.get('type')=='prismatic' for j in js) or (not a.include_multi and len(js)!=1):continue
   aid=Path(n).stem;mp='PhysX_mobility/finaljson/'+aid+'.json'
   if mp not in names:continue
   meta=json.loads(z.read(mp));parts=meta.get('parts',[])
   if not any(any(k.lower() in x.get('name','').lower() for k in a.part_keywords) for x in parts):continue
   if a.require_handle_semantics and not any('handle' in x.get('name','').lower() for x in parts):continue
   height=float(meta['dimension'].split('*')[-1])/100.
   if height>a.max_height:continue
   selected=next(j for j in js if j.get('type')=='prismatic')
   if a.moving_handle:
    moving={selected.find('child').get('link')}
    while True:
     nxt=moving|{j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in moving}
     if nxt==moving:break
     moving=nxt
    if not any('handle' in x['name'].lower() and 'l_'+str(x['label']) in moving for x in parts):continue
   inventory.append({'asset_id':aid,'height_m':height,'object_name':meta['object_name'],'part_count':len(parts),'joint':selected.attrib,'joint_count':len(js),'rule':{'joint_type':'prismatic','part_keywords':a.part_keywords,'moving_handle_required':a.moving_handle,'order':['joint_count','real_height','part_count','asset_id'],'other_joints_preserved':True}})
  inventory.sort(key=lambda x:(x['joint_count'],x['height_m'],x['part_count'],x['asset_id']))
  (a.output/'asset_inventory.json').write_text(json.dumps(inventory,indent=2))
  for item in inventory[:a.limit]:
   aid=item['asset_id'];prefix='PhysX_mobility/'
   needed=[n for n in names if n==prefix+'urdf/'+aid+'.urdf' or n==prefix+'finaljson/'+aid+'.json' or n.startswith(prefix+'partseg/'+aid+'/')]
   for n in needed:
    if n.endswith('/'):continue
    f=raw/n[len(prefix):];f.parent.mkdir(parents=True,exist_ok=True);f.write_bytes(z.read(n))
   # Guarded copy adds correct linear-limit scaling and type metadata only.
   s=(ROOT/'scripts/prepare_physx_microwave.py').read_text()
   s=s.replace("j.get('type')=='revolute'","j.get('type')=='prismatic'")
   s=s.replace("if len(movable)!=1:raise ValueError('minimal microwave requires one revolute door joint')","if not movable:raise ValueError('dataset requires a prismatic joint')")
   s=s.replace("'object_name':'Microwave'","'object_name':json.loads(meta.read_text())['object_name']")
   s=s.replace("'door_link':'l_1'","'door_link':next(n for n,_,_,_ in pieces if n in moving_links)")
   s=s.replace("'joint_name':joint.get('name')","'joint_type':joint.get('type'),'joint_name':joint.get('name')")
   marker="    prepared=target/'urdf'/f'{args.asset_id}.urdf'"
   assert s.count(marker)==1
   s=s.replace(marker,"    for j in root.findall('joint'):\n        if j.get('type')=='prismatic':\n            for key in ('lower','upper'):\n                j.find('limit').set(key,str(float(j.find('limit').get(key))*factor))\n"+marker)
   old=sys.argv;sys.argv=['prepare','--source',str(raw),'--asset-id',aid,'--height-m',str(item['height_m']),'--output',str(a.output/'prepared'/aid)]
   exec(compile(s,'<dataset-drawer-preparation>','exec'),{'__name__':'__main__','__file__':str(ROOT/'scripts/prepare_physx_microwave.py')});sys.argv=old
 from rank_prepared_handle_assets import scan
 rank=scan(a.output/'prepared');rank['ranking']=[x for x in rank['ranking'] if x['joint_type']=='prismatic']
 if a.require_handle_semantics:
  semantic={}
  for item in inventory[:a.limit]:
   meta=json.loads((raw/'finaljson'/f"{item['asset_id']}.json").read_text());semantic[item['asset_id']]={stem for part in meta['parts'] if 'handle' in part['name'].lower() for stem in part.get('obj',[])}
  rank['ranking']=[x for x in rank['ranking'] if x['mesh'] in semantic[x['asset_id']]]
  rank['semantic_selection']='dataset Handle metadata required; no drawer rail/frame proxies'
 (a.output/'handle_ranking.json').write_text(json.dumps(rank,indent=2));print('prepared',inventory[:a.limit])
if __name__=='__main__':main()
