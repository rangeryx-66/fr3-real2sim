"""Hash closed private evidence once; publish only compact manifest commitments."""
import argparse,hashlib,json,subprocess
from pathlib import Path


def main(root,output):
    root=Path(root);directory=root/'results/prismatic_v751_20261022';integrity=directory/'integrity';integrity.mkdir(exist_ok=True)
    episodes=[]
    for p in sorted((directory/'episodes').iterdir()):
        if not p.is_dir():continue
        done=json.loads((p/'PROCESS_COMPLETION.json').read_text())
        assert done['actor_exited'],'OWNED_ACTOR_NOT_CLOSED:'+p.name
        process=json.loads((p/'PROCESS.json').read_text());pid=process['pid']
        status=subprocess.run(['ps','-p',str(pid),'-o','args='],capture_output=True,text=True)
        if p.name in status.stdout:raise ValueError('OWNED_ACTOR_STILL_ALIVE:'+p.name)
        files={str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest()for f in sorted(p.rglob('*'))if f.is_file()}
        manifest=integrity/(p.name+'.sha256.json');manifest.write_text(json.dumps(files,indent=2)+'\n')
        result=json.loads((p/'EPISODE_RESULT.json').read_text())if(p/'EPISODE_RESULT.json').exists()else{}
        seal=json.loads((p/'ONLINE_SEAL_V751.json').read_text())if(p/'ONLINE_SEAL_V751.json').exists()else{}
        for name,digest in seal.get('sha256',{}).items():assert files[name]==digest,'SEALED_EVIDENCE_CHANGED:'+name
        episodes.append(dict(episode=p.name,server='test-X640-G50 (labserver_inschool)',private_directory=str(p.resolve()),
            physical_result=result.get('status'),completion=done,owned_actor_absent=True,
            job_sha256=process.get('job_sha256'),online_seal=seal,private_file_count=len(files),
            bytes=sum(f.stat().st_size for f in p.rglob('*')if f.is_file()),
            private_complete_manifest=str(manifest.resolve()),private_complete_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest()))
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(dict(raw_measurements_private=True,no_raw_copies_created=True,episodes=episodes,
        total_episode_bytes=sum(r['bytes']for r in episodes)),indent=2)+'\n')
    print('closed private scenes',len(episodes),'bytes',sum(r['bytes']for r in episodes))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--root',default='.');ap.add_argument('--output',required=True);a=ap.parse_args();main(a.root,a.output)
