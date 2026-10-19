"""Fetch selected original SFU ZIP members with HTTP ranges and CRC verification."""
import argparse,json,subprocess,tempfile,zipfile,struct,zlib,hashlib
from pathlib import Path

def fetch(url,offset,end):
    with tempfile.NamedTemporaryFile() as f:
        subprocess.run(['curl','-fL','--retry','2','--retry-all-errors','--connect-timeout','15','--max-time','180','-r',f'{offset}-{end}',url,'-o',f.name],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        data=Path(f.name).read_bytes()
    if len(data)!=end-offset+1:raise RuntimeError('SERVER_RANGE_NOT_HONORED')
    return data

def extract(url,index,pattern,output):
    root=Path(output).resolve();root.mkdir(parents=True,exist_ok=True);rows=[]
    for info in zipfile.ZipFile(index).infolist():
        if pattern not in info.filename or info.is_dir():continue
        target=(root/info.filename).resolve()
        if root not in target.parents:raise ValueError('ZIP_PATH_OUTSIDE_OUTPUT')
        if target.exists() and target.stat().st_size==info.file_size:
            data=target.read_bytes()
            if zlib.crc32(data)==info.CRC:continue
        h=fetch(url,info.header_offset,info.header_offset+29)
        values=struct.unpack('<IHHHHHIIIHH',h);n,e=values[-2:];offset=info.header_offset+30+n+e
        payload=fetch(url,offset,offset+info.compress_size-1)
        data=zlib.decompress(payload,-15) if info.compress_type==8 else payload
        if len(data)!=info.file_size or zlib.crc32(data)!=info.CRC:raise RuntimeError('ZIP_MEMBER_CRC_FAILED')
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
        rows.append({'file':str(target),'source':url,'zip_member':info.filename,'crc32':info.CRC,'sha256':hashlib.sha256(data).hexdigest()})
        (root/'official_download_provenance.json').write_text(json.dumps(rows,indent=2))
    return rows

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--url',required=True);p.add_argument('--index',type=Path,required=True);p.add_argument('--pattern',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(extract(a.url,a.index,a.pattern,a.output),indent=2))
