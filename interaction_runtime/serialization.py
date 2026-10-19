"""Result-only JSON conversion; commands stay strict, raw binary remains unchanged."""
import math
from pathlib import Path

def result_packet(value):
    unavailable=[]
    def convert(x,path):
        if hasattr(x,'tolist'):return convert(x.tolist(),path)
        if isinstance(x,Path):return str(x)
        if isinstance(x,float) and not math.isfinite(x):unavailable.append(path);return None
        if isinstance(x,dict):return {str(k):convert(v,path+'/'+str(k)) for k,v in x.items()}
        if isinstance(x,(list,tuple)):return [convert(v,path+'/'+str(i)) for i,v in enumerate(x)]
        return x
    result=convert(value,'')
    if unavailable:
        if isinstance(result,dict):result['unavailable_nonfinite_JSON_fields']=unavailable
        else:result=dict(value=result,unavailable_nonfinite_JSON_fields=unavailable)
    return result
