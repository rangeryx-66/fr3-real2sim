"""Read-only branch inventory of the actual loaded-source ledger plus current adapters.

No Isaac import, plant/asset read, command publication or safety-registry write.
Branch inventory is deliberately overinclusive; authority is assessed in LIMITS.md.
"""
import argparse, ast, hashlib, json
from pathlib import Path

PACKAGES = ('interaction_runtime','agentic_interaction_v81',
            'agentic_interaction_v84','agentic_interaction_v87')


def inventory(root, ledger):
    root=Path(root); loaded=json.loads(Path(ledger).read_text())['sources']
    names=set(loaded)
    for folder in PACKAGES:
        names.update(str(p.relative_to(root)) for p in (root/folder).glob('*.py'))
    sources=[];branches=[]
    for name in sorted(names):
        p=root/name
        if not p.is_file() or p.suffix!='.py':continue
        data=p.read_bytes();tree=ast.parse(data,filename=name)
        sources.append(dict(file=name,sha256=hashlib.sha256(data).hexdigest(),
            historical_loaded_sha256=loaded.get(name),in_incident_ledger=name in loaded))
        class Scan(ast.NodeVisitor):
            def __init__(self):self.scope=[]
            def visit_FunctionDef(self,node):
                self.scope.append(node.name);self.generic_visit(node);self.scope.pop()
            visit_AsyncFunctionDef=visit_FunctionDef
            def add(self,node,kind,expression):
                branches.append(dict(file=name,line=node.lineno,scope='.'.join(self.scope),
                    kind=kind,expression=expression))
            def visit_If(self,node):
                self.add(node,'CONDITION',ast.unparse(node.test));self.generic_visit(node)
            def visit_Raise(self,node):
                self.add(node,'RAISE',ast.unparse(node.exc) if node.exc else 'RERAISE')
            def visit_ExceptHandler(self,node):
                self.add(node,'EXCEPTION_OWNER',ast.unparse(node.type) if node.type else 'BaseException');self.generic_visit(node)
            def visit_Break(self,node):self.add(node,'ACTION_LOOP_BOUNDARY','break')
            def visit_Return(self,node):
                text=ast.unparse(node.value) if node.value else ''
                if any(k in text for k in ('status','reason','safe','valid','feasible','error','latched','stop')):
                    self.add(node,'RETURN_GATE',text)
        Scan().visit(tree)
    return dict(schema='v87-execution-branch-inventory-v1',
        scope='Historical loaded modules plus all current runtime/adapters; includes cold branches. Not dynamic coverage.',
        ledger_sha256=hashlib.sha256(Path(ledger).read_bytes()).hexdigest(),
        missing_loaded_sources=sorted(n for n in loaded if not (root/n).is_file()),
        source_count=len(sources),branch_count=len(branches),sources=sources,branches=branches)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--ledger',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=inventory(a.root,a.ledger)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k]for k in ('source_count','branch_count','missing_loaded_sources')}))
