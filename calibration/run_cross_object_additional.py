"""One additional existing category, same frozen protocol; no GT-based selection."""
from pathlib import Path
p=Path(__file__).with_name('run_cross_object_validation.py');s=p.read_text().replace("O=R/'results/fr3_qcomp_cross_object'","O=R/'results/fr3_qcomp_cross_object_additional'")
a=s.index('objects=');b=s.index('\nfrozen=',a);s=s[:a]+"objects=[('hidden_tuna',list(range(1010,1015)))]"+s[b:]
exec(compile(s,str(p),'exec'),{'__file__':str(p),'__name__':'__main__'})
