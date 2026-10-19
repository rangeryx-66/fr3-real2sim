"""Correct native-state units in an uncut diagnostic video; retain the original."""
import argparse,json,subprocess,shutil
from pathlib import Path


def stamp(seconds):
    h=int(seconds//3600);m=int(seconds//60)%60;s=seconds%60
    return f'{h}:{m:02d}:{s:05.2f}'


def annotate(output):
    output=Path(output);report=json.loads((output/'report.json').read_text())
    rows=json.loads((output/'observations.json').read_text())
    events=[];fps=float(report['render_fps']);dt=float(report['physics_dt_s'])
    period=max(1,round(1./fps/dt))
    unit='deg' if report['actual_units']=='degrees' else 'mm'
    scale=1. if unit=='deg' else 1000.
    for frame,index in enumerate(range(period-1,len(rows),period)):
        row=rows[index];state=float(row['door_angle_deg'])*scale
        loads=row['forces_n'];load='/'.join(f'{v:.2f}' for v in loads.values())
        label=f"KNOWN_MODEL_DIAGNOSTIC | {row['phase']}\\Nactual object = {state:.2f} {unit} | margin = {row['margin_rad']:.3f} rad | pads = {load} N"
        events.append(f'Dialogue: 0,{stamp(frame/fps)},{stamp((frame+1)/fps)},Default,,0,0,0,,{label}')
    ass='''[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 960
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,DejaVu Sans,24,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,1,0,7,12,12,8,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
'''+ '\n'.join(events)+'\n'
    subtitles=output/'actual_state_units.ass';subtitles.write_text(ass)
    destination=output/'continuous_actual_state.mp4'
    subprocess.run([shutil.which('ffmpeg') or 'ffmpeg','-y','-i',str(output/report['video']),
                    '-vf',f'drawbox=x=0:y=0:w=iw:h=80:color=0x0C0C0C:t=fill,ass={subtitles}',
                    '-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(destination)],
                   check=True,stdout=subprocess.DEVNULL,stderr=(output/'annotation_ffmpeg.log').open('w'))
    (output/'video_provenance.json').write_text(json.dumps(dict(original=report['video'],annotated=destination.name,
      edits='Correct diagnostic state units and phase overlay only; no cuts, retiming, scene editing or stitched trials.',
      source='Native joint state in observations.json',mode=report['mode']),indent=2))
    return destination

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);print(annotate(p.parse_args().output))
