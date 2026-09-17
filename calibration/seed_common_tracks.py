"""Map pass-0 BundleSDF tracking stems back to the combined prepared dataset."""
import argparse,json,shutil
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('combined_dataset',type=Path);p.add_argument('pass_dataset',type=Path);p.add_argument('pass_tracking',type=Path);p.add_argument('common_tracking',type=Path);a=p.parse_args()
    out=a.common_tracking/'ob_in_cam';out.mkdir(parents=True,exist_ok=True);n=0
    for pose in sorted((a.pass_dataset/'poses').glob('*.json')):
        row=json.loads(pose.read_text());shutil.copy2(a.pass_tracking/'ob_in_cam'/f'{pose.stem}.txt',out/f"{row['combined_prepared_stem']}.txt");n+=1
    print(n)


if __name__=='__main__':main()
