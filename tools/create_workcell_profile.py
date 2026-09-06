#!/usr/bin/env python3
"""Generate a safe local profile. This does not qualify or connect hardware."""
from pathlib import Path
import argparse,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rm75_app.workcell.io import atomic_json,read_json


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():
        raise FileExistsError('Refusing to replace an existing hardware profile')
    root=Path(__file__).resolve().parents[1]
    profile=read_json(root/'examples/workcell/machine.example.json')
    for task,environment in [('pickplace','foundationpose310'),('magnetic','foundationpose310'),('pusht','curobo2')]:
        python=Path.home()/f'anaconda3/envs/{environment}/bin/python'
        profile[task]['python']=str(python if python.is_file() else Path(sys.executable))
    atomic_json(a.output,profile)
    print(f'Safe profile written: {a.output.resolve()} — hardware is DISABLED')

if __name__=='__main__':
    main()
