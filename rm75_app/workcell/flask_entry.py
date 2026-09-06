"""Start the existing Flask frontend with the new three-task routes mounted.

Usage: python -m rm75_app.workcell.flask_entry --profile machine.json
Old scanner routes remain available; new controls are /workcell/.
"""
from __future__ import annotations
import argparse
from pathlib import Path


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--port',type=int,default=7860)
    parser.add_argument('--allow-real',action='store_true')
    args=parser.parse_args(argv)
    from rm75_app.web import control_panel
    from .server import mount
    app=getattr(control_panel,'app',None)
    if app is None:
        raise RuntimeError('Existing Flask app not found; inspect control_panel module before mounting')
    service=mount(app,args.profile,Path(__file__).resolve().parents[2],allow_real=args.allow_real)
    try:
        app.run(host='127.0.0.1',port=args.port,threaded=True,use_reloader=False,debug=False)
    finally:
        service.close()

if __name__=='__main__':
    main()
