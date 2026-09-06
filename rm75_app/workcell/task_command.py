"""Bridge the existing TaskAdapter contract into the common worker service."""
from pathlib import Path
import os


def command(request,*,task,mode,python='python'):
    from rm75_app.core.contracts import CommandSpec
    from rm75_app.paths import APP_ROOT
    if request.source is None:
        raise ValueError('This mode requires --source <workcell_request.json>; see examples/workcell')
    profile=request.options.get('machine_profile') or os.environ.get('RM75_WORKCELL_PROFILE')
    if not profile:
        profile=APP_ROOT/'runtime_data/workcell/machine.json'
    argv=(python,'-m','rm75_app.workcell','run','--profile',str(profile),
          '--request',str(Path(request.source).expanduser().resolve()),
          '--task',task,'--mode',mode,*request.args)
    return CommandSpec(argv=argv,cwd=APP_ROOT,description=f'{task} / {mode}',
                       notes=('Real mode additionally requires --allow-real and interactive operator confirmation.',))
