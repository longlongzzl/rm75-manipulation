from types import SimpleNamespace as NS
import argparse
import pytest
from rm75_app.workcell.legacy import native_contact_policy,build_native_argv
from rm75_app.workcell.spec import validate_spec


@pytest.mark.parametrize('task',['pickplace','magnetic'])
def test_sim_contact_compatibility_never_applies_to_real(task):
    profile={task:{'simulation_contact_policy':'transport_world_checked_compatibility'}}
    assert native_contact_policy({'task':task,'mode':'sim'},profile)=='transport_world_checked_compatibility'
    assert native_contact_policy({'task':task,'mode':'real'},profile)=='strict'


@pytest.mark.parametrize('task',['pickplace','magnetic'])
def test_browser_cannot_select_a_contact_policy(task,profile,design):
    params={'object_name':'gluestick'} if task=='pickplace' else {'design':design}
    params['simulation_contact_policy']='transport_world_checked_compatibility'
    with pytest.raises(ValueError):validate_spec({'task':task,'mode':'sim','parameters':params},profile)


def test_unknown_or_broad_magnetic_policy_fails_closed():
    with pytest.raises(ValueError):native_contact_policy({'task':'pickplace','mode':'sim'},
        {'pickplace':{'simulation_contact_policy':'disable_all'}})
    with pytest.raises(ValueError):native_contact_policy({'task':'magnetic','mode':'sim'},
        {'magnetic':{'simulation_contact_policy':'original'}})


def test_sim_policy_requires_original_non_graph_cli_contract(tmp_path):
    def parser():
        p=argparse.ArgumentParser()
        for flag in ('--auto-execute','--no-fast-chain-cuda-graph-ik'):p.add_argument(flag,action='store_true')
        p.add_argument('--object-name');return p
    module=NS(build_arg_parser=parser)
    spec={'task':'pickplace','mode':'sim','parameters':{'object_name':'gluestick'}}
    profile={'pickplace':{'simulation_contact_policy':'transport_world_checked_compatibility'}}
    argv=build_native_argv(module,spec,profile,tmp_path,tmp_path)
    assert '--no-fast-chain-cuda-graph-ik' in argv and '--execute-real' not in argv
