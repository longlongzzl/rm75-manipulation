"""Constrained LLM design generation on hash-addressed ORIGINAL Jimu boards.

The model selects a parent-closed substructure; it cannot invent coordinates,
parts, base geometry, robot commands, or calibration. A generated design still
requires the existing native full-chain planning and physical outcome checks.
"""
from __future__ import annotations
import copy
from collections import Counter
from pathlib import Path
import re
from typing import Callable

from .design import validate_design, piece_key, parent_key, piece_matrix
from rm75_app.workcell.io import read_json, digest, dumps, loads, integer

BOARD_IDS = ('grid_3x3', 'arc')
NAME = re.compile(r'[A-Za-z0-9_-]{1,100}\Z')


def _text(value, name, maximum):
    if not isinstance(value, str) or not value.strip() or len(value) > maximum:
        raise ValueError(f'{name} must be nonempty text of at most {maximum} characters')
    return value.strip()


def validate_library(value):
    """Validate every template, inventory and explicit support dependency."""
    if not isinstance(value, dict) or value.get('schema') != 'rm75_jimu_design_library_v1':
        raise ValueError('Import the original grid/arc task library first')
    boards = value.get('boards')
    if not isinstance(boards, dict) or not boards or set(boards) - set(BOARD_IDS):
        raise ValueError('Library boards must be grid_3x3 and/or arc')
    for board_id, board in boards.items():
        if not isinstance(board, dict):
            raise ValueError('Invalid board record')
        templates = board.get('templates')
        if not isinstance(templates, list) or not 1 <= len(templates) <= 24:
            raise ValueError('Expected 1..24 original templates for each board')
        seen = set()
        for template in templates:
            key = template.get('id')
            if not isinstance(key, str) or not NAME.fullmatch(key) or key in seen:
                raise ValueError('Unique path-safe template id required')
            seen.add(key)
            scene = template.get('design')
            design = validate_design(scene)
            if digest(scene) != template.get('design_digest'):
                raise ValueError(f'Original template digest mismatch: {board_id}/{key}')
            locked = [p for p in scene['pieces'] if p.get('locked', False)]
            if not locked or digest(locked) != template.get('locked_digest'):
                raise ValueError('Original locked base missing or modified')
            if board_id == 'grid_3x3' and len(locked) != 9:
                raise ValueError('The old 3x3 board must retain all nine locked plates')
            inventory = template.get('inventory')
            actual = dict(Counter(p['type'] for p in scene['pieces'] if not p.get('locked', False)))
            if inventory != actual or sum(actual.values()) > 12:
                raise ValueError('Inventory must match the original <=12 movable pieces')
            dependencies(template)  # Reject ambiguous/absent supports and cycles.
            recipe = template.get('native_recipe', {})
            if not isinstance(recipe, dict) or set(recipe) - {'fixed_scene', 'fixed_scene_sha256', 'native_args', 'task_manifest'}:
                raise ValueError('Invalid trusted native recipe')
            manifest=recipe.get('task_manifest')
            if manifest is not None and (not isinstance(manifest,dict) or manifest.get('schema')!='jimu_task_manifest_v1'):
                raise ValueError('Original native task manifest schema mismatch')
            # Native args come from the read-only importer, NOT the LLM/browser.
            if not isinstance(recipe.get('native_args', []), list) or not all(isinstance(x, str) for x in recipe.get('native_args', [])):
                raise ValueError('Invalid original native argument list')
    return copy.deepcopy(value)


def dependencies(template):
    pieces = template['design']['pieces']
    by_key = {piece_key(p): p for p in pieces}
    aliases = {}
    for key, piece in by_key.items():
        for alias in (key, str(piece.get('id') or key)):
            if alias in aliases and aliases[alias] != key:
                raise ValueError('Ambiguous original support identity')
            aliases[alias] = key
    extra = template.get('support_dependencies', {})
    if not isinstance(extra, dict) or set(extra) - set(by_key):
        raise ValueError('Unknown explicit support dependency')
    deps = {}
    for key, piece in by_key.items():
        support = parent_key(piece)
        raw = extra.get(key, [])
        if not isinstance(raw, list): raise ValueError('Support dependency must be a list')
        raw = list(raw)
        if support: raw.append(support)
        if not piece.get('locked', False) and not raw:
            raise ValueError(f'{key}: explicit original parent/support is required for generation')
        if any(not isinstance(x, str) or x not in aliases for x in raw):
            raise ValueError(f'{key}: missing support')
        deps[key] = set(aliases[x] for x in raw)
        if key in deps[key]: raise ValueError('Self dependency')
    seen, visiting = set(), set()
    def visit(key):
        if key in visiting: raise ValueError('Support dependency cycle')
        if key in seen: return
        visiting.add(key)
        for parent in deps[key]: visit(parent)
        visiting.remove(key); seen.add(key)
    for key in deps: visit(key)
    return deps


def load_library(profile):
    path = profile.get('magnetic', {}).get('design_library')
    if not isinstance(path, str) or not path:
        raise ValueError('Original Jimu base library is not installed; run import_jimu_design_library.py')
    return validate_library(read_json(Path(path).expanduser(), max_bytes=4_000_000))


def catalog_summary(library):
    result = []
    for board_id, board in library['boards'].items():
        for template in board['templates']:
            deps = dependencies(template)
            pieces = template['design']['pieces']
            result.append(dict(board_id=board_id, board_title=board.get('title', board_id),
                template_id=template['id'], title=template.get('title', template['id']),
                inventory=template['inventory'], locked_count=sum(bool(p.get('locked', False)) for p in pieces),
                slots=[dict(role=piece_key(p), type=p['type'], parent_roles=sorted(deps[piece_key(p)]),
                            center=p['center']) for p in pieces if not p.get('locked', False)],
                selection_space='original_parent_closed_substructures',
                physical_buildability_verified=False))
    return result


def compile_selection(library, proposal, *, board_id, piece_budget=12):
    integer(piece_budget, 'piece_budget', 1, 12)
    if not isinstance(proposal, dict) or set(proposal) != {'board_id', 'template_id', 'title', 'selected_roles', 'explanation'}:
        raise ValueError('LLM must return only board_id/template_id/title/selected_roles/explanation')
    if board_id not in library['boards'] or proposal['board_id'] != board_id:
        raise ValueError('LLM changed the requested base')
    title = _text(proposal['title'], 'title', 100)
    explanation = _text(proposal['explanation'], 'explanation', 1500)
    templates = {x['id']: x for x in library['boards'][board_id]['templates']}
    if proposal.get('template_id') not in templates:
        raise ValueError('Unknown original template')
    template = templates[proposal['template_id']]
    roles = proposal.get('selected_roles')
    if (not isinstance(roles, list) or not 1 <= len(roles) <= piece_budget
            or any(not isinstance(x, str) for x in roles) or len(set(roles)) != len(roles)):
        raise ValueError('Select distinct roles within the available piece budget')
    source = template['design']; by_key = {piece_key(p): p for p in source['pieces']}
    locked = {k for k, p in by_key.items() if p.get('locked', False)}
    selected = set(roles)
    if not selected <= set(by_key) - locked:
        raise ValueError('Unknown, fixed-base or invented movable role')
    deps = dependencies(template)
    for key in roles:
        if not deps[key] <= selected | locked:
            raise ValueError(f'{key} needs its supports: {sorted(deps[key] - selected - locked)}')
    # Exact original coordinates/axes/parent transforms, never generated by LLM.
    payload = copy.deepcopy(source)
    payload['pieces'] = [copy.deepcopy(p) for p in source['pieces'] if piece_key(p) in selected | locked]
    tags = payload.get('apriltags', {}).get('attached_tags')
    if isinstance(tags, list):
        aliases = {str(p.get('id') or piece_key(p)) for p in payload['pieces']} | selected | locked
        payload['apriltags']['attached_tags'] = [x for x in tags if
            not (x.get('attached_to_role') or x.get('attached_to_piece_id')) or
            (x.get('attached_to_role') or x.get('attached_to_piece_id')) in aliases]
    check = validate_design(payload)
    used = dict(Counter(by_key[k]['type'] for k in roles))
    if any(v > template['inventory'].get(k, 0) for k, v in used.items()):
        raise ValueError('Inventory exceeded')
    proof = dict(board_id=board_id, template_id=template['id'], title=title,
        selected_roles=list(check.ordered_roles), template_digest=template['design_digest'],
        locked_digest=template['locked_digest'], design_digest=digest(payload),
        native_recipe_digest=digest(template.get('native_recipe',{})),
        original_geometry_preserved=True, explicit_support_closure=True, inventory_used=used,
        movable_count=len(roles), explanation=explanation,
        validation='original_geometry_and_support_graph_only',
        native_planning_verified=False, physical_buildability_verified=False)
    return dict(design=payload, proof=proof)


def subset_task_manifest(manifest, design, selected_roles):
    """Adapt the original task manifest to a generated parent-closed subset.

    The native program rejects build_layers/tray roles that are not part of the
    submitted design, and derives physical tray slot poses from the role order
    index. This keeps the selected roles' original layer grouping, original
    tray slot positions (via an explicit tray.slot_layout) and triangle-slot
    identity, while every non-selected role disappears from scheduling.
    """
    manifest = copy.deepcopy(manifest)
    roles = set(selected_roles)
    builder = manifest.get('builder')
    if not isinstance(builder, dict):
        raise ValueError('Original task manifest lacks a builder section')
    if isinstance(builder.get('build_layers'), list):
        builder['build_layers'] = [
            [role for role in layer if role in roles]
            for layer in builder['build_layers']
            if any(role in roles for role in layer)
        ]
    original_order = builder.get('tray_slot_role_order')
    if isinstance(original_order, list) and original_order:
        filtered = [role for role in original_order if role in roles]
        builder['tray_slot_role_order'] = filtered
        raw_indices = builder.get('triangle_tray_slot_indices')
        if isinstance(raw_indices, list) and raw_indices and filtered:
            original_triangle = set(int(i) for i in raw_indices if isinstance(i, (int, float)))
            remapped = []
            for position, role in enumerate(filtered):
                original_index = original_order.index(role) if role in original_order else position
                if original_index in original_triangle:
                    remapped.append(position)
            builder['triangle_tray_slot_indices'] = remapped
        pieces = {piece_key(p): p for p in design.get('pieces', [])}
        manifest.setdefault('tray', {})
        manifest['tray']['slot_layout'] = [
            dict(role=role, slot=(original_order.index(role) if role in original_order else position),
                 type=str(pieces.get(role, {}).get('type', 'square')))
            for position, role in enumerate(filtered)
        ]
    return manifest


def validate_generated_request(library, design, proof):
    if not isinstance(proof, dict): raise ValueError('Generated design provenance required')
    compiled = compile_selection(library, {k: proof[k] for k in
        ('board_id', 'template_id', 'title', 'selected_roles', 'explanation')}, board_id=proof['board_id'])
    if digest(design) != compiled['proof']['design_digest'] or proof != compiled['proof']:
        raise ValueError('Generated design or provenance changed; generate/validate it again')
    template = next(t for t in library['boards'][proof['board_id']]['templates'] if t['id'] == proof['template_id'])
    return copy.deepcopy(template.get('native_recipe', {}))


def generate(library, request, complete: Callable):
    """At most two model calls. Invalid outputs are never silently repaired/executed."""
    if not isinstance(request, dict) or set(request) - {'prompt', 'board_id', 'piece_budget'}:
        raise ValueError('Unexpected generation fields')
    prompt = _text(request.get('prompt'), 'prompt', 3000)
    board_id = request.get('board_id')
    if board_id not in library['boards']: raise ValueError('This original base is not installed')
    budget = integer(request.get('piece_budget', 12), 'piece_budget', 1, 12)
    catalog = [x for x in catalog_summary(library) if x['board_id'] == board_id]
    system = ('你是磁吸积木结构设计器。只输出 JSON。用户有至多12块活动件，固定底板不占库存。'
        '仅从给定原模板选择 selected_roles，保留每个选中件的所有父件和支撑件；'
        '不可新增坐标、旋转、材料、部件、URL、代码或运动指令。选择形成有意义外形的子结构。'
        '描述超出槽位空间时，explanation必须明确说明近似或不支持，不能把选择原样房屋谎称为任意新结构。'
        '输出严格五字段：board_id,template_id,title,selected_roles,explanation。'
        'explanation说明外形和限制，不得声称已经通过机器人或磁吸物理验证。')
    messages = [dict(role='system', content=system), dict(role='user', content=dumps(
        dict(request=prompt, board_id=board_id, piece_budget=budget, templates=catalog)))]
    errors = []
    for attempt in range(2):
        raw = complete(messages)
        try:
            proposal = loads(raw) if isinstance(raw, str) else raw
            result = compile_selection(library, proposal, board_id=board_id, piece_budget=budget)
            result.update(generation_method='llm_constrained_original_slots', model_calls=attempt + 1,
                          validation_errors=errors)
            return result
        except (ValueError, KeyError, TypeError) as exc:
            reason = str(exc)[:500]; errors.append(reason)
            if attempt == 0:
                messages += [dict(role='assistant', content=(raw[:12000] if isinstance(raw, str) else dumps(raw))),
                             dict(role='user', content='上个 JSON 未通过验证：' + reason + '。修正后只输出 JSON。')]
    raise ValueError('LLM returned no valid build description: ' + '; '.join(errors))
