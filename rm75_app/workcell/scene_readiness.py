"""Read-only scene hints. Selected generated jobs still require exact hash checks."""
from pathlib import Path
import re


def template_scene_hint(profile):
    """Check library validity and bounded file presence, not GPU/physical success.

    Do not hash all scenes on every bootstrap poll; the selected recipe is hashed
    by ConsoleAPI.preflight before submission, as before this display fix.
    """
    from rm75_app.magnetic.generation import load_library
    try:
        library = load_library(profile)
        templates = [t for b in library['boards'].values() for t in b['templates']]
        usable = 0
        for template in templates:
            recipe = template.get('native_recipe', {})
            path = recipe.get('fixed_scene')
            expected = recipe.get('fixed_scene_sha256')
            if (isinstance(path, str) and path and isinstance(expected, str)
                    and re.fullmatch(r'[0-9a-f]{64}', expected)):
                p = Path(path).expanduser()
                if p.is_file() and p.stat().st_size <= 8_000_000:
                    usable += 1
    except (OSError, ValueError, KeyError, TypeError):
        templates, usable = [], 0
    if not usable:
        return None
    return dict(key='magnetic.scene', title='原模板冻结场景',
                status='ready' if usable == len(templates) else 'warning',
                hint=f'已找到 {usable}/{len(templates)} 个模板场景；提交时校验所选场景摘要。'
                     '不带来源证明的手动设计仍需单独配置 fixed_scene。')
