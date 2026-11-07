"""Manifest-local surface sharing; exact coordinates/source/error, no mesh merging.

The native manifest binds definitions and references into its existing digest.
These helpers only transform transport; standalone operation dictionaries stay inline.
"""

import json


def _slots(manifest):
    for op in manifest.get('operations', ()):
        key = 'target' if op.get('kind') == 'roughing' else 'surface'
        if op.get('kind') in ('roughing', 'parallel', 'waterline', 'flat', 'deburr') and key in op:
            yield op, key
    config = manifest.get('verification') or {}
    for check in config.get('material', ()):
        target = check['target']
        if 'surface' in target:
            yield target, 'surface'


def share_geometry(manifest):
    """Snapshot containers; intern equal wire meshes, preserving all metadata.

    Never mutate an authored dictionary. Equal meshes may be separate Python
    objects. JSON spelling distinguishes signed zero; no rounded geometry keys.
    The native reader validates the original mesh and resolves bounded references.
    """
    result = dict(manifest)
    if 'operations' in result:
        result['operations'] = [dict(op) for op in result['operations']]
    if result.get('verification') is not None:
        config = result['verification'] = dict(result['verification'])
        config['material'] = [dict(check, target=dict(check['target'])) for check in config.get('material', ())]
    slots = list(_slots(result))
    if not slots:
        return result
    geometry, lookup = [], {}
    for container, key in slots:
        mesh = container[key]
        facets = {k: mesh[k] for k in ('triangles', 'vertices', 'indices') if k in mesh}
        encoded = json.dumps(facets, allow_nan=False, sort_keys=True, separators=(',', ':'))
        index = lookup.get(encoded)
        if index is None:
            index = len(geometry)
            if index >= 129:
                raise ValueError('shared geometry exceeds manifest limit')
            lookup[encoded] = index
            geometry.append(facets)
        container[key] = {k: v for k, v in mesh.items() if k not in facets}
        container[key]['geometry'] = index
    result['geometry'] = geometry
    return result


def resolve_geometry(manifest):
    """Expose ordinary inline definitions from a freshly decoded manifest.

    Uses share coordinate/index arrays; each keeps its own source/error metadata.
    The returned dictionary is detached from the retained canonical bytes.
    """
    geometry = manifest.pop('geometry', None)
    if geometry is None:
        return manifest
    if not isinstance(geometry, list):
        raise ValueError('invalid shared geometry table')
    used = set()
    for container, key in _slots(manifest):
        value = container[key]
        if isinstance(value, dict) and 'geometry' in value:
            index = value['geometry']
            if (
                set(value) != {'geometry', 'version', 'units', 'surface_error_mm', 'source'}
                or type(index) is not int
                or not 0 <= index < len(geometry)
            ):
                raise ValueError('invalid shared geometry reference')
            container[key] = {k: v for k, v in value.items() if k != 'geometry'}
            container[key].update(geometry[index])
            used.add(index)
    if len(used) != len(geometry):
        raise ValueError('unused shared geometry definition')
    return manifest
