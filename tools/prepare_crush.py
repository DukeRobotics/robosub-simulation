"""Derive a compact visual mesh, collision box and principal inertia from the CAD export.

Run in the simulation image (NumPy and PyYAML are required). Source files stay unchanged.
"""

import argparse
import hashlib
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


def write_obj(path, vertices, faces, *, normals=True):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as stream:
        stream.write('# Generated from the Crush CAD export; metres, +X forward, +Y right, +Z down.\n')
        for vertex in vertices:
            stream.write('v ' + ' '.join(f'{value:.7g}' for value in vertex) + '\n')
        if not normals:
            for face in faces:
                stream.write('f ' + ' '.join(str(int(index) + 1) for index in face) + '\n')
            return
        normals = np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]], vertices[faces[:, 2]] - vertices[faces[:, 0]])
        lengths = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(lengths[:, None], 1e-12)
        normals[lengths < 1e-12] = [0, 0, 1]
        for normal in normals:
            stream.write('vn ' + ' '.join(f'{value:.7g}' for value in normal) + '\n')
        for normal_index, face in enumerate(faces, start=1):
            stream.write('f ' + ' '.join(f'{int(index) + 1}//{normal_index}' for index in face) + '\n')


def box_faces():
    # Vertices are the Cartesian product of low/high X, Y, Z. Outward winding.
    return np.array(
        [
            [0, 1, 3],
            [0, 3, 2],
            [4, 6, 7],
            [4, 7, 5],
            [0, 4, 5],
            [0, 5, 1],
            [2, 3, 7],
            [2, 7, 6],
            [0, 2, 6],
            [0, 6, 4],
            [1, 5, 7],
            [1, 7, 3],
        ]
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--max-faces', type=int, default=60000)
    parser.add_argument('--normals-only', action='store_true', help='Add normals to already generated meshes')
    args = parser.parse_args()
    root = args.root
    if args.normals_only:
        for path in (root / 'src/robosub_simulation/meshes').glob('*.obj'):
            vertices, faces = [], []
            for line in path.read_text().splitlines():
                if line.startswith('v '):
                    vertices.append([float(value) for value in line.split()[1:]])
                elif line.startswith('f '):
                    faces.append([int(value.split('/')[0]) - 1 for value in line.split()[1:]])
            write_obj(path, np.array(vertices), np.array(faces), normals=path.name != 'crush_collision.obj')
        return
    source = root / 'assets/crush/full_model_assembly'
    mesh_path = source / 'meshes/base_link.STL'
    raw = mesh_path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + count * 50:
        raise ValueError('Expected the supplied binary STL in metres')
    facets = np.frombuffer(
        raw,
        dtype=np.dtype(
            [
                ('normal', '<f4', (3,)),
                ('vertices', '<f4', (3, 3)),
                ('attribute', '<u2'),
            ]
        ),
        offset=84,
        count=count,
    )
    vertices = facets['vertices'].reshape(-1, 3).astype(float) * [1, -1, -1]
    bounds = np.array([vertices.min(axis=0), vertices.max(axis=0)])
    if not np.all(np.isfinite(vertices)) or not np.all((bounds[1] - bounds[0] > 0.05) & (bounds[1] - bounds[0] < 2)):
        raise ValueError(f'Unexpected mesh dimensions in metres: {bounds.tolist()}')
    cell = 0.003
    while True:
        _, inverse = np.unique(np.round(vertices / cell).astype(np.int32), axis=0, return_inverse=True)
        faces = inverse.reshape(-1, 3)
        faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
        _, unique = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
        faces = faces[unique]
        if len(faces) <= args.max_faces:
            break
        cell *= 1.25
    weights = np.bincount(inverse)
    compact = np.column_stack([np.bincount(inverse, weights=vertices[:, axis]) / weights for axis in range(3)])
    used, remap = np.unique(faces, return_inverse=True)
    target = root / 'src/robosub_simulation/meshes'
    write_obj(target / 'crush_visual.obj', compact[used], remap.reshape(-1, 3))
    corners = np.array([[x, y, z] for x in bounds[:, 0] for y in bounds[:, 1] for z in bounds[:, 2]])
    # Keep eight shared physical vertices: per-face normals split vertices in Stonefish's
    # OBJ loader and change the point-cloud fit used for hydrodynamic added inertia.
    write_obj(target / 'crush_collision.obj', corners, box_faces(), normals=False)
    # A small box serves as the rotating propeller visual; its axis is local +X.
    propeller = np.array([[x, y, z] for x in (-0.006, 0.006) for y in (-0.035, 0.035) for z in (-0.008, 0.008)])
    write_obj(target / 'propeller.obj', propeller, box_faces())

    inertial = ET.parse(source / 'urdf/Full Model Assembly (1).urdf').find('link/inertial')
    inertia = inertial.find('inertia').attrib
    tensor = np.array([[float(inertia['i' + a + b if a <= b else 'i' + b + a]) for b in 'xyz'] for a in 'xyz'])
    conversion = np.diag([1, -1, -1])
    tensor = conversion @ tensor @ conversion
    moments, basis = np.linalg.eigh(tensor)
    if np.linalg.det(basis) < 0:
        basis[:, 0] *= -1
    rpy = [math.atan2(basis[2, 1], basis[2, 2]), math.asin(-basis[2, 0]), math.atan2(basis[1, 0], basis[0, 0])]
    com = np.fromstring(inertial.find('origin').get('xyz'), sep=' ') * [1, -1, -1]
    model = {
        'mass': float(inertial.find('mass').get('value')),
        'center_of_mass': com.tolist(),
        'principal_inertia': moments.tolist(),
        'principal_rpy': rpy,
        'inertia_tensor': tensor.tolist(),
        'bounds': bounds.tolist(),
        'source_triangles': count,
        'visual_triangles': len(faces),
        'cluster_size': cell,
        'source_mesh_sha256': hashlib.sha256(raw).hexdigest(),
    }
    (root / 'src/robosub_simulation/config/crush_model.yaml').write_text(yaml.safe_dump(model, sort_keys=False))
    print(yaml.safe_dump(model, sort_keys=False))


if __name__ == '__main__':
    main()
