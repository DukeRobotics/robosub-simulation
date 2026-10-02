"""Check pool boundaries, config validation and the reproducible default scene."""

import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import pytest
from robosub_simulation.pool import PoolConfig, build_scenario, load_config


@pytest.mark.parametrize('length,width,depth', [(25, 15, 3), (10, 5, 2), (50, 25, 5)])
def test_interior_boundaries(length, width, depth):
    config = PoolConfig(length=length, width=width, depth=depth)
    root = build_scenario(config).getroot()
    bounds = {}
    for box in root.findall('static'):
        size = [float(x) for x in box.find('dimensions').get('xyz').split()]
        centre = [float(x) for x in box.find('world_transform').get('xyz').split()]
        bounds[box.get('name')] = tuple((p - d / 2, p + d / 2) for p, d in zip(centre, size))
    assert len(bounds) == 5
    assert bounds['pool_floor'][2][0] == pytest.approx(depth)
    assert bounds['pool_wall_north'][0][0] == pytest.approx(length / 2)
    assert bounds['pool_wall_south'][0][1] == pytest.approx(-length / 2)
    assert bounds['pool_wall_east'][1][0] == pytest.approx(width / 2)
    assert bounds['pool_wall_west'][1][1] == pytest.approx(-width / 2)
    for name in ('north', 'south', 'east', 'west'):
        assert bounds[f'pool_wall_{name}'][2] == pytest.approx((-config.freeboard, depth))
    # Adjacent walls meet at each corner and the floor spans their entire footprint.
    assert bounds['pool_wall_east'][0][1] == pytest.approx(bounds['pool_wall_north'][0][0])
    assert bounds['pool_wall_west'][0][0] == pytest.approx(bounds['pool_wall_south'][0][1])
    assert bounds['pool_floor'][0][1] == pytest.approx(bounds['pool_wall_north'][0][1])
    assert bounds['pool_floor'][1][0] == pytest.approx(bounds['pool_wall_west'][1][0])
    assert root.find('environment/ocean/waves').get('height') == '0.0'
    assert root.find('environment/ocean/current') is None


@pytest.mark.parametrize(
    'name,value',
    [
        ('length', 0),
        ('depth', -1),
        ('width', float('nan')),
        ('wall_thickness', float('inf')),
        ('floor_thickness', 0),
        ('freeboard', -1),
        ('water_density', 0),
        ('jerlov', 1.1),
        ('latitude', 91),
        ('longitude', -181),
        ('length', '25'),
        ('length', True),
    ],
)
def test_invalid_config(name, value):
    with pytest.raises(ValueError):
        replace(PoolConfig(), **{name: value})


def test_unknown_setting(tmp_path):
    config = tmp_path / 'pool.yaml'
    config.write_text('lenght: 25\n')
    with pytest.raises(ValueError, match='Unknown pool settings'):
        load_config(config)


def test_default_scene_matches_config():
    package = Path(__file__).resolve().parents[1]
    config = load_config(package / 'config' / 'pool.yaml')
    expected = build_scenario(config).getroot()
    actual = ET.parse(package / 'scenarios' / 'pool.scn').getroot()
    assert ET.tostring(actual) == ET.tostring(ET.fromstring(ET.tostring(expected)))


def test_dimension_override():
    package = Path(__file__).resolve().parents[1]
    config = load_config(package / 'config' / 'pool.yaml', depth=5)
    assert config.depth == 5
    assert config.length == 25
