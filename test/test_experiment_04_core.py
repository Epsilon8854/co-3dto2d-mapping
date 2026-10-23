"""Behavioral geometry and lifecycle checks for Experiment 4 (no ROS needed)."""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from co_3dto2d_mapping.multi_agent.core import Snapshot,compose_grids,StationaryConsensus,save_map,transform_xy
from co_3dto2d_mapping.multi_agent.core import SnapshotConsensus,OccupancyAccumulator,IncrementalFusion
from co_3dto2d_mapping.common.occupancy_submap import GridGeometry,extract_local_patch
from co_3dto2d_mapping.multi_agent.alignment import register_startup,StartupConfig
from co_3dto2d_mapping.common.se2_map_registration import RegistrationConfig
from co_3dto2d_mapping.common.planar_transform_utils import invert_planar,normalize_angle


def scene(symmetric=False):
    grid=np.zeros((81,101),dtype=np.int8)
    grid[10:70,10]=100;grid[10:70,89]=100
    grid[10,10:90]=100;grid[69,10:90]=100
    if not symmetric:
        grid[21:48,39]=100;grid[47,39:60]=100;grid[54:61,65:72]=100
    return Snapshot(grid,GridGeometry(.1,101,81,-5.,-4.),1_000_000_000,'r0/odom')


def patches(angle,translation=(.4,-.3),symmetric=False):
    target=scene(symmetric)
    truth=(*translation,math.radians(angle));inverse=invert_planar(truth)
    origin=transform_xy([[target.geometry.origin_x,target.geometry.origin_y]],inverse)[0]
    source=Snapshot(target.grid,GridGeometry(.1,101,81,*origin,inverse[2]),target.stamp_ns,'r1/odom')
    return (extract_local_patch(target.grid,target.geometry,(0.,0.,0.),8.),
            extract_local_patch(source.grid,source.geometry,(0.,0.,0.),8.),truth)


def test_rotated_occupied_cell_preserves_area_and_wins_free_conflict():
    occupied=Snapshot([[100]],GridGeometry(1.,1,1),1,'r1/odom')
    free=Snapshot(np.zeros((5,5),dtype=np.int8),GridGeometry(.5,5,5,-1.,-1.),1,'r0/odom')
    merged=compose_grids([(free,(0,0,0)),(occupied,(0,0,math.pi/4))],.5)
    # Rotated 1m square occupies multiple 0.5m output cells, not only its center.
    assert np.count_nonzero(merged.grid==100)>=6
    assert np.count_nonzero(merged.grid==0)>0
    assert not occupied.grid.flags.writeable


def test_merging_is_order_independent_and_rebuilt_without_ghosts():
    a=Snapshot([[100]],GridGeometry(1.,1,1),1,'r0/odom')
    b=Snapshot([[0]],GridGeometry(1.,1,1),2,'r1/odom')
    first=compose_grids([(a,(0,0,0)),(b,(0,0,0))],1.)
    swapped=compose_grids([(b,(0,0,0)),(a,(0,0,0))],1.)
    assert np.array_equal(first.grid,swapped.grid)
    cleared=compose_grids([(b,(0,0,0))],1.)
    assert not np.any(cleared.grid==100)


def test_consensus_requires_disjoint_windows_and_circular_yaw():
    c=StationaryConsensus(3,.1,1.)
    a=(0.,0.,math.radians(179.9));b=(0.,0.,math.radians(-179.9))
    assert c.add(a,(0,30,0,30)) is None
    assert c.add(a,(0,30,0,30)) is None
    assert len(c.evidence)==1
    assert c.add(b,(31,60,31,60)) is None
    final=c.add(a,(61,90,61,90))
    assert final is not None and abs(abs(final[2])-math.pi)<.01
    c.reset();assert not c.evidence


def test_conflicting_consensus_does_not_lock():
    c=StationaryConsensus(2,.1,1.)
    c.add((0.,0.,0.),(0,10,0,10))
    assert c.add((2.,0.,0.),(11,20,11,20)) is None
    assert len(c.evidence)==1


def test_single_snapshot_confirms_first_finite_transform():
    c=SnapshotConsensus(1,.1,1.)
    assert c.add((float('nan'),0.,0.),(10,20)) is None
    assert not c.evidence
    assert c.add((.4,-.3,.2),(10,20))==pytest.approx((.4,-.3,.2))


def test_experiment04_profiles_use_one_confirmation():
    root=Path(__file__).resolve().parents[1]/'config/experiment/experiment_04_multi_agent_mapping'
    for path in root.glob('alignment*.yaml'):
        params=yaml.safe_load(path.read_text())['/**']['ros__parameters']
        assert params['confirmation_windows']==1
        assert params['registration.max_free_conflict_ratio'] in (.14,.18)


def test_exact_map_round_trip_and_navigation_origin(tmp_path):
    source=scene();save_map(tmp_path,'merged_global_map',source,{'r0':[(0.,0.),(1.,1.)]})
    with np.load(tmp_path/'merged_global_map_data.npz') as loaded:
        assert np.array_equal(loaded['data'],source.grid)
        assert loaded['stamp_ns']==source.stamp_ns
        assert loaded['frame']=='r0/odom'
    metadata=yaml.safe_load((tmp_path/'merged_global_map.yaml').read_text())
    assert metadata['origin']==[-5.,-4.,0.]
    assert metadata['resolution']==.1
    assert (tmp_path/'merged_global_map.pgm').read_bytes().startswith(b'P5\n101 81\n255\n')
    assert (tmp_path/'merged_global_map_with_paths.png').read_bytes().startswith(b'\x89PNG')


@pytest.mark.parametrize('angle',[0,30,-60,90,135,180])
def test_full_circle_recovers_yaw_and_translation(angle):
    target,source,truth=patches(angle)
    reg=RegistrationConfig(coarse_translation_range_m=1.,coarse_translation_step_m=.25,
        search_max_points=180,icp_max_points=1200,min_correspondences=50,
        min_symmetric_overlap=.5,max_symmetric_rmse_m=.18)
    report=register_startup(target,source,reg,StartupConfig())
    assert report['accepted'],report
    assert math.hypot(report['transform'][0]-truth[0],report['transform'][1]-truth[1])<.10
    assert abs(normalize_angle(report['transform'][2]-truth[2]))<math.radians(1.)


def test_symmetric_rectangle_is_not_false_locked():
    target,source,_=patches(90,translation=(0.,0.),symmetric=True)
    reg=RegistrationConfig(coarse_translation_range_m=.5,search_max_points=120,min_correspondences=40)
    result=register_startup(target,source,reg)
    assert not result['accepted']
    assert result['reason']=='ambiguous_yaw_or_translation'


def test_cumulative_snapshot_consensus_rejects_repeated_source_stamps():
    c=SnapshotConsensus(3,.1,1.)
    assert c.add((0,0,0),(10,10)) is None
    assert c.add((0,0,0),(10,20)) is None
    assert len(c.evidence)==1
    assert c.add((.01,0,0),(20,20)) is None
    assert c.add((0,0,0),(30,30)) is not None


def test_incremental_updates_expand_preserve_unknown_and_keep_static_occupied():
    a=OccupancyAccumulator(.1)
    a.add(Snapshot([[100,0]],GridGeometry(.1,2,1),1,'r0/odom'))
    a.add(Snapshot([[0,-1,100]],GridGeometry(.1,3,1),2,'r0/odom'))
    result=a.snapshot()
    assert result.grid.tolist()==[[100,0,100]]
    assert result.stamp_ns==2
    with pytest.raises(ValueError):a.add(Snapshot([[0]],GridGeometry(.2,1,1),3,'r0/odom'))
    tiny=OccupancyAccumulator(.1,max_cells=1)
    with pytest.raises(ValueError):tiny.add(result)


def test_incremental_map_matches_conservative_union_without_double_pose():
    seeds={0:Snapshot([[0,100]],GridGeometry(.1,2,1),1,'r0/odom'),
           1:Snapshot([[100]],GridGeometry(.1,1,1),1,'r1/odom')}
    local=Snapshot([[100,0]],GridGeometry(.1,2,1,1.,0.),2,'r1/odom');alignment=(.3,-.2,math.pi/2)
    engine=IncrementalFusion(.1)
    engine.update(0,seeds,[],alignment)
    merged,layers=engine.update(0,{},[(1,local)],alignment)
    expected=compose_grids([(seeds[0],(0,0,0)),(seeds[1],alignment),(local,alignment)],.1)
    def occupied(snapshot):
        rows,cols=np.nonzero(snapshot.grid>50);g=snapshot.geometry
        return set(zip(np.round(g.origin_x+(cols+.5)*g.resolution,5),np.round(g.origin_y+(rows+.5)*g.resolution,5)))
    assert occupied(merged)==occupied(expected)
    assert layers[1].frame=='r0/odom'
    reset,_=engine.update(1,{0:Snapshot([[0]],GridGeometry(.1,1,1),3,'r0/odom')},[],alignment)
    assert not np.any(reset.grid==100)


def test_cached_verification_keeps_competing_modes_and_recovers_transform():
    target,source,truth=patches(90)
    reg=RegistrationConfig(coarse_translation_range_m=1.,search_max_points=150,min_correspondences=50)
    full=register_startup(target,source,reg)
    assert full['accepted']
    cached=register_startup(target,source,reg,candidate_priors=[c['transform'] for c in full['candidates']])
    assert cached['accepted'] and cached['search_mode']=='cached_modes'
    assert cached['coarse_yaw_count']==0
    assert abs(normalize_angle(cached['transform'][2]-truth[2]))<math.radians(1.)
    assert len(cached['candidates'])>=2
