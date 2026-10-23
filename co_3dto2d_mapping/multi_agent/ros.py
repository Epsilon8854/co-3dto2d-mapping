"""Shared message conversions; no node or executor is created on import."""
from copy import deepcopy
import json
import math
import numpy as np
from diagnostic_msgs.msg import DiagnosticStatus,KeyValue
from nav_msgs.msg import OccupancyGrid
from co_3dto2d_mapping.multi_agent.core import Snapshot,json_safe,transform_xy
from co_3dto2d_mapping.common.occupancy_submap import GridGeometry

def stamp_ns(stamp):return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
def yaw_of(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose_of(msg):
    p=msg.pose.pose
    return float(p.position.x),float(p.position.y),yaw_of(p.orientation)
def values_of(msg):
    result={}
    for item in msg.status:
        for value in item.values:
            try:result[value.key]=json.loads(value.value)
            except (ValueError,TypeError):result[value.key]=value.value
    return result
def snapshot_of(msg):
    g=msg.info;p=g.origin
    return Snapshot(np.asarray(msg.data,dtype=np.int8).reshape(g.height,g.width),
        GridGeometry(g.resolution,g.width,g.height,p.position.x,p.position.y,yaw_of(p.orientation)),
        stamp_ns(msg.header.stamp),msg.header.frame_id)
def grid_message(snapshot):
    msg=OccupancyGrid();g=snapshot.geometry
    msg.header.frame_id=snapshot.frame
    msg.header.stamp.sec=snapshot.stamp_ns//1_000_000_000;msg.header.stamp.nanosec=snapshot.stamp_ns%1_000_000_000
    msg.info.resolution=g.resolution;msg.info.width=g.width;msg.info.height=g.height
    msg.info.origin.position.x=g.origin_x;msg.info.origin.position.y=g.origin_y
    msg.info.origin.orientation.z=math.sin(g.origin_yaw/2);msg.info.origin.orientation.w=math.cos(g.origin_yaw/2)
    msg.data=snapshot.grid.ravel().tolist()
    return msg
def diagnostic(name,values,level=DiagnosticStatus.OK,message=''):
    item=DiagnosticStatus(name=name,level=level,message=message)
    item.values=[KeyValue(key=str(k),value=json.dumps(json_safe(v),ensure_ascii=False)) for k,v in values.items()]
    return item
def transformed_odometry(msg,rid,transform,frame='r0/odom',child_frame=None):
    out=deepcopy(msg);p=out.pose.pose
    p.position.x,p.position.y=map(float,transform_xy([[p.position.x,p.position.y]],transform)[0])
    c,s=math.cos(transform[2]/2),math.sin(transform[2]/2);q=deepcopy(p.orientation)
    p.orientation.x=c*q.x-s*q.y;p.orientation.y=s*q.x+c*q.y
    p.orientation.z=c*q.z+s*q.w;p.orientation.w=c*q.w-s*q.z
    out.header.frame_id=frame;out.child_frame_id=child_frame or f'r{rid}/mapping_base_link'
    r=np.eye(6);c,s=math.cos(transform[2]),math.sin(transform[2]);r[:2,:2]=[[c,-s],[s,c]];r[3:5,3:5]=r[:2,:2]
    out.pose.covariance=(r@np.asarray(msg.pose.covariance).reshape(6,6)@r.T).ravel().tolist()
    return out
