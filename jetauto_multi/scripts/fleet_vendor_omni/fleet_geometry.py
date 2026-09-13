#!/usr/bin/env python
"""Shared map geometry and conservative route reservations (no ROS dependency)."""
from __future__ import print_function
import heapq
import math
import os
import numpy as np
import yaml

def distance(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/max(dx*dx+dy*dy,1e-12)))
    return distance(p,(a[0]+t*dx,a[1]+t*dy))

def point_path(p,path):
    return min(point_segment(p,a,b) for a,b in zip(path,path[1:])) if len(path)>1 else distance(p,path[0])

def segment_distance(a,b,c,d):
    def cross(p,q,r): return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    if cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0:
        return 0.
    return min(point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b))

def paths_conflict(first,second,clearance):
    return any(segment_distance(a,b,c,d)<clearance for a,b in zip(first,first[1:]) for c,d in zip(second,second[1:]))

class MapGrid(object):
    def __init__(self, values, resolution, origin, radius=.23):
        self.values=np.asarray(values)
        self.resolution=float(resolution)
        self.origin=tuple(origin[:2])
        self.height,self.width=self.values.shape
        # Unknown cells are not certified free space.
        occupied=(self.values!=0)
        self.blocked=occupied.copy()
        n=int(math.ceil(radius/self.resolution))
        for dy in range(-n,n+1):
            for dx in range(-n,n+1):
                if math.hypot(dx,dy)*self.resolution>radius+self.resolution*.71:
                    continue
                ya,yb=max(0,dy),min(self.height,self.height+dy)
                xa,xb=max(0,dx),min(self.width,self.width+dx)
                self.blocked[ya:yb,xa:xb] |= occupied[ya-dy:yb-dy,xa-dx:xb-dx]
        if n:
            self.blocked[:n,:]=True; self.blocked[-n:,:]=True
            self.blocked[:,:n]=True; self.blocked[:,-n:]=True

    @classmethod
    def load(cls,path,radius=.23):
        with open(path) as stream: meta=yaml.safe_load(stream)
        if abs(float(meta['origin'][2]))>1e-8:
            raise ValueError('Rotated map origin is not supported; normalize the map first')
        image=os.path.join(os.path.dirname(path),meta['image'])
        with open(image,'rb') as stream:
            def token():
                buf=b''
                while True:
                    ch=stream.read(1)
                    if not ch: return buf
                    if ch==b'#': stream.readline(); continue
                    if ch.isspace():
                        if buf: return buf
                    else: buf+=ch
            kind=token(); width=int(token()); height=int(token()); maximum=int(token())
            if kind!=b'P5' or maximum!=255:
                raise ValueError('Expected an 8-bit P5 PGM map')
            pixels=np.frombuffer(stream.read(width*height),dtype=np.uint8).reshape(height,width)[::-1]
        probability=pixels/255. if meta.get('negate',0) else 1.-pixels/255.
        values=np.full(pixels.shape,-1,dtype=np.int8)
        values[probability<float(meta['free_thresh'])]=0
        values[probability>float(meta['occupied_thresh'])]=100
        return cls(values,meta['resolution'],meta['origin'],radius)

    def cell(self,p):
        return (int(math.floor((p[0]-self.origin[0])/self.resolution)),int(math.floor((p[1]-self.origin[1])/self.resolution)))

    def world(self,c):
        return (self.origin[0]+(c[0]+.5)*self.resolution,self.origin[1]+(c[1]+.5)*self.resolution)

    def free(self,p,blocked=None):
        x,y=self.cell(p)
        return 0<=x<self.width and 0<=y<self.height and not (self.blocked if blocked is None else blocked)[y,x]

    def line_free(self,a,b,blocked=None):
        count=max(1,int(math.ceil(distance(a,b)/(self.resolution*.4))))
        return all(self.free((a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count),blocked) for i in range(count+1))

    def plan(self,start,goal,others=(),clearance=.55):
        blocked=self.blocked.copy()
        for other in others:
            cx,cy=self.cell(other); n=int(math.ceil(clearance/self.resolution))
            for y in range(max(0,cy-n),min(self.height,cy+n+1)):
                for x in range(max(0,cx-n),min(self.width,cx+n+1)):
                    if distance(self.world((x,y)),other)<clearance:
                        blocked[y,x]=True
        if not self.free(start,blocked) or not self.free(goal,blocked): return None
        if self.line_free(start,goal,blocked): return [list(start),list(goal)]
        source,target=self.cell(start),self.cell(goal)
        todo=[(0.,0.,source)]; costs={source:0.}; previous={}
        while todo:
            _score,g,current=heapq.heappop(todo)
            if g>costs[current]+1e-8: continue
            if current==target: break
            x,y=current
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
                nx,ny=x+dx,y+dy
                if nx<0 or nx>=self.width or ny<0 or ny>=self.height or blocked[ny,nx]: continue
                if dx and dy and (blocked[y,nx] or blocked[ny,x]): continue
                candidate=(nx,ny); cost=g+math.hypot(dx,dy)
                if cost>=costs.get(candidate,float('inf')): continue
                costs[candidate]=cost; previous[candidate]=current
                heapq.heappush(todo,(cost+distance(candidate,target),cost,candidate))
        else: return None
        raw=[target]
        while raw[-1]!=source: raw.append(previous[raw[-1]])
        path=[list(start)]+[self.world(c) for c in reversed(raw[1:-1])]+[list(goal)]
        result=[path[0]]; index=0
        while index<len(path)-1:
            end=len(path)-1
            while end>index+1 and not self.line_free(path[index],path[end],blocked): end-=1
            result.append(path[end]); index=end
        return [[round(x,4),round(y,4)] for x,y in result]

def assign_slots(names,poses,slots):
    """Exact minimum squared travel assignment for a small fleet (10 here)."""
    n=len(names)
    if n!=len(slots) or n>16: raise ValueError('slot assignment requires 1..16 matching robots')
    scores={0:(0.,[])}
    for index,name in enumerate(names):
        following={}
        for mask,(score,chosen) in scores.items():
            for j,slot in enumerate(slots):
                if mask&(1<<j): continue
                value=round(score+distance(poses[name],slot)**2,8)
                key=mask|(1<<j)
                candidate=(value,chosen+[j])
                if candidate<following.get(key,(float('inf'),[])): following[key]=candidate
        scores=following
    chosen=scores[(1<<n)-1][1]
    return dict((name,slots[j]) for name,j in zip(names,chosen))

def plan_transition(grid,starts,goals,clearance=.55,reservation=.65,order='shortest'):
    poses=dict(starts); pending=set(goals); tasks=[]
    while pending:
        choices=[]
        for name in sorted(pending):
            route=grid.plan(poses[name],goals[name],[p for n,p in poses.items() if n!=name],clearance)
            if route:
                length=sum(distance(a,b) for a,b in zip(route,route[1:]))
                priority=goals[name][0] if order=='rear_first' else (-goals[name][0] if order=='front_first' else length)
                outer=-abs(goals[name][1]-sum(p[1] for p in starts.values())/len(starts)) if order=='front_first' else 0.
                choices.append((round(priority,8),round(outer,8),round(length,8),name,route))
        if not choices:
            detail=[(n,poses[n],goals[n],round(min(distance(goals[n],p) for other,p in poses.items() if other!=n),3)) for n in sorted(pending)]
            raise ValueError('No collision-free relocation (robot/start/goal/nearest parked): %r; move the change area or alter spacing'%detail)
        _priority,_outer,length,name,route=min(choices)
        task_id='place_'+name
        tasks.append({'id':task_id,'robot':name,'path':route,'length':round(length,3),
                      'after':[t['id'] for t in tasks if paths_conflict(route,t['path'],reservation)]})
        poses[name]=goals[name]; pending.remove(name)
    return tasks
