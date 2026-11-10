"""Strict reader for kinematic_wrench_task_envelope_v1 (no robot/OpenSim imports).
Only implemented semantics are accepted; no extrapolation or inferred IE/VV axes.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

def transform(x):
 x=np.asarray(x,float)
 if x.shape!=(4,4) or not np.isfinite(x).all() or not np.allclose(x[3],[0,0,0,1],atol=1e-10,rtol=0) or not np.allclose(x[:3,:3].T@x[:3,:3],np.eye(3),atol=1e-8,rtol=0) or abs(np.linalg.det(x[:3,:3])-1)>1e-8:raise ValueError('Invalid rigid transform')
 return x

class TaskEnvelope:
 def __init__(self,folder):
  self.folder=Path(folder);self.meta=json.loads((self.folder/'task.json').read_text())
  m=self.meta
  if m['schema']!='kinematic_wrench_task_envelope_v1':raise ValueError('Unsupported task schema')
  if m['wrench_frame']!='T' or m['wrench_origin']!='T origin' or m['wrench_action']!='robot_on_tibia' or m['wrench_order']!=['Fx','Fy','Fz','Mx','My','Mz']:raise ValueError('Unsupported wrench convention')
  if m['motion_interpolation']!='linear translation and rotation SLERP between saved samples' or m['wrench_interpolation']!='constant per-case vectors in T':raise ValueError('Unsupported interpolation')
  if m['wrench_semantics']!='alternative signed peak capacity, not simultaneous loads or commanded waveform':raise ValueError('Unsupported load semantics')
  path=(self.folder/m['arrays_file']).resolve()
  if path.parent!=self.folder.resolve():raise ValueError('Task arrays must be in envelope folder')
  if hashlib.sha256(path.read_bytes()).hexdigest()!=m['task_arrays_sha256']:raise ValueError('Envelope array hash differs')
  with np.load(path,allow_pickle=False) as z:
   self.time_s=np.array(z['time_s'],float);self.X_FT=np.array(z['X_FT'],float);self.wrenches=np.array(z['wrench_cases_T'],float);raw=np.array(z['active_cases']);self.X_GF_reference=np.array(z['X_GF_reference'],float)
   self.angles_rad=np.array(z['angles_rad'],float) if 'angles_rad' in z else None
  t=self.time_s
  if t.ndim!=1 or len(t)<2 or not np.isfinite(t).all() or np.any(np.diff(t)<=0) or self.X_FT.shape!=(len(t),4,4):raise ValueError('Invalid task chronology/poses')
  for x in self.X_FT:transform(x)
  transform(self.X_GF_reference)
  if self.wrenches.ndim!=2 or self.wrenches.shape[1]!=6 or not np.isfinite(self.wrenches).all() or raw.shape!=(len(t),len(self.wrenches)) or not np.isin(raw,[0,1]).all():raise ValueError('Invalid load arrays')
  self.active=raw.astype(bool);self.phases=m['phases'];self.groups=tuple(m['groups'])
  if not self.phases or len(self.groups)!=len(set(self.groups)):raise ValueError('Invalid phase/groups')
  last=float(t[0])
  for ph in self.phases:
   if ph['kind'] not in ('station','transfer'):raise ValueError('Unsupported phase kind')
   lo,hi=float(ph['start_time_s']),float(ph['end_time_s'])
   if not np.isfinite([lo,hi]).all() or hi<=lo or abs(lo-last)>1e-9:raise ValueError('Phase gap,overlap or order invalid')
   last=hi
   if ph['kind']=='station' and ph['group_id'] not in self.groups:raise ValueError('Station group absent')
   if ph['kind']=='transfer' and (ph['group_id'] is not None or ph['active_case_indices']):raise ValueError('Transfer semantics invalid')
   for k in ph['active_case_indices']:
    if not isinstance(k,int) or isinstance(k,bool) or not 0<=k<len(self.wrenches):raise ValueError('Invalid case index')
   for key in ('position_tolerance_m','orientation_tolerance_rad'):
    if not np.isfinite(ph[key]) or ph[key]<=0:raise ValueError('Invalid tracking tolerance')
  if abs(last-t[-1])>1e-9:raise ValueError('Phase coverage incomplete')
  derived=np.zeros_like(self.active)
  for i,v in enumerate(t):derived[i,list(self.case_indices(v))]=True
  if not np.array_equal(derived,self.active):raise ValueError('Saved activation disagrees with phase semantics')
  self._rotation=Slerp(t,Rotation.from_matrix(self.X_FT[:,:3,:3]))
  for arr in (self.time_s,self.X_FT,self.wrenches,self.active,self.X_GF_reference):arr.setflags(write=False)
 def _time(self,v):
  v=float(v)
  if not np.isfinite(v) or v<self.time_s[0] or v>self.time_s[-1]:raise ValueError('Time outside task; no extrapolation')
  return v
 def phases_at(self,v):
  v=self._time(v);ph=[p for p in self.phases if p['start_time_s']-1e-9<=v<=p['end_time_s']+1e-9]
  if not ph:raise ValueError('No phase at requested time')
  return ph
 def case_indices(self,v):return tuple(sorted({i for ph in self.phases_at(v) for i in ph['active_case_indices']}))
 def groups_at(self,v):return tuple(sorted({ph['group_id'] for ph in self.phases_at(v) if ph['group_id'] is not None}))
 def tolerances(self,v):
  ph=self.phases_at(v);return min(x['position_tolerance_m'] for x in ph),min(x['orientation_tolerance_rad'] for x in ph)
 def pose_at(self,v):
  v=self._time(v);x=np.eye(4);x[:3,:3]=self._rotation(v).as_matrix();x[:3,3]=[np.interp(v,self.time_s,self.X_FT[:,j,3]) for j in range(3)];return x
 def required_ee(self,v,X_BF,A):return transform(X_BF)@self.pose_at(v)@np.linalg.inv(transform(A))
 def planning_times(self,stride_s):
  if not np.isfinite(stride_s) or stride_s<=0:raise ValueError('Invalid stride')
  return np.unique(np.r_[np.arange(self.time_s[0],self.time_s[-1],stride_s),self.time_s[-1],[p[k] for p in self.phases for k in ('start_time_s','end_time_s')]])
