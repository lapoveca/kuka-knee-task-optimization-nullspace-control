import argparse, csv, hashlib, json, struct, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
import pinocchio as pin
from robot_model import RobotModel
from task_envelope import TaskEnvelope
from tibia_collision import TibiaCollisionChecker
from transforms import validate_transform

def read(p): return json.loads(p.read_text())
def table(p):
    with p.open(newline='') as f: return list(csv.DictReader(f))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def require(ok,message):
    if not ok: raise ValueError(message)
def matrix(v):
    v=np.asarray(v,float);require(v.shape==(4,4) and np.isfinite(v).all(),'Expected finite 4x4 transform');validate_transform(v);return v
def stl(path):
    raw=Path(path).read_bytes()
    if len(raw)>=84:
        n=struct.unpack_from('<I',raw,80)[0]
        if len(raw)==84+n*50:
            d=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
            return np.frombuffer(raw,dtype=d,count=n,offset=84)['vertices'].astype(float)
    out=[]
    for line in raw.decode('ascii').splitlines():
        w=line.split()
        if w and w[0].lower()=='vertex':out.append([float(x) for x in w[1:]])
    return np.asarray(out,float).reshape(-1,3,3)
def move(points,T):return points@T[:3,:3].T+T[:3,3]
def cylinder():
    theta=np.linspace(0,2*np.pi,49);faces=[]
    for a,b in zip(theta[:-1],theta[1:]):
        p=np.array([[0,.03*np.cos(a),.03*np.sin(a)],[.31,.03*np.cos(a),.03*np.sin(a)],[.31,.03*np.cos(b),.03*np.sin(b)],[0,.03*np.cos(b),.03*np.sin(b)]])
        faces.extend([p[[0,1,2]],p[[0,2,3]],np.array([[0,0,0],p[0],p[3]]),np.array([[.31,0,0],p[2],p[1]])])
    return np.asarray(faces)
def frame(ax,T,label,length=.06,dashed=False):
    p=T[:3,3]
    for i,c in enumerate(('red','green','blue')):ax.plot(*np.stack([p,p+length*T[:3,i]]).T,color=c,ls='--' if dashed else '-',lw=1.7)
    ax.text(*p,label,fontsize=7)
def equal(ax,points):
    lo,hi=np.min(points,0),np.max(points,0);center=(lo+hi)/2;half=max(float(np.max(hi-lo))*.57,.06)
    ax.set_xlim(center[0]-half,center[0]+half);ax.set_ylim(center[1]-half,center[1]+half);ax.set_zlim(center[2]-half,center[2]+half);ax.set_box_aspect((1,1,1));ax.set_proj_type('ortho');ax.view_init(24,-55)

def main():
    ap=argparse.ArgumentParser(description='Read-only 3D campaign atlas for a completed ranked VV campaign')
    ap.add_argument('campaign',type=Path);ap.add_argument('output',type=Path);ap.add_argument('--top',type=int,default=5)
    ap.add_argument('--frames',type=int,default=60);ap.add_argument('--fps',type=int,default=10);ap.add_argument('--no-gif',action='store_true')
    a=ap.parse_args();r=a.campaign.resolve();o=a.output.resolve()
    require(a.top>0 and not o.exists() and not o.is_relative_to(r),'Use a new output directory outside the campaign and positive --top')
    m=read(r/'campaign_manifest.json');summary=read(r/'campaign_summary.json');require(summary['status']=='complete','Expected a completed ranked campaign')
    rank=r/'ranking_guarded';rm=read(rank/'ranking_manifest.json');d=table(rank/'design_ranking.csv');p=table(rank/'path_scores.csv')
    cp=r/'collision_checks.json';c=read(cp)['results'];require(sha(cp)==rm['collision_report_sha256'],'Collision report hash mismatch')
    pool=read(r/'generation/proposals.json');g={x['id']:x for x in pool['designs']};screen={x['design_id']:x for x in read(r/'screening/summary.json')['results']};jobs={x['design_id']:x for x in read(r/'evaluation/summary.json')['jobs']}
    passing={k for k,x in screen.items() if x['status']=='pointwise_screen_pass'};audited={k for k,x in jobs.items() if x['status']=='audited_path_found'};accepted={str(Path(x['path']).parent):x for x in c if x['result']['sampled_pass']};eligible=[x for x in p if x['eligible']=='True'];best={}
    for x in eligible:
        k=x['design_id'];key=lambda z:(float(z['score']),z['path'])
        if k not in best or key(x)>key(best[k]):best[k]=x
    expected=sorted(best.values(),key=lambda x:(-float(x['score']),x['design_id']))
    require(set(g)==set(screen) and passing==set(jobs) and set(accepted)=={x['path'] for x in p} and len(accepted)==len(p),'Stage coverage differs')
    require(all(x['path_sha256']==accepted[x['path']]['path_sha256'] and x['design_id']==accepted[x['path']]['design_id'] for x in p),'Scored path identity differs')
    require([{k:v for k,v in x.items() if k!='rank'} for x in d]==expected and [int(x['rank']) for x in d]==list(range(1,len(d)+1)),'Ranking differs from eligible path scores')
    require([len(g),len(passing),len(audited),len(c),len(p),len(d),len(eligible)]==[summary[k] for k in ('candidate_designs','screen_passes')]+[summary['evaluation_counts']['audited_path_found']]+[summary[k] for k in ('audited_paths','collision_accepted_paths','ranked_designs')]+[rm['eligible_paths']],'Summary counts differ')
    task=TaskEnvelope(m['task_envelope']);taskfile=Path(m['task_envelope'])/'task.json';require(sha(taskfile)==rm['task_json_sha256'],'Task hash mismatch');phases=read(taskfile)['phases']
    urdf,=[Path(k) for k in m['files'] if Path(k).name=='med14_runtime.urdf'];require(sha(urdf)==rm['urdf_sha256'],'URDF hash mismatch')
    joints={x.attrib['name']:x.find('limit').attrib for x in ET.parse(urdf).getroot().findall('joint') if x.attrib['name'] in {f'lbr_A{i}' for i in range(1,8)}}
    lo,hi,vel=(np.array([float(joints[f'lbr_A{i}'][k]) for i in range(1,8)]) for k in ('lower','upper','velocity'))
    o.mkdir(parents=True);robot=RobotModel();checker=TibiaCollisionChecker(robot);meshes=[stl(obj.meshPath)*np.asarray(obj.meshScale,float) for obj in checker.geom.geometryObjects];cyl=cylinder()
    def geometry(q):
        pin.updateGeometryPlacements(robot._model,robot._data,checker.geom,checker.geom_data,robot._configuration(q))
        return [move(mesh,pose.homogeneous) for mesh,pose in zip(meshes,checker.geom_data.oMg)]
    def scene(ax,q,desired,actual,ee,caption,close=False,camera=(24,-55),alpha=.55,show_task=True):
        ax.clear()
        for obj,triangles in zip(checker.geom.geometryObjects,geometry(q)):ax.add_collection3d(Poly3DCollection(triangles,facecolor='darkorange' if obj.name=='lbr_link_7_0' else 'silver',edgecolor='none',alpha=.15 if close else alpha))
        poses=robot.frame_poses(q);skeleton=np.array([matrix(poses[n])[:3,3] for n in robot.chain_frames]);ax.plot(*skeleton.T,'o-',color='midnightblue',lw=2,ms=3)
        ax.add_collection3d(Poly3DCollection(move(cyl,actual),facecolor='steelblue',edgecolor='none',alpha=.32))
        for x in (0.,.155,.31):
            theta=np.linspace(0,2*np.pi,49);ring=np.stack([np.full_like(theta,x),.03*np.cos(theta),.03*np.sin(theta)],axis=1);ax.plot(*move(ring,desired).T,color='magenta',lw=1,ls='--')
        ax.plot(*move(np.array([[0,0,0],[.31,0,0]]),desired).T,color='magenta',ls='--')
        frame(ax,actual,'T actual',.045);frame(ax,desired,'T requested',.035,True);frame(ax,ee,'E actual',.04);frame(ax,desired@np.linalg.inv(A),'E required',.035,True)
        if not close:frame(ax,np.eye(4),'B',.10);frame(ax,BF,'F',.08)
        if show_task:
            ax.plot(*target_E[:,:3,3].T,color='darkorange',lw=1,label='Required EE path');ax.plot(*FK[:,:3,3].T,color='black',ls='--',lw=1,label='FK EE path');ax.plot(*BT[:,:3,3].T,color='purple',lw=1,label='Requested tibia path');ax.plot(*actual_BT[:,:3,3].T,color='steelblue',lw=1,ls='--',label='Actual tibia path')
        equal(ax,np.vstack([move(cyl,actual).reshape(-1,3),move(cyl,desired).reshape(-1,3),ee[:3,3][None]] if close else [np.zeros((1,3)),move(cyl,actual).reshape(-1,3),move(cyl,desired).reshape(-1,3),ee[:3,3][None]]))
        ax.view_init(*camera);ax.set_title(caption,fontsize=9)
    detail=[];loaded=[]
    for x in d[:a.top]:
        folder=Path(x['path']);zpath=folder/'whole_trajectory_path.npz';require(zpath.resolve().is_relative_to(r) and sha(zpath)==x['path_sha256'],'Selected trajectory identity mismatch')
        with np.load(zpath,allow_pickle=False) as z:t,q,A,BF=(np.asarray(z[k],float) for k in ('time_s','q','A','X_BF'))
        require(t.ndim==1 and q.shape==(len(t),7) and np.isfinite(q).all() and np.isfinite(t).all() and np.all(np.diff(t)>0),'Bad trajectory arrays')
        room=np.rad2deg(np.minimum(q-lo,hi-q));require(np.isclose(room.min(),float(x['minimum_hard_room_deg']),atol=1e-6,rtol=0),'Joint room differs')
        cases=table(folder/'capability_cases.csv');caps={(float(y['time_s']),int(y['case_index'])):float(y['lambda_max']) for y in cases};curves=[]
        for ph in phases:
            ids=ph['active_case_indices'];l,h=ph['start_time_s'],ph['end_time_s']
            if not ids:continue
            ts=sorted({tt for tt,i in caps if l<=tt<=h and i in ids});vs=[min(caps[tt,i] for i in ids) for tt in ts];require(ts[0]==l and ts[-1]==h,'Loaded phase endpoints missing');curves.append((ts,vs))
        integ=sum(np.dot(np.diff(ts),(np.array(vs[:-1])+vs[1:])/2) for ts,vs in curves);duration=sum(ts[-1]-ts[0] for ts,vs in curves)
        require(np.allclose([min(min(v) for t,v in curves),integ/duration],[float(x['min_lambda']),float(x['time_mean_active_lambda'])],rtol=1e-7),'Capability metrics differ')
        detail.append((x,t,q,room,curves))
    plt.rcParams.update({'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':120})
    def save(f,n):
        for ext in ('png','pdf'):f.savefig(o/f'{n}.{ext}',bbox_inches='tight',dpi=200)
        plt.close(f)
    def bars(ax,labels,values,title):
        b=ax.barh(labels,values,color='#39768c');ax.bar_label(b,padding=4);ax.invert_yaxis();ax.margins(x=.16);ax.set_title(title)
    f,ax=plt.subplots(1,2,figsize=(12,4),layout='constrained');bars(ax[0],['Generated','Screen pass','Audited >=1 path','Accepted >=1 path','Ranked'],[len(g),len(passing),len(audited),len({x['design_id'] for x in p}),len(d)],'Design counts');bars(ax[1],['Audited','Collision accepted','Eligible'],[len(c),len(p),len(eligible)],'Path counts');save(f,'01_stage_yield')
    subset=list(reversed(d[:20]));f,ax=plt.subplots(figsize=(10,8));left=np.zeros(len(subset))
    for k,label in [('component_min','Minimum capability'),('component_mean','Active-time mean'),('component_joint','Joint comfort')]:
        vals=np.array([float(x[k]) for x in subset]);ax.barh([x['design_id'] for x in subset],vals,left=left,label=label);left+=vals
    ax.set(xlabel='Weighted score',title='Top 20 designs: score decomposition');ax.legend();save(f,'02_scores')
    f,ax=plt.subplots(figsize=(8,5))
    for val,label in [('False','Buffer not met'),('True','Eligible')]:
        pts=[x for x in p if x['eligible']==val];ax.scatter([float(x['extra_room_deg']) for x in pts],[float(x['min_lambda']) for x in pts],label=label,alpha=.6)
    ax.axvline(rm['min_extra_deg'],ls='--',color='black');ax.set(xlabel='Extra joint room beyond reserve [deg]',ylabel='Minimum capability multiplier',title='All collision-accepted paths');ax.legend();save(f,'03_tradeoff')
    f,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
    for i,(x,t,q,room,curves) in enumerate(detail):
        label=f"#{x['rank']} {x['design_id']}";color=f'C{i%10}'
        for j,(ts,vs) in enumerate(curves):axes[0].plot(ts,vs,color=color,label=label if j==0 else None)
        axes[1].plot(t,room.min(axis=1),color=color);axes[2].stairs(np.max(np.abs(np.diff(q,axis=0))/np.diff(t)[:,None]/vel,axis=1),t,color=color,baseline=None)
    for ax in axes:
        for ph in phases:
            if not ph['active_case_indices']:ax.axvspan(ph['start_time_s'],ph['end_time_s'],color='.9',zorder=-1)
    reserve=m['evaluation']['reserve_deg'];axes[1].axhline(reserve,color='.4',ls=':');axes[1].axhline(reserve+rm['min_extra_deg'],color='.4',ls='--');axes[2].axhline(1,color='.4',ls='--');axes[0].legend(fontsize=8,ncol=2);axes[0].set_ylabel('Worst active multiplier');axes[1].set_ylabel('Hard-limit room [deg]');axes[2].set(ylabel='Max joint speed / limit',xlabel='Task time [s]');save(f,'06_selected_path_profiles')
    rejects=[]
    for x in c:
        y=x['result']
        if y['sampled_pass']:continue
        first=y.get('first_violation') or {};worst=y.get('worst_sample') or {}
        rejects.append(dict(design=x['design_id'],path=Path(x['path']).parent.name,minimum_m=y['minimum_distance_m'],worst_geometry=worst.get('minimum_distance_geometry','unreported'),first_violating=';'.join(first['violating_geometries']) if 'violating_geometries' in first else 'unreported',first_sample=json.dumps(first)))
    if rejects:
        with (o/'collision_rejections.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rejects[0]));w.writeheader();w.writerows(rejects)
        f,ax=plt.subplots(figsize=(10,8));rows=sorted(rejects,key=lambda x:x['minimum_m']);ax.barh([x['design']+'/'+x['path'] for x in rows],[1000*x['minimum_m'] for x in rows],color='#aa5548');ax.invert_yaxis();ax.set(xlabel='Minimum reported signed distance [mm]',title='Rejected paths: worst distance, not necessarily first policy violation');save(f,'07_collision_rejections')
    for i,(x,t,q,room,curves) in enumerate(detail):
        label=f"rank_{int(x['rank']):02d}_{x['design_id']}";folder=Path(x['path']);zpath=folder/'whole_trajectory_path.npz'
        with np.load(zpath,allow_pickle=False) as z:A=matrix(z['A']);BF=matrix(z['X_BF'])
        FK=np.array([matrix(robot.fk_ee(qq)) for qq in q]);BT=np.array([BF@matrix(task.pose_at(float(v))) for v in t]);actual_BT=FK@A;target_E=BT@np.linalg.inv(A);ep=np.linalg.norm(FK[:,:3,3]-target_E[:,:3,3],axis=1)*1000;er=np.rad2deg(Rotation.from_matrix(np.swapaxes(target_E[:,:3,:3],1,2)@FK[:,:3,:3]).magnitude())
        f=plt.figure(figsize=(16,4.5*4))
        for row,(tt,close) in enumerate([(t[0],False),(t[len(t)//2],False),(t[-1],False),(t[np.argmax(ep)],True)]):
            for col,camera in enumerate(((24,-55),(90,-90),(0,-90))):
                ax=f.add_subplot(4,3,3*row+col+1,projection='3d');qq=np.array([np.interp(tt,t,q[:,j]) for j in range(7)]);ee=matrix(robot.fk_ee(qq));actual=ee@A;desired=BF@matrix(task.pose_at(float(tt)));scene(ax,qq,desired,actual,ee,f"{label}\n{'Interface' if close else 'Full robot'} | t={tt:.2f}s | EE error {np.interp(tt,t,ep):.3f}mm/{np.interp(tt,t,er):.3f}deg",close,camera,show_task=not close)
        f.suptitle(label+'\nSilver: URDF collision meshes; orange: link 7; blue: attached tibia; magenta dashed: requested tibia\nRGB axes = x/y/z. Geometry review, not a safety certificate.',fontsize=12);f.tight_layout(rect=(0,0,1,.94));save(f,f'08_{label}_scenes')
        f,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
        for j in range(q.shape[1]):axes[0].plot(t,np.rad2deg(q[:,j]),label=f'J{j+1}')
        axes[0].set_ylabel('Joint angle [deg]');axes[0].legend(ncol=7);axes[1].plot(t,ep,label='EE');axes[1].plot(t,np.linalg.norm(actual_BT[:,:3,3]-BT[:,:3,3],axis=1)*1000,label='Tibia origin');axes[1].set_ylabel('Position error [mm]');axes[1].legend();axes[2].plot(t,er,label='EE');axes[2].plot(t,np.rad2deg(Rotation.from_matrix(np.swapaxes(BT[:,:3,:3],1,2)@actual_BT[:,:3,:3]).magnitude()),label='Tibia');axes[2].set_ylabel('Orientation error [deg]');axes[2].set_xlabel('Task time [s]');axes[2].legend()
        for ax in axes:ax.grid(alpha=.25)
        f.suptitle(label+' - saved-node FK versus independently requested task');f.tight_layout();save(f,f'09_{label}_errors')
        if not a.no_gif:
            times=np.linspace(t[0],t[-1],a.frames);qi=np.column_stack([np.interp(times,t,q[:,j]) for j in range(7)]);f=plt.figure(figsize=(13,10));views=[(False,(24,-55),'Whole robot - oblique'),(False,(15,45),'Whole robot - opposite side'),(True,(90,-90),'Interface - top'),(True,(0,-90),'Interface - side')];axs=[f.add_subplot(2,2,i+1,projection='3d') for i in range(len(views))]
            def update(k):
                ee=matrix(robot.fk_ee(qi[k]));actual=ee@A;desired=BF@matrix(task.pose_at(float(times[k])))
                for ax,(close,camera,view_label) in zip(axs,views):scene(ax,qi[k],desired,actual,ee,view_label,close,camera)
                f.suptitle(f"{label} | t={times[k]:.2f}s | {(t[-1]-t[0])/((a.frames-1)/a.fps):.1f}x playback\nBlue: attached tibia; magenta: requested; RGB: x/y/z; joint-linear interpolation",fontsize=10);return []
            FuncAnimation(f,update,frames=a.frames,interval=1000/a.fps,blit=False).save(o/f'10_{label}_motion.gif',writer=PillowWriter(fps=a.fps),dpi=85);plt.close(f)
    (o/'summary.json').write_text(json.dumps(dict(campaign=summary,eligible_paths=len(eligible),accepted_designs=len({x['design_id'] for x in p}),selected=d[:a.top],unreported_first_violations=sum(x['first_violating']=='unreported' for x in rejects)),indent=2))
    print(o)

if __name__=='__main__':main()
