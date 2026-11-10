"""Requirement-normalized scalar wrench margins with two independent solvers.

For a = J.T @ W_required, require lower <= bias + lambda*a <= upper,
lambda >= 0. No robot, ROS, geometry, force-control or dynamics dependency.
"""
import numpy as np
from scipy.optimize import linprog


def _inputs(a,lower,upper,bias):
    a=np.asarray(a,dtype=float);lo=np.asarray(lower,dtype=float);hi=np.asarray(upper,dtype=float)
    b=np.zeros_like(a) if bias is None else np.asarray(bias,dtype=float)
    if a.ndim!=1 or a.size==0 or lo.shape!=a.shape or hi.shape!=a.shape or b.shape!=a.shape:
        raise ValueError("all torque vectors must have one common nonempty vector shape")
    if not np.all(np.isfinite(np.r_[a,lo,hi,b])) or np.any(lo>hi):
        raise ValueError("finite coefficients/bounds/bias and lower<=upper are required")
    return a,lo,hi,b


def _record(status,lmin,lmax,a,lo,hi,b):
    return {"status":status,"lambda_min":lmin,"lambda_max":lmax,
            "bias_feasible":bool(np.all(b>=lo) and np.all(b<=hi)),
            "requirement_feasible":bool(np.all(b+a>=lo-1e-10) and np.all(b+a<=hi+1e-10)),
            "zero_wrench_torque_coefficient":bool(np.all(a==0.))}


def interval_margin(a,lower,upper,bias=None):
    a,lo,hi,b=_inputs(a,lower,upper,bias)
    left=0.;right=np.inf
    for ai,li,ui,bi in zip(a,lo,hi,b):
        if ai==0.:
            if bi<li or bi>ui:return _record("infeasible",None,None,a,lo,hi,b)
            continue
        x1=(li-bi)/ai;x2=(ui-bi)/ai
        left=max(left,min(x1,x2));right=min(right,max(x1,x2))
    if left>right:return _record("infeasible",None,None,a,lo,hi,b)
    return _record("unbounded" if np.isinf(right) else "finite",float(left),
                   None if np.isinf(right) else float(right),a,lo,hi,b)


def lp_margin(a,lower,upper,bias=None):
    a,lo,hi,b=_inputs(a,lower,upper,bias)
    # Let x=lambda*scale. This avoids dropping an entirely tiny nonzero ray.
    scale=float(np.max(np.abs(a)))
    if scale==0.:scale=1.
    ray=a/scale
    matrix=np.r_[ray,-ray][:,None];rhs=np.r_[hi-b,b-lo]
    minimum=linprog([1.],A_ub=matrix,b_ub=rhs,bounds=[(0.,None)],method="highs")
    if minimum.status==2:return _record("infeasible",None,None,a,lo,hi,b)
    if not minimum.success:raise RuntimeError(f"LP lower-end solve failed: {minimum.message}")
    maximum=linprog([-1.],A_ub=matrix,b_ub=rhs,bounds=[(0.,None)],method="highs")
    lmin=float(minimum.x[0]/scale)
    if maximum.status==3:return _record("unbounded",lmin,None,a,lo,hi,b)
    if not maximum.success:raise RuntimeError(f"LP maximum solve failed: {maximum.message}")
    lmax=float(maximum.x[0]/scale)
    if not np.all(np.isfinite([lmin,lmax])):raise FloatingPointError("margin exceeds finite floating-point range")
    total=b+lmax*a
    if np.max(np.r_[lo-total,total-hi])>1e-7:
        raise RuntimeError("LP solution violates torque bounds beyond numerical check tolerance")
    return _record("finite",lmin,lmax,a,lo,hi,b)


def checked_margin(a,lower,upper,bias=None):
    a,lo,hi,b=_inputs(a,lower,upper,bias)
    reference=interval_margin(a,lo,hi,b);lp=lp_margin(a,lo,hi,b)
    if reference["status"]!=lp["status"]:raise AssertionError(f"LP/interval status disagreement: {reference} vs {lp}")
    for key in ("lambda_min","lambda_max"):
        if reference[key] is None or lp[key] is None:
            if reference[key]!=lp[key]:raise AssertionError("LP/interval endpoint mismatch")
        elif not np.isclose(reference[key],lp[key],rtol=1e-7,atol=1e-9):
            raise AssertionError(f"LP/interval {key} mismatch")
    result=dict(reference);result["LP_interval_agreement"]=True
    result["limiting_joints"]=[]
    if result["status"]=="finite":
        total=b+result["lambda_max"]*a
        for index in range(len(a)):
            if a[index]==0.:continue
            tol=1e-7*(1+max(abs(lo[index]),abs(hi[index])))
            if abs(total[index]-lo[index])<=tol:
                result["limiting_joints"].append({"index":index,"bound":"lower"})
            if abs(total[index]-hi[index])<=tol:
                result["limiting_joints"].append({"index":index,"bound":"upper"})
    result["interpretation_note"]=("bias feasible: usual load-factor interpretation applies" if result["bias_feasible"]
        else "bias infeasible: use lambda_min and direct requirement_feasible; lambda_max>=1 alone is insufficient")
    return result
