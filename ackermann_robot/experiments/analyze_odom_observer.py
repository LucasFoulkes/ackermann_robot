#!/usr/bin/env python3
"""Offline raw/EMA/alpha-beta comparison for esc_closed_loop.csv."""
import argparse, csv, math, statistics


def load(path):
    rows=list(csv.DictReader(open(path)))
    out={}
    for r in rows:
        if int(r['fresh_odom']) != 1: continue
        s=int(r['segment']); out.setdefault(s,[]).append({
            't':float(r['tf_stamp']), 'x':float(r['x_m']), 'y':float(r['y_m']),
            'yaw':float(r['yaw_rad']), 'v':float(r['measured_mps']),
            'target':float(r['target_mps'])})
    for q in out.values():
        x0,y0,h=q[0]['x'],q[0]['y'],q[0]['yaw']
        for r in q:r['z']=(r['x']-x0)*math.cos(h)+(r['y']-y0)*math.sin(h)
    return out


def ab_filter(q,alpha,beta):
    x,v=q[0]['z'],q[0]['v']; result=[v]; innovations=[]
    for old,r in zip(q,q[1:]):
        dt=r['t']-old['t']
        if dt<=0: result.append(v); continue
        xp=x+v*dt; residual=r['z']-xp; innovations.append(residual)
        x=xp+alpha*residual; v=v+beta*residual/dt; result.append(v)
    return result,innovations


def ema(q,gain):
    v=q[0]['v']; out=[v]
    for r in q[1:]:v += gain*(r['v']-v);out.append(v)
    return out


def raw_prediction(q):
    e=[]
    for a,b in zip(q,q[1:]):
        dt=b['t']-a['t'];e.append(b['z']-(a['z']+a['v']*dt))
    return e


def rms(x):return math.sqrt(statistics.fmean(v*v for v in x)) if x else math.nan


def score(data,alpha,beta,segments):
    innovations=[]
    for s in segments:
        _,e=ab_filter(data[s],alpha,beta);innovations+=e
    return rms(innovations)


def metrics(data,segments,kind,params):
    prediction=[]; tracking=[]; rough=[]; bias=[]
    for s in segments:
        q=data[s]
        if kind=='raw': v=[r['v'] for r in q]; prediction+=raw_prediction(q)
        elif kind=='ema':
            v=ema(q,params[0]);prediction += [b['z']-(a['z']+va*(b['t']-a['t']))
                for a,b,va in zip(q,q[1:],v)]
        else:v,prediction_part=ab_filter(q,*params);prediction+=prediction_part
        target=q[0]['target']; tracking += [target-x for x in v]
        bias += [target-statistics.fmean(v)]
        rough += [b-a for a,b in zip(v,v[1:])]
    return rms(prediction),rms(tracking),rms(rough),statistics.fmean(bias)


def main():
    p=argparse.ArgumentParser();p.add_argument('csv',nargs='?',default='esc_closed_loop.csv')
    args=p.parse_args();data=load(args.csv)
    train=[s for s in sorted(data) if s<=4];test=[s for s in sorted(data) if s>=5]
    best=min(((score(data,a,b,train),a,b)
              for a in [i/20 for i in range(2,20)]
              for b in [i/100 for i in range(1,61)]),key=lambda x:x[0])
    ema_best=min(((metrics(data,train,'ema',(g,))[0],g)
                  for g in [i/20 for i in range(1,21)]),key=lambda x:x[0])
    print('samples:',{s:len(data[s]) for s in sorted(data)})
    print(f'tuned on {train}: alpha={best[1]:.2f}, beta={best[2]:.2f}; EMA gain={ema_best[1]:.2f}')
    print('validation segments:',test)
    print('method     pos-predict-RMSE(m) tracking-RMSE(m/s) velocity-step-RMS(m/s) bias(m/s)')
    for name,param in [('raw',()),('ema',(ema_best[1],)),('alpha-beta',(best[1],best[2]))]:
        m=metrics(data,test,name,param)
        print(f'{name:10s} {m[0]:19.6f} {m[1]:19.6f} {m[2]:23.6f} {m[3]:10.6f}')


if __name__=='__main__':main()
