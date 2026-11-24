# Convert ANT-X 1-DOF .ulg logs -> 250 Hz csv (t, theta, q, M, T, m_inj, openloop)
import sys, glob, os, numpy as np, pandas as pd
from pyulog import ULog
src, out = sys.argv[1], sys.argv[2]
KIND={22:'impulse',24:'impulse',25:'impulse',26:'impulse',27:'impulse',
      30:'doublet',31:'doublet',32:'doublet',33:'doublet',34:'doublet',35:'doublet'}
rows=[]; count={}
for f in sorted(glob.glob(src+'/*.ulg'), key=lambda s:int(os.path.basename(s).split('_')[1])):   # time order
    n=int(os.path.basename(f).split('_')[1])
    if n not in KIND: continue
    D={}
    for d in ULog(f).data_list:
        if d.multi_id==0: D[d.name]=d.data
    att=D['vehicle_attitude']; t0=float(att['timestamp'][0])
    q0,q1,q2,q3=[att[f'q[{i}]'] for i in range(4)]
    pitch=np.arcsin(np.clip(2*(q0*q2-q3*q1),-1,1))
    av=D['vehicle_angular_velocity']; ac=D['actuator_controls_0']; L=D['lmnf_injection']
    tg=np.arange(0,(float(att['timestamp'][-1])-t0)/1e6,0.004)
    I=lambda ts,v: np.interp(tg,(ts.astype(np.float64)-t0)/1e6,v)
    Z=lambda ts,v: v[np.clip(np.searchsorted((ts.astype(np.float64)-t0)/1e6,tg,side='right')-1,0,len(v)-1)]  # zero-order hold for commands
    df=pd.DataFrame({'t':tg,'theta':I(att['timestamp'],pitch),'q':I(av['timestamp'],av['xyz[1]']),
        'motor_front':Z(ac['timestamp'],ac['control[0]']),'motor_back':Z(ac['timestamp'],ac['control[1]']),
        'm_inj':Z(L['timestamp'],L['m']),'openloop':Z(L['timestamp'],L['type_openloop'].astype(float))})
    df['M']=df.motor_front-df.motor_back; df['T']=(df.motor_front+df.motor_back)/2
    ol=df.openloop.values>0.5; e=np.nonzero(ol)[0]
    count[KIND[n]]=count.get(KIND[n],0)+1
    name=f'{KIND[n]}_{count[KIND[n]]}.csv'; df.to_csv(out+'/'+name,index=False,float_format='%.6g')
    nz=np.nonzero(df.m_inj.values)[0]
    rows.append(dict(file=name,original_log=f'log_{n}',kind=KIND[n],duration_s=round(tg[-1],1),inj_start_s=round(tg[nz[0]],2),
        ol_start_s=round(tg[e[0]],2),ol_end_s=round(tg[e[-1]],2),m_min=df.m_inj.min(),m_max=df.m_inj.max(),
        T_mean=round(df['T'].mean(),3),theta_range_deg=f"{np.degrees(df.theta.min()):.1f}..{np.degrees(df.theta.max()):.1f}"))
pd.DataFrame(rows).to_csv(out+'/flights_1dof.csv',index=False); print(pd.DataFrame(rows).to_string())
