# Convert ANT-X 2-DOF .ulg logs -> 250 Hz csv
# columns: t, theta, q, x, v, dist (raw laser), M (front-back motor), T (mean motor), m_inj,
#          theta_sp, x_sp, v_sp (controller setpoints)
import sys, glob, os, numpy as np, pandas as pd
from pyulog import ULog
src, out = sys.argv[1], sys.argv[2]
rows=[]; count={}
for f in sorted(glob.glob(src+'/*/*.ulg'), key=lambda s:os.path.basename(s).split('_',2)[2]):   # time order (date in file name)
    kind=os.path.basename(os.path.dirname(f)).lower().replace('-ident','').replace('-','_'); n=int(os.path.basename(f).split('_')[1])
    D={}
    for d in ULog(f).data_list:
        if d.multi_id==0: D[d.name]=d.data
    att=D['vehicle_attitude']; t0=float(att['timestamp'][0])
    tg=np.arange(0,(float(att['timestamp'][-1])-t0)/1e6,0.004)
    T_=lambda ts:(ts.astype(np.float64)-t0)/1e6
    I=lambda ts,v: np.interp(tg,T_(ts),v)
    Z=lambda ts,v: v[np.clip(np.searchsorted(T_(ts),tg,side='right')-1,0,len(v)-1)]
    pitch=lambda a,b,c,d: np.arcsin(np.clip(2*(a*c-d*b),-1,1))
    av=D['vehicle_angular_velocity']; ac=D['actuator_controls_0']; lp=D['vehicle_local_position']
    ds=D['distance_sensor']; fs=D['flight_controller_setpoint']
    df=pd.DataFrame({'t':tg,
        'theta':I(att['timestamp'],pitch(*[att[f'q[{i}]'] for i in range(4)])),
        'q':I(av['timestamp'],av['xyz[1]']),
        'x':I(lp['timestamp'],lp['x']),'v':I(lp['timestamp'],lp['vx']),
        'dist':Z(ds['timestamp'],ds['current_distance'])})
    front=Z(ac['timestamp'],ac['control[0]']); back=Z(ac['timestamp'],ac['control[1]'])
    df['M']=front-back; df['T']=(front+back)/2
    L=D.get('lmnf_injection'); df['m_inj']=Z(L['timestamp'],L['m']) if L is not None else 0.0
    df['theta_sp']=Z(fs['timestamp'],pitch(*[fs[f'q_r[{i}]'] for i in range(4)]))
    df['x_sp']=Z(fs['timestamp'],fs['p_r[0]']); df['v_sp']=Z(fs['timestamp'],fs['v_r[0]'])
    count[kind]=count.get(kind,0)+1
    name=f'{kind}_{count[kind]}.csv'; df.to_csv(out+'/'+name,index=False,float_format='%.6g')
    fly=df['T']>0.2
    rows.append(dict(file=name,original_log=f'log_{n}',kind=kind,duration_s=round(tg[-1],1),flying_s=round(fly.sum()*0.004,1),
        x_range_m=round(df.x.max()-df.x.min(),3),theta_range_deg=round(np.degrees(df.theta.max()-df.theta.min()),1),
        m_inj_max=round(df.m_inj.abs().max(),3),T_hover=round(df['T'][fly].mean(),3) if fly.any() else 0,
        usable='yes' if df.x.max()-df.x.min()>0.05 else 'NO: x does not move'))
pd.DataFrame(rows).to_csv(out+'/flights_2dof.csv',index=False); print(pd.DataFrame(rows).to_string())
