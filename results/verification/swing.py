import numpy as np, pandas as pd
from scipy.signal import find_peaks
D='../../data/clean/1dof/'
rows=[]
for i in range(1,6):
    f=pd.read_csv(D+f'impulse_{i}.csv'); tend=f.t[f.M>0].iloc[-1]
    g=f[f.t>tend+0.02]; th0=g.theta.iloc[-200:].mean(); y=(g.theta-th0).values; t=g.t.values
    pk,_=find_peaks(y,prominence=0.01); tr,_=find_peaks(-y,prominence=0.01)
    ex=np.sort(np.r_[pk,tr]); A=np.abs(y[ex]); te=t[ex]
    Td=te[2]-te[0]                  # damped period from first half-cycles
    d=np.log(A[0]/A[2]) if len(A)>2 else np.nan   # log decrement over one full cycle
    xi=d/np.sqrt(4*np.pi**2+d**2); wd=2*np.pi/Td; wn=wd/np.sqrt(1-xi**2)
    rows.append(dict(flight=f'impulse_{i}',n_peaks=len(ex),Td=Td,wn=wn,xi=xi,amps=np.round(A[:6]*57.3,2).tolist(),times=np.round(te[:6],3).tolist()))
    print(rows[-1])
df=pd.DataFrame(rows); df.to_csv('swing_results.csv',index=False); print(df[['wn','xi']].describe())
