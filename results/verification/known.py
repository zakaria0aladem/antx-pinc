import numpy as np, pandas as pd
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
D='../../data/1dof/'; DT=0.02
FL=pd.read_csv(D+'flights_1dof.csv').set_index('file')
def hover(n):
    r=FL.loc[n+'.csv']; df=pd.read_csv(D+n+'.csv')
    after=df[df.t>r.inj_start_s+0.7]; end=after.t[after.M.abs()>0.12].iloc[0]-0.05
    f=df[(df.t>=r.inj_start_s-0.1)&(df.t<end)]
    t=np.arange(f.t.iloc[0],f.t.iloc[-1],DT)
    return pd.DataFrame(dict(t=t-t[0],y=np.interp(t,f.t,f.theta),yd=np.interp(t,f.t,f.q),r=np.interp(t,f.t,f.M)))
F={n:hover(f'doublet_{n}') for n in range(1,7)}
keys=['wn','xi','mu','th0']; lo=[0.5,0,0,-0.5]; hi=[20,2,2000,0.5]
def run(p,t,R,y0):
    rhs=lambda tt,z:[z[1],-2*p[1]*p[0]*z[1]-p[0]**2*(z[0]-p[3])+p[2]*np.interp(tt-0.02,t,R,left=R[0])]
    return solve_ivp(rhs,(t[0],t[-1]),y0,t_eval=t,max_step=0.005).y[0]
def fit(fl,ys,x0):
    res=lambda v:np.concatenate([run(v,F[n].t.values,F[n].r.values,[ys[n][0],0.0])-ys[n] for n in fl])
    return least_squares(res,x0,bounds=(lo,hi)).x
# step 1: real fit (the "true" constants for the test) + real noise level
real={n:F[n].y.values for n in F}
for n in F: F[n].loc[:, 'yd']=0
P=fit([1,2,3,4],real,[3.8,0.15,180,-0.15]); print('real fit',P)
resid=np.concatenate([run(P,F[n].t.values,F[n].r.values,[real[n][0],0])-real[n] for n in [1,2,3,4]])
sig=resid.std(); print('resid std rad',sig)
clean={n:run(P,F[n].t.values,F[n].r.values,[real[n][0],0]) for n in F}
rng=np.random.default_rng(0); rows=[]
for noise in [0.0, 0.5, 1.0, 2.0]:     # x real residual level
  for s in range(10):
    ys={n:clean[n]+rng.normal(0,noise*sig,len(clean[n])) for n in F}
    x0=[rng.uniform(2,6),rng.uniform(0.02,0.5),rng.uniform(50,400),rng.uniform(-0.3,0.3)]
    q=fit([1,2,3,4],ys,x0)
    rows.append(dict(noise=noise,seed=s,**{k+'_0':a for k,a in zip(keys,x0)},**dict(zip(keys,q))))
    print(noise,s,np.round(q,4))
df=pd.DataFrame(rows); df.to_csv('known_answer_results.csv',index=False)
pd.Series(dict(zip(keys,P)),name='true').to_csv('known_answer_true.csv'); print('sig',sig)
# test 2: each real flight fitted alone
rows=[]
for n in F:
    q=fit([n],real,[3.8,0.15,180,-0.15]); rows.append(dict(flight=f'doublet_{n}',**dict(zip(keys,q)))); print('flight',n,np.round(q,4))
pd.DataFrame(rows).to_csv('per_flight_results.csv',index=False)
