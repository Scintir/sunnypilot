"""Drivability audit driven by driver feedback after iter15 drives a8..b3:

  1) ROLLING WINDOW (set-vs-actual delta): the limiter is supposed to hold
     effective set speed <= vEgo + dynamic_margin. Measure (cruiseState.speed - vEgo)
     while ENGAGED, by speed bucket, and enumerate large-delta episodes (>15 mph)
     with the governor context we can infer (decel/lead vs normal).

  2) POWER-OVER-CAP WITHOUT BACKOFF: episodes where estPowerControlW stays >= a
     threshold (45 kW) while engaged & moving, with whether the limiter emitted any
     SET decrement during the episode. 'No backoff' = sustained over-cap, 0 SET.

  3) MANUAL INTERVENTION signature: driver decelCruise press(es) followed by
     accelCruise/resumeCruise within 20 s (driver knocks speed down then back up).

Filters to ENGAGED frames only (cruiseState.enabled). Run with bare route id.
"""
import sys, json
from pathlib import Path
from collections import defaultdict
sys.path.insert(0, "/home/alex.smith/git/sunnypilot/tools/log_uploader/forensics")
import _offdevice_shim  # noqa
from openpilot.tools.lib.logreader import LogReader
MPH = 2.2369362920544025
STATE_NAMES = {0:"IDLE",1:"STANDSTILL_HOLD",2:"PRELAUNCH_SET",3:"SOFT_CAP",4:"RECOVERY",
               5:"OVERRIDE_SET",6:"OVERRIDE_RES",7:"BUS_FAULT",8:"DISABLED"}
OVERCAP_KW = 45.0          # user said cap was 40-45; use 45 as the conservative over-cap line
DELTA_EPISODE_MPH = 15.0   # set-vs-actual delta that counts as a windup episode
MIN_EPISODE_FRAMES = 30    # 0.3 s

def pct(v, p):
  if not v: return None
  s = sorted(v); return s[min(int(p*len(s)), len(s)-1)]

def analyze(route):
  rd = Path("can_data")/route
  segs = sorted([p for p in rd.parent.glob(f"{rd.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))

  # state held from latest carState
  vego=0.0; setspd=0.0; enabled=False; lead=False; alead=0.0

  delta_by_bucket=defaultdict(list)     # vEgo bucket -> [delta_mph] (engaged, moving)
  delta_all=[]
  n_delta_gt10=n_delta_gt20=n_delta_gt30=0; n_engaged_moving=0
  delta_episodes=[]; cur_d=None

  overcap_episodes=[]; cur_o=None
  prev_set_emitted=None

  btn_press=[]  # (t, type) for decel/accel/res

  for seg in segs:
    rlog=seg/"rlog.zst"
    if not rlog.exists(): continue
    try: lr=LogReader(str(rlog))
    except Exception: continue
    for msg in lr:
      typ=msg.which(); t=msg.logMonoTime/1e9
      if typ=="carState":
        cs=msg.carState; vego=cs.vEgo; setspd=cs.cruiseState.speed; enabled=bool(cs.cruiseState.enabled)
        try:
          ld=cs.leadOne; lead=bool(ld.status); alead=float(getattr(ld,'aLeadK',0.0))
        except Exception: pass
        for be in cs.buttonEvents:
          if be.pressed and str(be.type) in ("decelCruise","accelCruise","resumeCruise"):
            btn_press.append((round(t,2), str(be.type)))
      elif typ=="carStateSP":
        sp=msg.carStateSP
        st=int(sp.evLimiterState); ctl_kw=float(sp.estPowerControlW)/1000.0
        capped=bool(sp.estPowerCapped); set_emitted=int(sp.evLimiterSetEmitted)
        vmph=vego*MPH; setmph=setspd*MPH; delta=setmph-vmph

        engaged_moving = enabled and vmph>3.0
        if engaged_moving:
          n_engaged_moving+=1
          delta_all.append(delta)
          delta_by_bucket[int(vmph//10)*10].append(delta)
          if delta>10: n_delta_gt10+=1
          if delta>20: n_delta_gt20+=1
          if delta>30: n_delta_gt30+=1
          # delta episode (windup)
          if delta>DELTA_EPISODE_MPH:
            if cur_d is None: cur_d={'start_t':t,'frames':0,'peak_delta':0,'vego_min':1e9,'vego_at_peak':0,'lead_frac':0}
            cur_d['frames']+=1
            if delta>cur_d['peak_delta']: cur_d['peak_delta']=delta; cur_d['vego_at_peak']=vmph
            cur_d['vego_min']=min(cur_d['vego_min'],vmph)
            cur_d['lead_frac']+=1 if lead else 0
          else:
            if cur_d and cur_d['frames']>=MIN_EPISODE_FRAMES:
              cur_d['dur_s']=t-cur_d['start_t']; cur_d['lead_frac']=round(cur_d['lead_frac']/cur_d['frames'],2)
              cur_d['peak_delta']=round(cur_d['peak_delta'],1); cur_d['vego_min']=round(cur_d['vego_min'],1); cur_d['vego_at_peak']=round(cur_d['vego_at_peak'],1)
              delta_episodes.append({k:cur_d[k] for k in ('start_t','dur_s','peak_delta','vego_min','vego_at_peak','lead_frac','frames')})
            cur_d=None

        # over-cap episode (engaged & moving)
        if engaged_moving and ctl_kw>=OVERCAP_KW:
          if cur_o is None: cur_o={'start_t':t,'frames':0,'peak_kw':0,'vego_at_peak':0,'set0':set_emitted,'state_set':set(),'capped_frac':0}
          cur_o['frames']+=1
          if ctl_kw>cur_o['peak_kw']: cur_o['peak_kw']=ctl_kw; cur_o['vego_at_peak']=vmph
          cur_o['state_set'].add(STATE_NAMES.get(st,st)); cur_o['capped_frac']+=1 if capped else 0
          cur_o['set1']=set_emitted
        else:
          if cur_o and cur_o['frames']>=MIN_EPISODE_FRAMES:
            cur_o['dur_s']=round(t-cur_o['start_t'],2); cur_o['peak_kw']=round(cur_o['peak_kw'],1)
            cur_o['vego_at_peak']=round(cur_o['vego_at_peak'],1)
            cur_o['set_emitted_during']=cur_o.get('set1',cur_o['set0'])-cur_o['set0']
            cur_o['capped_frac']=round(cur_o['capped_frac']/cur_o['frames'],2)
            cur_o['states']=sorted(cur_o['state_set'])
            overcap_episodes.append({k:cur_o[k] for k in ('start_t','dur_s','peak_kw','vego_at_peak','set_emitted_during','capped_frac','states','frames')})
          cur_o=None

  # manual interventions: decelCruise then accel/resume within 20s
  interventions=[]
  for i,(t,ty) in enumerate(btn_press):
    if ty=="decelCruise":
      for (t2,ty2) in btn_press[i+1:]:
        if t2-t>20: break
        if ty2 in ("accelCruise","resumeCruise"):
          interventions.append({'decel_t':t,'recover_t':t2,'gap_s':round(t2-t,1)}); break

  bucket_stats={}
  for b,vals in sorted(delta_by_bucket.items()):
    if len(vals)<100: continue
    bucket_stats[f"{b}-{b+10}mph"]={'n':len(vals),'delta_p50':round(pct(vals,.5),1),
      'delta_p90':round(pct(vals,.9),1),'delta_p99':round(pct(vals,.99),1),'delta_max':round(max(vals),1)}

  overcap_no_backoff=[e for e in overcap_episodes if e['set_emitted_during']==0 and e['dur_s']>=1.0]
  return {
    'route':route,'engaged_moving_frames':n_engaged_moving,
    # 1) rolling window
    'delta_set_minus_vego': {
      'p50':round(pct(delta_all,.5) or 0,1),'p90':round(pct(delta_all,.9) or 0,1),
      'p99':round(pct(delta_all,.99) or 0,1),'max':round(max(delta_all,default=0),1),
      'pct_frames_gt10mph':round(n_delta_gt10/max(n_engaged_moving,1)*100,1),
      'pct_frames_gt20mph':round(n_delta_gt20/max(n_engaged_moving,1)*100,1),
      'pct_frames_gt30mph':round(n_delta_gt30/max(n_engaged_moving,1)*100,1)},
    'delta_by_speed_bucket':bucket_stats,
    'windup_episodes_count':len(delta_episodes),
    'windup_episodes_top':sorted(delta_episodes,key=lambda e:-e['peak_delta'])[:8],
    # 2) power over cap
    'overcap45_episodes_count':len(overcap_episodes),
    'overcap45_NO_BACKOFF_episodes':len(overcap_no_backoff),
    'overcap45_no_backoff_top':sorted(overcap_no_backoff,key=lambda e:-e['peak_kw'])[:8],
    'overcap45_peak_kw':round(max((e['peak_kw'] for e in overcap_episodes),default=0),1),
    # 3) manual interventions
    'manual_intervention_count':len(interventions),
    'manual_interventions':interventions[:12],
  }

if __name__=="__main__":
  print(json.dumps([analyze(r) for r in sys.argv[1:]],indent=2,default=str))
