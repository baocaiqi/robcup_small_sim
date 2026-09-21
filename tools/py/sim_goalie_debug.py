import sys, math
sys.path.insert(0, r'e:/robcup/robcup_small_sim/tools/py')
import rlg_analyzer as R

path = r'C:/Strategy/20260907165430-5-DEMO Yellow-MyTeam-Blue.rlg'
frames = R.parse_rlg(path)

def norm(a):
    while a > 180.0: a -= 360.0
    while a <= -180.0: a += 360.0
    return a
def adiff(a,b): return norm(a-b)
def ato(x1,y1,x2,y2): return norm(math.atan2(y2-y1, x2-x1)*180/math.pi)
def dist(x1,y1,x2,y2): return math.hypot(x2-x1,y2-y1)
def clamp(v,lo,hi): return max(lo,min(hi,v))

OGX=220.0; AD=-1.0; OPPGX=0.0
GL=70.0; GH=110.0
KGuard=10.0; KTrackLo=76.0; KTrackHi=104.0
KMaxTTA=15.0; KMaxReach=30.0; KMinSpeed=5.0
KDrib=6.0; KSupport=60.0; KFast=12.0; KPull=20.0
KClear=20.0; KPush=8.0; KLat=15.0

def clamp_goalie(x,y):
    lo=min(OGX, OGX+AD*80); hi=max(OGX, OGX+AD*80)
    return clamp(x,lo,hi), clamp(y,72.5,107.5)

def danger_spd(bx,by,vx,vy):
    dx=OGX-bx; dy=90-by; d=math.hypot(dx,dy)
    if d<1e-6: return 0.0
    return max(0.0,(vx*dx+vy*dy)/d)

def predict_y(x,y,vx,vy,tx):
    if abs(vx)<1e-9: return None
    t=(tx-x)/vx
    if t<0: return None
    return y+vy*t

def motion_pos(r, tx, ty):
    dx=tx-r['x']; dy=ty-r['y']; de=math.hypot(dx,dy)
    if de<1.0: return ('stop',0,0,0,0)
    da=ato(r['x'],r['y'],tx,ty); te=adiff(da,r['rot'])
    vc=150.0; Ka=10.0/90.0
    if de>100: Ka=20/90
    elif de>50: Ka=22/90
    elif de>30: Ka=24/90
    elif de>20: Ka=26/90
    else: Ka=28/90
    drive=vc*(1.0/(1.0+math.exp(-3.0*de))-0.25)
    if de<15: drive*=de/15
    if de<12 and abs(te)>30: drive*=0.25
    if te>95 or te<-95:
        te += -180 if te>0 else 180
        te=clamp(te,-80,80)
        if de<5 and abs(te)<40: Ka=0.1
        vr=-drive+Ka*te; vl=-drive-Ka*te
        return ('back',vl,vr,te,da)
    elif te>-85 and te<85:
        if de<5 and abs(te)<40: Ka=0.1
        vr=drive+Ka*te; vl=drive-Ka*te
        return ('fwd',vl,vr,te,da)
    else:
        vr=0.17*te; vl=-0.17*te
        return ('rot',vl,vr,te,da)

def run_goalie(fi):
    f=frames[fi]
    g=f['blue'][0]
    r={'x':g['x'],'y':g['y'],'rot':g['rot']}
    bx,by=f['ball']['x'],f['ball']['y']
    pb=frames[fi-1]['ball']
    vx=bx-pb['x']; vy=by-pb['y']
    if abs(vx)>30: vx=0
    if abs(vy)>30: vy=0
    danger=danger_spd(bx,by,vx,vy)
    db=dist(r['x'],r['y'],bx,by)
    dg=abs(bx-OGX)
    spd=math.hypot(vx,vy)
    ball_still=spd<2.0
    opp_dmin=1e9
    if not ball_still:
        for o in f['yellow']:
            opp_dmin=min(opp_dmin, dist(bx,by,o['x'],o['y']))
    if dg<45.0 and (ball_still or (opp_dmin<25.0 and danger<KDrib)):
        dbg=db; aligned=abs(r['y']-by)<=3.0
        if dbg<25.0 and aligned:
            px=bx+AD*30; py=clamp(by,78,102)
            tag='clear'
        else:
            px=bx-AD*8; py=clamp(by,78,102)
            tag='clear_else'
        px,py=clamp_goalie(px,py)
        st,vl,vr,te,da=motion_pos(r,px,py)
        return (tag,px,py,st,vl,vr,te,da,vx,vy,spd,danger)
    if dg<45.0 and opp_dmin<25.0:
        back=max(0.0,dg-12.0); depth2=min(40.0,max(KGuard,back))
        px2=OGX+AD*depth2; py2=90+(by-90)*(back/dg); py2=clamp(py2,78,102)
        px2,py2=clamp_goalie(px2,py2)
        st,vl,vr,te,da=motion_pos(r,px2,py2)
        return ('opp_press',px2,py2,st,vl,vr,te,da,vx,vy,spd,danger)
    gx=OGX+AD*(3.0 if abs(bx-OGX)<15 else KGuard)
    yg=predict_y(bx,by,vx,vy,OGX)
    heading=(yg is not None); on_target=heading and GL<=yg<=GH
    tta=1e9
    if abs(vx)>1e-9: tta=abs(OGX-bx)/abs(vx)
    dmin=1e9; dribbler=-1
    for i,o in enumerate(f['yellow']):
        d=dist(bx,by,o['x'],o['y'])
        if d<dmin: dmin=d; dribbler=i
    opp_has=dmin<15.0; dribble_press=opp_has and dg<50.0
    has_support=False
    if opp_has and danger>KDrib:
        dg_d=abs(f['yellow'][dribbler]['x']-OGX)
        for i,o in enumerate(f['yellow']):
            if i==dribbler: continue
            d=dist(f['yellow'][dribbler]['x'],f['yellow'][dribbler]['y'],o['x'],o['y'])
            if d<KSupport and abs(o['x']-OGX)<dg_d-5: has_support=True; break
    opp_in_box=0
    for o in f['yellow']:
        if 140<=o['x']<=220 and 72.5<=o['y']<=107.5: opp_in_box+=1
    frac=clamp((danger-KMinSpeed)/(KFast-KMinSpeed),0,1)
    depth=KGuard+frac*(80-KGuard)
    depth=max(KGuard, depth-opp_in_box*KPull)
    if dribble_press and danger<KMinSpeed:
        depth=min(40,max(KGuard,dg-12))
    out_x=OGX+AD*(3.0 if abs(bx-OGX)<15 else depth)
    iy=90.0
    if predict_y(bx,by,vx,vy,out_x) is None:
        if abs(bx-OGX)>1e-6:
            t=(out_x-OGX)/(bx-OGX); t=clamp(t,0,1); iy=90+t*(by-90)
    if on_target and abs(yg-90)>abs(iy-90): iy=yg
    out_x,iy=clamp_goalie(out_x,iy)
    near_line=dg<40.0; near_post=(by<GL+12) or (by>GH-12)
    heading_post=heading and (yg<GL+12 or yg>GH-12)
    if near_line and (near_post or heading_post):
        ref_y=yg if heading_post else by
        post_y=GL if ref_y<90 else GH
        block_y=clamp(ref_y, post_y-6, post_y+6); block_y=clamp(block_y,GL,GH)
        st,vl,vr,te,da=motion_pos(r,OGX+AD*6,block_y)
        return ('post_block',OGX+AD*6,block_y,st,vl,vr,te,da,vx,vy,spd,danger)
    if on_target and db<KMaxReach:
        aim_x=OGX+AD*6; aim_y=clamp(yg,74,106)
        if abs(yg-r['y'])>KMaxReach: aim_y=74 if yg<90 else 106
        st,vl,vr,te,da=motion_pos(r,aim_x,aim_y)
        return ('slow_block',aim_x,aim_y,st,vl,vr,te,da,vx,vy,spd,danger)
    clearing=False; clear_x=clear_y=0
    if db<KClear:
        best_id=-1; best_open=-1e9
        for i in range(5):
            if i==0: continue
            tx,ty=f['blue'][i]['x'],f['blue'][i]['y']
            if abs(tx-OGX)<abs(bx-OGX): continue
            mo=1e9
            for o in f['yellow']: mo=min(mo,dist(tx,ty,o['x'],o['y']))
            sc=mo+(30 if (ty<10 or ty>170) else 0)
            if sc>best_open: best_open=sc; best_id=i
        dirx=diry=0
        if on_target: dirx=AD; diry=0
        elif best_id>=0: dirx=f['blue'][best_id]['x']-bx; diry=f['blue'][best_id]['y']-by
        else: dirx=OPPGX-bx; diry=0
        ln=math.hypot(dirx,diry)
        if ln<1e-6: dirx=OPPGX-bx; diry=0; ln=math.hypot(dirx,diry)
        if ln<1e-6: dirx=0; diry=1; ln=1
        dirx/=ln; diry/=ln
        clear_x=bx-dirx*KPush; clear_y=by-diry*KPush
        if abs(bx-OGX)<abs(r['x']-OGX):
            nx=-diry; ny=dirx
            side=(r['x']-bx)*nx+(r['y']-by)*ny
            s=1 if side>=0 else -1
            clear_x+=s*nx*KLat; clear_y+=s*ny*KLat
        clear_x,clear_y=clamp_goalie(clear_x,clear_y); clearing=True
    if clearing:
        st,vl,vr,te,da=motion_pos(r,clear_x,clear_y)
        return ('clearing',clear_x,clear_y,st,vl,vr,te,da,vx,vy,spd,danger)
    if has_support:
        st,vl,vr,te,da=motion_pos(r,gx,clamp(yg,KTrackLo,KTrackHi))
        return ('support',gx,clamp(yg,KTrackLo,KTrackHi),st,vl,vr,te,da,vx,vy,spd,danger)
    if on_target and tta<KMaxTTA and db<KMaxReach and danger>KMinSpeed:
        aim_x=OGX+AD*3; aim_y=clamp(yg,74,106)
        if abs(yg-r['y'])>KMaxReach: aim_y=74 if yg<90 else 106
        st,vl,vr,te,da=motion_pos(r,aim_x,aim_y)
        return ('dive',aim_x,aim_y,st,vl,vr,te,da,vx,vy,spd,danger)
    if on_target or (opp_has and danger>KMinSpeed) or dribble_press:
        st,vl,vr,te,da=motion_pos(r,out_x,iy)
        return ('longshot',out_x,iy,st,vl,vr,te,da,vx,vy,spd,danger)
    st,vl,vr,te,da=motion_pos(r,gx,clamp(by,KTrackLo,KTrackHi))
    return ('track',gx,clamp(by,KTrackLo,KTrackHi),st,vl,vr,te,da,vx,vy,spd,danger)

print('fr    分支         目标x,目标y   动作   vl     vr      te     da    实际位移方向  rot')
for fi in range(1395, 1422):
    br,px,py,st,vl,vr,te,da,vx,vy,spd,dg=run_goalie(fi)
    g0=frames[fi]['blue'][0]; g1=frames[fi+1]['blue'][0]
    dx=g1['x']-g0['x']; dy=g1['y']-g0['y']
    actual=norm(math.atan2(dy,dx)*180/math.pi) if (dx or dy) else 0
    print('%5d %-12s (%6.1f,%6.1f) %-4s %6.1f %6.1f %6.1f %6.1f  实际%7.1f  rot=%6.1f' % (
        fi, br, px, py, st, vl, vr, te, da, actual, g0['rot']))
