import json, sys
for f in sys.argv[1:]:
    d = json.load(open(f))
    print(f"\n##### {d['file'].split('Robots/')[-1]}  defaultPrim={d['defaultPrim']} up={d['upAxis']} mpu={d['metersPerUnit']}")
    print(f" size(m)={d.get('size')}  artRoots={d['articulationRoots']}  rigidBodies={d['rigidBodies']}  massTotal={d['massTotal']}")
    print(f" colliders={d['colliders']}")
    print(f" masses={[(m[0].split('/')[-1], m[1]) for m in d['masses']]}")
    for j in d['joints']:
        drv = j.get('drive'); ds = ''
        if drv: ds = f" drive[{drv['kind']}:{drv['type']} k={drv['stiffness']} c={drv['damping']} F={drv['maxForce']}]"
        lim = '' if j['lower'] is None and j['upper'] is None else f" lim=({j['lower']},{j['upper']})"
        mv = f" maxV={j['maxJointVelocity']}" if 'maxJointVelocity' in j else ''
        bs = f" body1_size={j['body1_size']}" if 'body1_size' in j else ''
        en = '' if j['enabled'] else ' [DISABLED]'
        print(f"  J {j['type'][7:]:<14} {j['path'].split('/')[-1]:<28} {j['body0']}->{j['body1']} ax={j['axis']}{lim}{ds}{mv}{bs}{en}")
    for s in d['sensors']:
        print(f"  S {s['type']:<22} {s['path']}  {s.get('config','')} {', '.join(f'{k}={v}' for k,v in s.items() if k not in ('type','path','config'))}")
    for g in d['omniGraphs']: print(f"  G {g}")
    for v in d['variants']: print(f"  V {v['prim']} {v['set']}={v['sel']} of {v['opts']}")
    print(f" primTypes={d['primTypes']}")
    print(f" refs={d['refs'][:12]}")
