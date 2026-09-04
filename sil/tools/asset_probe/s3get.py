import sys, os, urllib.request, concurrent.futures as cf
from s3list import list_prefix, B, ROOT
def fetch(key, dst):
    if os.path.exists(dst) and os.path.getsize(dst) > 0: return 0
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    for attempt in range(3):
        try:
            urllib.request.urlretrieve(f"{B}/{key}", dst); return os.path.getsize(dst)
        except Exception as e:
            err = e
    print("FAIL", key, err, flush=True); return -1
out = sys.argv[1]; only_top = "--top" in sys.argv
for p in [a for a in sys.argv[2:] if not a.startswith("--")]:
    keys, _ = list_prefix(ROOT + p)
    jobs = []
    for k, s in keys:
        rel = k.replace(ROOT, "")
        if "/.thumbs/" in rel: continue
        if only_top and "/" in rel.replace(p, ""): continue
        jobs.append((k, os.path.join(out, rel)))
    with cf.ThreadPoolExecutor(16) as ex:
        got = list(ex.map(lambda j: fetch(*j), jobs))
    print(f"{p}: {len(jobs)} files, {sum(g for g in got if g>0)/1e6:.1f} MB downloaded, {sum(1 for g in got if g<0)} failed", flush=True)
