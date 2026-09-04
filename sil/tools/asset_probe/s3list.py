import sys, re, urllib.request, urllib.parse
B = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
ROOT = "Assets/Isaac/6.0/Isaac/Robots/"
def list_prefix(prefix, delimiter=None):
    keys, prefixes, token = [], [], None
    while True:
        q = {"list-type": "2", "prefix": prefix}
        if delimiter: q["delimiter"] = delimiter
        if token: q["continuation-token"] = token
        x = urllib.request.urlopen(f"{B}/?{urllib.parse.urlencode(q)}", timeout=60).read().decode()
        keys += re.findall(r"<Key>(.*?)</Key>.*?<Size>(\d+)</Size>", x)
        prefixes += re.findall(r"<Prefix>(.*?)</Prefix>", x)
        m = re.search(r"<NextContinuationToken>(.*?)</NextContinuationToken>", x)
        if not m: break
        token = m.group(1)
    return keys, prefixes
if __name__ == "__main__":
    mode = sys.argv[1]
    for p in sys.argv[2:]:
        if mode == "dirs":
            _, pre = list_prefix(ROOT + p, "/")
            print(p, "->", [q.replace(ROOT + p, "") for q in pre if q != ROOT + p])
        else:
            keys, _ = list_prefix(ROOT + p)
            tot = sum(int(s) for _, s in keys)
            print(f"== {p}: {len(keys)} files, {tot/1e6:.1f} MB")
            for k, s in keys:
                r = k.replace(ROOT + p, "")
                if "/" not in r.strip("/") and not r.startswith(".thumbs"):
                    print(f"   {int(s)/1e6:7.2f} MB  {r}")
