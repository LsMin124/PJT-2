# -*- coding: utf-8 -*-
"""Generate docs/architecture.html — summary diagram in the reference (images/architecture.png) style.

  python3 docs/tools/gen_architecture.py            # → docs/architecture.html (icons by relative path)
  python3 docs/tools/gen_architecture.py --embed    # → docs/architecture_embed.html (icons inlined)
"""
import base64, io, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.normpath(os.path.join(HERE, ".."))
IMGDIR = os.path.join(DOCS, "images")
EMBED = "--embed" in sys.argv
OUT = os.path.join(DOCS, "architecture_embed.html" if EMBED else "architecture.html")

W, H = 2400, 1000
INK, GREY, LINE, CARD, DEV = "#333333", "#6b6b6b", "#9a9a9a", "#d9d9d9", "#3a3a3a"
GREEN, BLUE, ORANGE = "#5f9e3a", "#3f5fd1", "#e07b1a"
FONT = 'Pretendard, "Apple SD Gothic Neo", Inter, "Malgun Gothic", system-ui, sans-serif'

E = []
warn = []


def add(s):
    E.append(s)


def tw(s, size):
    """Rough text width: CJK ≈ 1.0 em, Latin/digits ≈ 0.58 em, spaces/dots ≈ 0.35 em."""
    w = 0.0
    for ch in s:
        o = ord(ch)
        if ch in " ·/:.,-": w += 0.35
        elif o > 0x2E80: w += 1.0
        elif ch.isupper(): w += 0.68
        else: w += 0.58
    return w * size


def t(x, y, s, size=22, weight=400, fill=INK, anchor="start", extra="", limit=None):
    if limit is not None and tw(s, size) > limit:
        warn.append(f"overflow: '{s}' {tw(s, size):.0f}px > {limit}px")
    add(f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" fill="{fill}" text-anchor="{anchor}" {extra}>{s}</text>')


def img(name, x, y, w, h):
    href = f"images/{name}.png"
    if EMBED:
        href = "data:image/png;base64," + base64.b64encode(open(os.path.join(IMGDIR, name + ".png"), "rb").read()).decode()
    add(f'<image href="{href}" x="{x}" y="{y}" width="{w}" height="{h}" preserveAspectRatio="xMidYMid meet"/>')


def device(x, y, w, h, icon, title, sub, fill, stroke, iw=56, ih=56, stack=0, badge=None):
    for k in range(stack, 0, -1):   # ghost boxes behind, offset to the top-right
        add(f'<rect x="{x + 14 * k}" y="{y - 14 * k}" width="{w}" height="{h}" rx="26" fill="{fill}" stroke="{stroke}" stroke-width="1.6" opacity="{0.55 - 0.15 * (k - 1)}"/>')
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="26" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
    if badge:
        bw = tw(badge, 20) + 28
        add(f'<rect x="{x + w - bw - 24}" y="{y + 22}" width="{bw}" height="36" rx="18" fill="#fff" stroke="{stroke}" stroke-width="1.6"/>')
        t(x + w - 24 - bw / 2, y + 47, badge, 20, 700, "#3a6b2a", "middle")
    img(icon, x + 26, y + 24, iw, ih)
    tx = x + 26 + iw + 14
    t(tx, y + 24 + 40, title, 32, 700, limit=w - (tx - x) - 20)
    t(tx, y + 24 + 40 + 30, sub, 19, 400, GREY, limit=w - (tx - x) - 20)


def group(x, y, w, h, title, fill):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="20" fill="{fill}" stroke="{CARD}" stroke-width="1.6"/>')
    t(x + 24, y + 32, title, 21, 700, "#444", limit=w - 90)
    img("docker", x + w - 58, y + 12, 40, 22)


def card(x, y, w, h, icon, label, sub=None, iw=46, ih=46):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="18" fill="#fff" stroke="{CARD}" stroke-width="1.8"/>')
    img(icon, x + 18, y + (h - ih) / 2, iw, ih)
    lx = x + 18 + iw + 16
    limit = w - (lx - x) - 14
    if sub:
        t(lx, y + h / 2 - 2, label, 26, 700, limit=limit)
        t(lx, y + h / 2 + 24, sub, 17, 400, GREY, limit=limit)
    else:
        t(lx, y + h / 2 + 9, label, 26, 700, limit=limit)


def line(pts, color=LINE, dash=None, width=2):
    d = " ".join(f"{'M' if i == 0 else 'L'}{x} {y}" for i, (x, y) in enumerate(pts))
    da = f' stroke-dasharray="{dash}"' if dash else ""
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{da} stroke-linecap="round" stroke-linejoin="round"/>')


def pill(cx, cy, label, color, fill, w=110):
    add(f'<rect x="{cx - w / 2}" y="{cy - 19}" width="{w}" height="38" rx="19" fill="{fill}" stroke="{color}" stroke-width="2"/>')
    t(cx, cy + 8, label, 22, 700, color, "middle", limit=w - 16)


def branch(y, title, subs, port, x0=505, x1=810, whale_x=722):
    t(x0 + 16, y - 10, title, 22, 700, limit=whale_x - x0 - 24)
    line([(x0, y), (x1, y)])
    for i, s in enumerate(subs):
        t(x0 + 16, y + 26 + i * 26, s, 19, 400, GREY, limit=whale_x - x0 - 24)
    img("docker", whale_x, y - 34, 44, 24)
    for i, p in enumerate(port):
        t(whale_x + 22, y + 26 + i * 26, p, 19, 400, GREY, "middle", limit=x1 - whale_x + 20)


# ============ EC2 ============
EC2_X, EC2_W = 260, 1370
device(EC2_X, 20, EC2_W, 960, "EC2", "EC2", "k3s 자체 호스팅 · 관리형 서비스 없음", "#FFFBF5", "#C9A27A")
card(290, 160, 190, 80, "nginx", "Nginx", iw=60, ih=42)
line([(480, 200), (505, 200)]); line([(505, 145), (505, 660)])
branch(145, "프론트엔드", ["location /"], ["정적 빌드"])
branch(235, "견적 BE", ["location /api", "8080"], ["8080:8080"])
line([(650, 235), (650, 550)])
for y, port in ((340, "5432:5432"), (445, "9000:9000"), (550, "5672:5672")):
    line([(650, y), (810, y)]); img("docker", 722, y - 34, 44, 24); t(744, y + 26, port, 19, 400, GREY, "middle")
branch(660, "MQTT / WSS", ["location /mqtt", "8083"], ["wss:8083", "tcp:1883"])
CW = 300
card(810, 105, CW, 80, "react", "React", "웹 콘솔")
card(810, 195, CW, 80, "springboot", "Spring Boot", "견적 BE · REST", iw=46, ih=42)
card(810, 300, CW, 80, "postgres", "PostgreSQL", "견적 · KPI · 아티팩트", iw=56, ih=40)
card(810, 405, CW, 80, "minio", "MinIO", "영상 · 리플레이 · 히트맵", iw=50, ih=50)
card(810, 510, CW, 80, "rabbitmq", "RabbitMQ", "잡 큐 · GPU 런 큐 · DLQ")
card(810, 620, CW, 80, "emqx", "EMQX", "MQTT 브로커 · 상시")
CR = 810 + CW  # 1110

# k3s groups
GX, GW = 1220, 380
group(GX, 80, GW, 300, "계산 엔진 · k3s Job", "#F7F5FD")
card(GX + 20, 130, GW - 40, 80, "python", "계산 엔진 워커", "탐색 · θ 추론 · DES · KPI")
card(GX + 20, 260, GW - 40, 80, "k8s", "런 오케스트레이터", "GPU 런 큐 · k8s API", iw=60, ih=40)
GC = GX + GW / 2
line([(GC, 210), (GC, 260)]); t(GC + 13, 241, "상위 구성", 17, 400, GREY)
group(GX, 560, GW, 300, "시뮬 스택 · 런 단위 파드", "#FFF8EE")
card(GX + 20, 610, GW - 40, 80, "springboot", "시뮬 WMS", "H2 인메모리 · 런 스코프", iw=46, ih=42)
card(GX + 20, 740, GW - 40, 80, "python", "FMS 코어", "PIBT 헤딩 모델 · θ 통행 비용")
line([(GC, 340), (GC, 560)], LINE, "8 8")
t(GC + 16, 450, "k8s API · 파드 기동", 17, 400, GREY, "middle", extra=f'transform="rotate(-90 {GC + 16} 450)"')
# AMQP: RabbitMQ → worker
AX = CR + 62
line([(CR, 550), (AX, 550), (AX, 170), (GX + 20, 170)], ORANGE, "8 7", 2.4)
pill(AX, 497, "AMQP", ORANGE, "#FFF4E8", 96)
# MQTT: EMQX → sim WMS / FMS core
BX = CR + 40
line([(CR, 660), (BX, 660), (BX, 650), (GX + 20, 650)], BLUE, "8 7", 2.4)
line([(BX, 660), (BX, 780), (GX + 20, 780)], BLUE, "8 7", 2.4)
t(BX - 14, 715, "MQTT", 17, 700, BLUE, "middle", extra=f'transform="rotate(-90 {BX - 14} 715)"')

# ============ GPU server ============
GPX, GPW = 1700, 640
device(GPX, 260, GPW, 560, "nvidia", "L40S", "GPU 노드 × N · GPU 런 큐 소비자 = N", "#F6FBF4", "#8DBB7A", stack=2, badge="× N")
PC = 360
PX = GPX + (GPW - PC) / 2  # centered cards
card(PX, 370, PC, 80, "python", "sim-runner", "sim/control · 런 아티팩트")
card(PX, 500, PC, 80, "ros2", "amr-agent × N", "VDA 5050 · 그리드 프리미티브", iw=70, ih=40)
card(PX, 630, PC, 80, "nvidia", "Isaac Sim", "창고 씬 · iw.hub · 라이다", iw=46, ih=46)
PCX = PX + PC / 2
line([(PCX, 450), (PCX, 500)]); t(PCX + 14, 481, "스폰 · 녹화", 17, 400, GREY)
line([(PCX, 580), (PCX, 630)]); t(PCX + 14, 611, "DDS  /cmd_vel · /odom · /scan", 17, 400, GREY)
t(GPX + GPW / 2, 770, "노드당 로봇 N대 = 에이전트 N개 · Isaac 1 인스턴스", 18, 400, GREY, "middle")

# MQTT to GPU (EMQX ↔ agents/runner)
MX = EC2_X + EC2_W + 30  # between the two boxes
line([(960, 700), (960, 920), (MX, 920), (MX, 540), (PX, 540)], BLUE, "9 8", 2.6)
pill(1320, 920, "MQTT", BLUE, "#EEF1FC", 110)
t(1320, 957, "VDA 5050 order / state · sim/control · tailscale", 18, 400, GREY, "middle")
# S3 API: sim-runner → MinIO
SX = CR + 48
line([(PX, 395), (MX + 12, 395), (MX + 12, 40), (SX, 40), (SX, 445), (CR, 445)], GREEN, "9 8", 2.6)
pill(1320, 40, "S3 API", GREEN, "#EEF6E8", 120)
t(1395, 68, "presigned PUT · 런 아티팩트", 18, 400, GREY)

# ============ client (drawn last so nothing covers it) ============
add('<circle cx="90" cy="158" r="30" fill="#1f1f1f"/>')
add('<path d="M40 262 C40 212 140 212 140 262 L140 268 L40 268 Z" fill="#1f1f1f"/>')
t(90, 306, "고객 · 운영자", 24, 700, anchor="middle")
line([(145, 200), (258, 200)])
t(201, 184, "웹 콘솔", 20, 700, anchor="middle"); t(201, 228, "https / 443", 19, 400, GREY, "middle")

svg = "\n".join(E)
html = f'''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AMR 견적 서비스 아키텍처 (요약)</title>
<style>
  *{{box-sizing:border-box;margin:0;padding:0}}
  body{{background:#fff;font-family:{FONT};display:flex;justify-content:center;padding:24px}}
  svg{{width:100%;max-width:2400px;height:auto;display:block}}
  svg text{{font-family:{FONT}}}
</style>
</head>
<body>
<svg viewBox="0 0 {W} {H}" role="img" aria-label="AMR 도입 견적 서비스 아키텍처 요약. 고객이 nginx를 통해 EC2의 React·Spring Boot·PostgreSQL·MinIO·RabbitMQ·EMQX에 접근하고, k3s Job인 계산 엔진과 런 오케스트레이터가 시뮬 WMS·FMS 코어 파드를 기동하며, GPU 서버의 sim-runner·amr-agent·Isaac Sim이 MQTT와 S3 API로 연결된다.">
{svg}
</svg>
</body>
</html>
'''
io.open(OUT, "w", encoding="utf-8").write(html)
print("written", OUT, len(html))
for w_ in warn: print("WARN", w_)
