# -*- coding: utf-8 -*-
"""Generate docs/architecture.html — summary diagram in the reference (architecture.png) style."""
import io
import sys, base64, os
OUT="/home/lsmin124/workspace/PJT-2/docs/architecture.html"
EMBED=len(sys.argv)>1 and sys.argv[1]=="--embed"
IMGDIR="/home/lsmin124/workspace/PJT-2/docs/images"
OUT2=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","architecture_embed.html")
W,H=2200,1000
INK="#333333"; GREY="#666666"; LINE="#9a9a9a"; CARD="#d9d9d9"; DEV="#3a3a3a"
GREEN="#5f9e3a"; BLUE="#3f5fd1"; ORANGE="#e07b1a"; TEAL="#0f8b8d"
FONT='Pretendard, "Apple SD Gothic Neo", Inter, "Malgun Gothic", system-ui, sans-serif'
E=[]
def add(s): E.append(s)
def t(x,y,s,size=22,weight=400,fill=INK,anchor="start",extra=""):
    add(f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" fill="{fill}" text-anchor="{anchor}" {extra}>{s}</text>')
def img(name,x,y,w,h):
    href=f"images/{name}.png"
    if EMBED:
        href="data:image/png;base64,"+base64.b64encode(open(os.path.join(IMGDIR,name+".png"),"rb").read()).decode()
    add(f'<image href="{href}" x="{x}" y="{y}" width="{w}" height="{h}" preserveAspectRatio="xMidYMid meet"/>')
def device(x,y,w,h,icon,title,sub=None,iw=56,ih=56):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="26" fill="#fff" stroke="{DEV}" stroke-width="2"/>')
    img(icon,x+24,y+22,iw,ih)
    t(x+24+iw+12,y+22+ih*0.72,title,32,700)
    if sub: t(x+24+iw+12,y+22+ih*0.72+30,sub,19,400,GREY)
def group(x,y,w,h,title):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="20" fill="#fff" stroke="{CARD}" stroke-width="1.6"/>')
    t(x+(w-70)/2,y+30,title,22,700,"#444",anchor="middle"); img("docker",x+w-58,y+12,40,22)
def card(x,y,w,h,icon,label,sub=None,iw=46,ih=46,whale=False):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="18" fill="#fff" stroke="{CARD}" stroke-width="1.8"/>')
    if icon: img(icon,x+18,y+(h-ih)/2,iw,ih)
    lx=x+18+iw+16 if icon else x+22
    if sub:
        t(lx,y+h/2-2,label,26,700); t(lx,y+h/2+24,sub,17,400,GREY)
    else:
        t(lx,y+h/2+9,label,26,700)
    if whale: img("docker",x+w-50,y+h-30,40,22)
def line(pts,color=LINE,dash=None,width=2):
    d=" ".join(f"{'M' if i==0 else 'L'}{x} {y}" for i,(x,y) in enumerate(pts))
    da=f' stroke-dasharray="{dash}"' if dash else ""
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{da} stroke-linecap="round" stroke-linejoin="round"/>')
def pill(cx,cy,label,color,w=110):
    add(f'<rect x="{cx-w/2}" y="{cy-19}" width="{w}" height="{38}" rx="19" fill="#fff" stroke="{color}" stroke-width="2"/>')
    t(cx,cy+8,label,22,700,color,"middle")
def branch(y,title,subs,port=None,x0=505,x1=810,whale_x=720):
    t(x0+16,y-10,title,22,700)
    line([(x0,y),(x1,y)])
    for i,s in enumerate(subs): t(x0+16,y+26+i*26,s,19,400,GREY)
    if port is not None:
        img("docker",whale_x,y-34,44,24)
        for i,p in enumerate(port): t(whale_x+22,y+26+i*26,p,19,400,GREY,"middle")

# ---------------- EC2 ----------------
device(260,20,1220,960,"EC2","EC2 · t3.xlarge","k3s 자체 호스팅 · 관리형 서비스 없음")
card(290,160,190,80,"nginx","Nginx",iw=60,ih=42)
line([(480,200),(505,200)]); line([(505,145),(505,660)])
branch(145,"프론트엔드",["location /"],port=["정적 빌드"])
branch(235,"견적 BE",["location /api","8080"],port=["8080:8080"])
line([(650,235),(650,550)])
for y,port in ((340,"5432:5432"),(445,"9000:9000"),(550,"5672:5672")):
    line([(650,y),(810,y)]); img("docker",720,y-34,44,24); t(742,y+26,port,19,400,GREY,"middle")
branch(660,"MQTT / WSS",["location /mqtt","8083"],port=["wss:8083","tcp:1883"])
card(810,105,250,80,"react","React","웹 콘솔")
card(810,195,250,80,"springboot","Spring Boot","견적 BE · REST",iw=46,ih=42)
card(810,300,250,80,"postgres","PostgreSQL","견적 · KPI · 아티팩트 메타",iw=56,ih=40)
card(810,405,250,80,"minio","MinIO","영상 · 리플레이 · 히트맵",iw=50,ih=50)
card(810,510,250,80,"rabbitmq","RabbitMQ","잡 큐 · GPU 런 큐 · DLQ")
card(810,620,250,80,"emqx","EMQX","MQTT 브로커 · 상시")

# right column groups (k3s)
group(1150,80,300,300,"계산 엔진 · k3s Job")
card(1170,130,260,80,"python","계산 엔진 워커","구성 탐색 · θ 추론 · DES · KPI")
card(1170,260,260,80,"k8s","런 오케스트레이터","GPU 런 큐 · k8s API",iw=60,ih=40)
line([(1300,210),(1300,260)]); t(1313,241,"상위 구성",17,400,GREY)
group(1150,560,300,300,"시뮬 스택 · 런 단위 파드")
card(1170,610,260,80,"springboot","시뮬 WMS","H2 인메모리 · 런 스코프",iw=46,ih=42)
card(1170,740,260,80,"python","FMS 코어","PIBT 헤딩 모델 · θ 통행 비용")
# orchestrator → sim pods
line([(1300,340),(1300,560)],LINE,"8 8")
t(1316,450,"k8s API · 파드 기동",17,400,GREY,anchor="middle",extra='transform="rotate(-90 1316 450)"')
# AMQP: RabbitMQ → worker
line([(1060,550),(1122,550),(1122,170),(1170,170)],ORANGE,"8 7",2.4)
pill(1122,497,"AMQP",ORANGE,96)
# MQTT: EMQX → sim WMS / FMS core
line([(1060,660),(1108,660),(1108,650),(1170,650)],BLUE,"8 7",2.4)
line([(1108,660),(1108,780),(1170,780)],BLUE,"8 7",2.4)
t(1094,715,"MQTT",17,700,BLUE,anchor="middle",extra='transform="rotate(-90 1094 715)"')

# ---------------- GPU server ----------------
device(1540,250,620,560,"nvidia","GPU 서버 · RTX 5080","홈서버 · 네이티브 실행 · tailscale 아웃바운드")
card(1600,360,300,80,"python","sim-runner","sim/control · 런 아티팩트")
card(1600,490,300,80,"ros2","amr-agent × N","VDA 5050 · 그리드 프리미티브",iw=70,ih=40)
card(1600,620,300,80,"nvidia","Isaac Sim","창고 씬 · iw.hub · 라이다",iw=46,ih=46)
line([(1750,440),(1750,490)]); t(1764,471,"스폰 · 녹화",17,400,GREY)
line([(1750,570),(1750,620)]); t(1764,601,"DDS  /cmd_vel · /odom · /scan",17,400,GREY)
t(1750,760,"× N 로봇 = N 에이전트 · Isaac 1 인스턴스",18,400,GREY,anchor="middle")

# MQTT to GPU (EMQX ↔ agents/runner)
line([(935,700),(935,920),(1510,920),(1510,530),(1600,530)],BLUE,"9 8",2.6)
pill(1225,920,"MQTT",BLUE,110)
t(1225,957,"VDA 5050 order / state · sim/control · tailscale",18,400,GREY,"middle")
# S3 API: sim-runner → MinIO
line([(1600,385),(1520,385),(1520,40),(1108,40),(1108,445),(1060,445)],GREEN,"9 8",2.6)
pill(1230,40,"S3 API",GREEN,120)
t(1300,68,"presigned PUT · MP4 · 리플레이 · 히트맵",18,400,GREY)

# ---------------- client ----------------
add('<circle cx="85" cy="150" r="30" fill="#111"/>')
add('<path d="M35 250 C35 200 135 200 135 250 L135 262 L35 262 Z" fill="#111"/>')
t(85,300,"고객 · 운영자",24,700,anchor="middle")
line([(140,200),(258,200)])
t(199,184,"웹 콘솔 · 요청",20,700,anchor="middle"); t(199,228,"https / 443",19,400,GREY,"middle")


svg="\n".join(E)
html=f'''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AMR 견적 서비스 아키텍처 (요약)</title>
<style>
  *{{box-sizing:border-box;margin:0;padding:0}}
  body{{background:#fff;font-family:{FONT};display:flex;justify-content:center;padding:24px}}
  svg{{width:100%;max-width:2200px;height:auto;display:block}}
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
io.open(OUT2 if EMBED else OUT,"w",encoding="utf-8").write(html)
print("written",len(html))
