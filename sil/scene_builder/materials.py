"""머티리얼 바인딩 — OmniPBR(텍스처+틴트 / 단색), Simple_Warehouse MDL, 반투명 스트레치 랩 (pxr 필요)."""
from pxr import Gf, Sdf, UsdShade


def bind_omnipbr(stage, prim, name, diffuse_tex, normal_tex, tint):
    """OmniPBR + 원본 텍스처 + 틴트 — 순정 MDL은 톤 조절 입력을 못 믿어 직접 조립."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(diffuse_tex))
    sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(normal_tex))
    sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.75)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mtl)


def bind_pbr(stage, prim, name, color, normal_tex=None, rough=0.5, metal=0.0):
    """단색(+노멀맵) OmniPBR — 철골·샌드위치 패널 등 (텍스처 원색이 어두워
    틴트로 못 살리는 경우의 대안, OfficeWhite 실측 교훈의 일반화)."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    if normal_tex:
        sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(normal_tex))
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metal)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim() if hasattr(prim, "GetPrim") else prim).Bind(mtl)


def bind_mdl(stage, prim, name, mdl_file):
    """Simple_Warehouse MDL을 머티리얼로 정의해 prim(하위 상속)에 바인딩."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath(mdl_file), "mdl")
    sh.SetSourceAssetSubIdentifier(name, "mdl")
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim() if hasattr(prim, "GetPrim") else prim).Bind(mtl)


def define_stretch_wrap(stage):
    """래핑 파렛트용 반투명 OmniPBR — 기성 래핑 에셋 없음(합성). 반환: Material (원본 [2b] wrap_mtl)."""
    wrap_path = "/World/Looks/StretchWrap"
    wrap_mtl = UsdShade.Material.Define(stage, wrap_path)
    wsh = UsdShade.Shader.Define(stage, wrap_path + "/Shader")
    wsh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    wsh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    wsh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    # opacity 0.45는 밀키 불투명으로 렌더돼 속 박스가 안 비침(실측) — 0.22로
    wsh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.75, 0.78, 0.83))
    wsh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
    wsh.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(0.22)
    wsh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.1)
    wout = wsh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    wrap_mtl.CreateSurfaceOutput("mdl").ConnectToSource(wout)
    return wrap_mtl
