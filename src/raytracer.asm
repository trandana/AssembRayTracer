; =============================================================================
;  AssembRayTracer v4 - INTERACTIVE, abstract neon style, glass + more shapes.
;  x64 MASM, AVX-512 ray packets, multithreaded.
;  Fly around: mouse = look, W/A/S/D = move, E/Q = up/down, Shift = fast, Esc = quit.
;  Imports: kernel32, user32, gdi32 only (no C runtime).
;
;  * Every frame, one thread per physical core renders 16x4-pixel work units.
;  * One 4x4 tile = one "packet" of 16 rays, Structure-of-Arrays in zmm registers.
;  * Shapes: spheres, axis-aligned boxes, diamonds (octahedra) and a floor. Boxes and
;    diamonds share one slab intersector; object data lives in 16-entry tables that
;    vpermps looks up in a single instruction.
;  * Glass: refraction (Snell), Fresnel (Schlick), total internal reflection, tint,
;    light-tinted shadows. Rays carry per-channel (RGB) attenuation.
;  * Style: banded sky with a sun, toon-banded diffuse, hard highlights, neon rim
;    light, glowing grid on the floor, strong saturated colours, 2x2 AA, gamma 2.0.
;
;  Register conventions inside the render thread (all private, no ABI inside):
;    rbp  64-byte aligned frame base; "slots" below live at [rbp+offset]
;    k7   rays still alive   k6  rays that hit something this bounce
;    zmm30 = hit distance, zmm31 = hit object index (persist across calls)
; =============================================================================
option casemap:none

includelib kernel32.lib
includelib user32.lib
includelib gdi32.lib
extrn ExitProcess:proc
extrn GetSystemInfo:proc
extrn GetLogicalProcessorInformation:proc
extrn VirtualAlloc:proc
extrn CreateThread:proc
extrn WaitForMultipleObjects:proc
extrn CloseHandle:proc
extrn GetModuleHandleA:proc
extrn RegisterClassExA:proc
extrn CreateWindowExA:proc
extrn AdjustWindowRect:proc
extrn DefWindowProcA:proc
extrn DestroyWindow:proc
extrn PostQuitMessage:proc
extrn PeekMessageA:proc
extrn DispatchMessageA:proc
extrn LoadCursorA:proc
extrn SetCursor:proc
extrn GetDC:proc
extrn SetWindowTextA:proc
extrn GetClientRect:proc
extrn ClientToScreen:proc
extrn GetCursorPos:proc
extrn SetCursorPos:proc
extrn GetForegroundWindow:proc
extrn GetAsyncKeyState:proc
extrn StretchDIBits:proc
extrn QueryPerformanceCounter:proc
extrn QueryPerformanceFrequency:proc

; ---- configuration ----------------------------------------------------------
IMG_W    equ 1280                ; must be a multiple of 16
IMG_H    equ 720                 ; must be a multiple of 4
NS       equ 4                   ; spheres: objects 0..NS-1
NB       equ 3                   ; boxes
ND       equ 2                   ; diamonds
NOBJ     equ NS+NB+ND            ; the floor has object index NOBJ
MAXD     equ 6                   ; max bounces (glass needs a few: in, out, ...)
UNITS_X  equ IMG_W/16            ; work unit = 4 tiles = 16x4 pixels
NUNITS   equ UNITS_X*(IMG_H/4)

; ---- stack-frame slots (each 64 bytes = one zmm) -----------------------------
s_ox  equ 0                      ; current ray origin
s_oy  equ 64
s_oz  equ 128
s_dx  equ 192                    ; current ray direction
s_dy  equ 256
s_dz  equ 320
s_ar  equ 384                    ; accumulated colour of the packet
s_ag  equ 448
s_ab  equ 512
s_tr  equ 576                    ; per-channel attenuation (what later bounces may still contribute)
s_tg  equ 640
s_tb  equ 704
s_px  equ 768                    ; hit point
s_py  equ 832
s_pz  equ 896
s_nx  equ 960                    ; surface normal, flipped to face the ray
s_ny  equ 1024
s_nz  equ 1088
s_lr  equ 1152                   ; surface albedo
s_lg  equ 1216
s_lb  equ 1280
s_rf  equ 1344                   ; reflectivity
s_lx  equ 1408                   ; direction to the light
s_ly  equ 1472
s_lz  equ 1536
s_ld  equ 1600                   ; distance to the light
s_ix  equ 1664                   ; 1/direction (slab tests)
s_iy  equ 1728
s_iz  equ 1792
s_sr  equ 1856                   ; sum over the 4 AA samples
s_sg  equ 1920
s_sb  equ 1984
s_xmm equ 2048                   ; saved xmm6-15 (160 bytes)
s_mx0 equ 2208                   ; saved MXCSR
s_mx1 equ 2212                   ; new MXCSR
FRAME_SZ equ 2304

; ---- macros -------------------------------------------------------------------
VLD macro z, slot
    vmovaps z, zmmword ptr [rbp+slot]
endm
VST macro slot, z
    vmovaps zmmword ptr [rbp+slot], z
endm
BC macro z, mem                  ; broadcast a float from memory to all 16 lanes
    vbroadcastss z, dword ptr [mem]
endm

; dst = 1/sqrt(src): 14-bit estimate + one Newton-Raphson step (~23 bits)
RSQRT macro dst, src, t1, t2
    vrsqrt14ps dst, src
    vmulps t1, src, dst
    vmulps t1, t1, dst           ; x*y*y
    BC t2, kHalf
    vmulps t1, t1, t2            ; 0.5*x*y*y
    BC t2, kThreeHalves
    vsubps t2, t2, t1            ; 1.5 - 0.5*x*y*y
    vmulps dst, dst, t2
endm

; Sky colour of direction (d1,d2,d3): banded vertical gradient plus a sun with a ring.
; rr,gg,bb receive the colour; t1,t2 are scratch; uses mask k4.
SKY macro rr, gg, bb, d1, d2, d3, t1, t2
    vxorps t1, t1, t1
    vmaxps t1, d2, t1
    BC t2, kSkyK
    vmulps t1, t1, t2            ; stretch: the view only covers the lower part of the sky
    BC t2, kOne
    vminps t1, t1, t2            ; clamp(y*k, 0, 1)
    BC t2, kSBands
    vmulps t1, t1, t2
    vrndscaleps t1, t1, 1        ; floor: the sky is posterised into bands
    BC t2, kSInv
    vmulps t1, t1, t2
    BC rr, kHorR
    BC t2, kDzR
    vfmadd231ps rr, t2, t1
    BC gg, kHorG
    BC t2, kDzG
    vfmadd231ps gg, t2, t1
    BC bb, kHorB
    BC t2, kDzB
    vfmadd231ps bb, t2, t1
    BC t1, kSunX                 ; cos(angle to the sun)
    vmulps t1, t1, d1
    BC t2, kSunY
    vfmadd231ps t1, t2, d2
    BC t2, kSunZ
    vfmadd231ps t1, t2, d3
    BC t2, kRingCos
    vcmpps k4, t1, t2, 14
    vbroadcastss rr{k4}, dword ptr [ringR]
    vbroadcastss gg{k4}, dword ptr [ringG]
    vbroadcastss bb{k4}, dword ptr [ringB]
    BC t2, kSunCos
    vcmpps k4, t1, t2, 14
    vbroadcastss rr{k4}, dword ptr [sunR]
    vbroadcastss gg{k4}, dword ptr [sunG]
    vbroadcastss bb{k4}, dword ptr [sunB]
endm

; 1/direction of the ray (zmm3-5) into slots, used by every slab test
INVDIR macro
    BC zmm8, kOne
    vdivps zmm9, zmm8, zmm3
    VST s_ix, zmm9
    vdivps zmm9, zmm8, zmm4
    VST s_iy, zmm9
    vdivps zmm9, zmm8, zmm5
    VST s_iz, zmm9
endm

; ray (zmm0-2 origin, zmm3-5 dir) vs sphere o -> zmm14 = t, k1 = valid (t > eps).
; Takes the near root, or the far root when the ray starts inside (needed for glass).
SPHERE_T macro o
    BC zmm8, objCx+4*o
    vsubps zmm8, zmm0, zmm8                  ; oc = O - C
    BC zmm9, objCy+4*o
    vsubps zmm9, zmm1, zmm9
    BC zmm10, objCz+4*o
    vsubps zmm10, zmm2, zmm10
    vmulps zmm11, zmm8, zmm3
    vfmadd231ps zmm11, zmm9, zmm4
    vfmadd231ps zmm11, zmm10, zmm5           ; b = oc . D
    vmulps zmm12, zmm8, zmm8
    vfmadd231ps zmm12, zmm9, zmm9
    vfmadd231ps zmm12, zmm10, zmm10          ; oc . oc
    BC zmm13, objR2+4*o
    vsubps zmm12, zmm12, zmm13               ; c = oc.oc - r^2
    vmulps zmm13, zmm11, zmm11
    vsubps zmm13, zmm13, zmm12               ; disc = b^2 - c
    vxorps zmm14, zmm14, zmm14
    vcmpps k1, zmm13, zmm14, 13              ; disc >= 0
    vsqrtps zmm13, zmm13
    vsubps zmm14, zmm14, zmm11               ; -b
    vsubps zmm15, zmm14, zmm13               ; t0 = -b - sqrt
    vaddps zmm14, zmm14, zmm13               ; t1 = -b + sqrt
    BC zmm12, kEps
    vcmpps k4, zmm15, zmm12, 14              ; t0 > eps ?
    vmovaps zmm14{k4}, zmm15                 ; t = t0 if usable, else t1
    vcmpps k1{k1}, zmm14, zmm12, 14          ; t > eps
endm

; one axis of a box: folds the near/far slab distances into zmm12 (tnear) / zmm13 (tfar)
BOXAX macro cen, hal, nhal, inv, oc
    BC zmm8, cen
    vsubps zmm8, oc, zmm8                    ; o - c
    BC zmm9, hal
    vsubps zmm9, zmm9, zmm8                  ; h - (o-c)
    vmulps zmm9, zmm9, zmmword ptr [rbp+inv]
    BC zmm10, nhal
    vsubps zmm10, zmm10, zmm8                ; -h - (o-c)
    vmulps zmm10, zmm10, zmmword ptr [rbp+inv]
    vminps zmm11, zmm9, zmm10
    vmaxps zmm15, zmm9, zmm10
    vmaxps zmm12, zmm12, zmm11
    vminps zmm13, zmm13, zmm15
endm

; axis-aligned box o (slab method) -> zmm14 = t, k1 = valid
BOX_T macro o
    BC zmm12, kNegBig
    BC zmm13, kBig
    BOXAX objCx+4*o, objHx+4*o, objNHx+4*o, s_ix, zmm0
    BOXAX objCy+4*o, objHy+4*o, objNHy+4*o, s_iy, zmm1
    BOXAX objCz+4*o, objHz+4*o, objNHz+4*o, s_iz, zmm2
    vcmpps k1, zmm12, zmm13, 2               ; tnear <= tfar
    BC zmm8, kEps
    vcmpps k1{k1}, zmm13, zmm8, 14           ; tfar > eps
    vcmpps k4, zmm12, zmm8, 2                ; tnear <= eps: ray starts inside, use tfar
    vmovaps zmm14, zmm12
    vmovaps zmm14{k4}, zmm13
endm

; one of the four slab directions (1,+-1,+-1)/sqrt3 of a diamond (octahedron)
DSLAB macro opy, opz
    opy zmm11, zmm3, zmm4
    opz zmm11, zmm11, zmm5
    vmulps zmm11, zmm11, zmm16               ; a = n . D
    opy zmm12, zmm8, zmm9
    opz zmm12, zmm12, zmm10
    vmulps zmm12, zmm12, zmm16               ; m = n . (O - C)
    vdivps zmm11, zmm17, zmm11               ; 1/a
    vsubps zmm21, zmm15, zmm12
    vmulps zmm21, zmm21, zmm11               ; t1 = ( w - m)/a
    vsubps zmm22, zmm18, zmm12
    vmulps zmm22, zmm22, zmm11               ; t2 = (-w - m)/a
    vminps zmm23, zmm21, zmm22
    vmaxps zmm24, zmm21, zmm22
    vmaxps zmm19, zmm19, zmm23
    vminps zmm20, zmm20, zmm24
endm

; diamond o: |x|+|y|+|z| <= R, i.e. four slabs of half-width w = R/sqrt3 -> zmm14 = t, k1 = valid
DIAMOND_T macro o
    BC zmm8, objCx+4*o
    vsubps zmm8, zmm0, zmm8
    BC zmm9, objCy+4*o
    vsubps zmm9, zmm1, zmm9
    BC zmm10, objCz+4*o
    vsubps zmm10, zmm2, zmm10
    BC zmm15, objHx+4*o                      ; w
    BC zmm18, objNHx+4*o                     ; -w
    BC zmm16, kInvSqrt3
    BC zmm17, kOne
    BC zmm19, kNegBig
    BC zmm20, kBig
    DSLAB vaddps, vaddps
    DSLAB vaddps, vsubps
    DSLAB vsubps, vaddps
    DSLAB vsubps, vsubps
    vcmpps k1, zmm19, zmm20, 2
    BC zmm8, kEps
    vcmpps k1{k1}, zmm20, zmm8, 14
    vcmpps k4, zmm19, zmm8, 2
    vmovaps zmm14, zmm19
    vmovaps zmm14{k4}, zmm20
endm

; ray vs plane y=0 -> zmm14 = t, k1 = valid
PLANE_T macro
    BC zmm8, kNegEps
    vcmpps k1, zmm4, zmm8, 1                 ; dy < -1e-6
    vdivps zmm14, zmm1, zmm4                 ; oy / dy
    vxorps zmm10, zmm10, zmm10
    vsubps zmm14, zmm10, zmm14               ; t = -oy/dy
    BC zmm15, kEps
    vcmpps k1{k1}, zmm14, zmm15, 14
endm

; keep the nearest hit so far (zmm6 = t, zmm7 = object index)
UPDBEST macro o
    vcmpps k1{k1}, zmm14, zmm6, 1
    vmovaps zmm6{k1}, zmm14
    mov eax, o
    vpbroadcastd zmm7{k1}, eax
endm

; shadow-ray accounting for object o (zmm14/k1 from its test): k2 = opaque blockers, k3 = glass blockers
OCCTEST macro o
    vcmpps k1{k1}, zmm14, zmm6, 1            ; closer than the light
    BC zmm8, matTr+4*o
    vxorps zmm9, zmm9, zmm9
    vcmpps k4{k1}, zmm8, zmm9, 14            ; ...and the object is glass
    korw k3, k3, k4
    kandnw k1, k4, k1                        ; hit and not glass
    korw k2, k2, k1
    kandnw k4, k2, k6                        ; lanes of interest not yet blocked
    kortestw k4, k4
    jz occ_done
endm

; channel -> 8-bit: clamp(sum/4), sqrt (gamma 2), *255, round
PACKCH macro z, slot
    VLD z, slot
    vmulps z, z, zmm4
    vmaxps z, z, zmm6
    vminps z, z, zmm5
    vsqrtps z, z
    vmulps z, z, zmm7
    vcvtps2dq z, z
endm

.data
; ---- BMP header (reused as BITMAPINFOHEADER for StretchDIBits) ----------------
bmphdr  db 'B','M'
        dd 54 + IMG_W*IMG_H*4
        dd 0
        dd 54
        dd 40
        dd IMG_W
        dd -IMG_H
        dw 1
        dw 32
        dd 0
        dd IMG_W*IMG_H*4
        dd 2835, 2835, 0, 0

clsName   db "AsmRT",0
titleInit db "AssembRayTracer",0
titleA    db "AssembRayTracer | ",0
titleB    db " fps | ",0
titleC    db " ms/frame | WASD move, mouse look, Q/E down/up, Shift fast, Esc quit",0

; ---- scene: 16-entry tables indexed by object number --------------------------
;   0-3 spheres, 4-6 boxes, 7-8 diamonds, 9 = floor, 10-15 padding
;   (vpermps uses these tables as 16-way in-register lookups)
objCx   real4  0.0, -2.4,  2.4,  0.8, -3.4, -0.9,  3.6,  0.3,  2.2,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objCy   real4  1.0,  0.9,  0.6,  0.35, 0.82, 0.47, 1.22, 2.3,  1.1,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objCz   real4  3.0,  1.6,  1.0,  0.0,  4.4,  0.9,  3.6,  6.0, -0.8,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objHx   real4  1.0,  0.9,  0.6,  0.35, 0.8,  0.45, 0.45, 0.8,  0.55,  0.0,0.0,0.0,0.0,0.0,0.0,0.0   ; radius / half-size x / diamond w
objHy   real4  0.0,  0.0,  0.0,  0.0,  0.8,  0.45, 1.2,  0.0,  0.0,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objHz   real4  0.0,  0.0,  0.0,  0.0,  0.8,  0.45, 0.45, 0.0,  0.0,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objNHx  real4 -1.0, -0.9, -0.6, -0.35,-0.8, -0.45,-0.45,-0.8, -0.55,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objNHy  real4  0.0,  0.0,  0.0,  0.0, -0.8, -0.45,-1.2,  0.0,  0.0,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objNHz  real4  0.0,  0.0,  0.0,  0.0, -0.8, -0.45,-0.45, 0.0,  0.0,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
objIHx  real4  1.0,  1.1111111, 1.6666667, 2.8571429, 1.25, 2.2222222, 2.2222222, 1.25, 1.8181818,  0.0,0.0,0.0,0.0,0.0,0.0,0.0
objIHy  real4  0.0,  0.0,  0.0,  0.0,  1.25, 2.2222222, 0.8333333, 0.0, 0.0,  0.0,0.0,0.0,0.0,0.0,0.0,0.0
objIHz  real4  0.0,  0.0,  0.0,  0.0,  1.25, 2.2222222, 2.2222222, 0.0, 0.0,  0.0,0.0,0.0,0.0,0.0,0.0,0.0
objR2   real4  1.0,  0.81, 0.36, 0.1225, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0,0.0,0.0,0.0,0.0,0.0,0.0
matAr   real4  1.00, 0.30, 1.00, 0.55, 0.10, 1.00, 0.65, 1.00, 0.20,  0.0,0.0,0.0,0.0,0.0,0.0,0.0
matAg   real4  0.10, 1.00, 0.45, 1.00, 0.35, 0.90, 0.10, 0.10, 1.00,  0.0,0.0,0.0,0.0,0.0,0.0,0.0
matAb   real4  0.70, 0.95, 0.00, 0.10, 1.00, 0.15, 1.00, 0.20, 0.50,  0.0,0.0,0.0,0.0,0.0,0.0,0.0
matRf   real4  0.65, 0.00, 0.12, 0.15, 0.30, 0.00, 0.20, 0.30, 0.00,  0.22,0.0,0.0,0.0,0.0,0.0,0.0
matTr   real4  0.0,  1.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,   0.0,0.0,0.0,0.0,0.0,0.0,0.0   ; 1 = glass

; ---- camera and light ---------------------------------------------------------
camX      real4  0.0              ; camera position (moved by update_camera)
camY      real4  1.7
camZ      real4 -6.5
yaw       real4  0.0              ; radians, 0 looks along +z, positive turns right
pitch     real4 -0.14             ; radians, positive looks up
kSens     real4  0.0025           ; radians per mouse pixel
kSpeed    real4  4.0              ; units per second
kShift    real4  3.0              ; Shift speed multiplier
kPitchMax real4  1.5
kPitchMin real4 -1.5
kMinY     real4  0.25             ; lowest camera height (stay above the floor)
kMaxDt    real4  0.1              ; longest frame time used for movement
lightX    real4 -6.0
lightY    real4  5.0
lightZ    real4 -1.0

; ---- palette and style --------------------------------------------------------
chkAr     real4  0.50             ; floor squares: magenta ...
chkAg     real4  0.03
chkAb     real4  0.45
chkBr     real4  0.03             ; ... and deep blue
chkBg     real4  0.06
chkBb     real4  0.30
lineR     real4  0.0              ; neon cyan grid lines
lineG     real4  1.0
lineB     real4  0.85
kLineW    real4  0.03             ; line half-width in floor units
kLineK    real4  0.004            ; extra half-width per unit of distance
kHorR     real4  1.00             ; sky horizon: hot pink
kHorG     real4  0.30
kHorB     real4  0.50
kDzR      real4 -0.92             ; zenith (deep indigo) minus horizon
kDzG      real4 -0.27
kDzB      real4 -0.10
kSkyK     real4  2.4              ; sky gradient stretch
kSBands   real4  7.0              ; number of sky bands
kSInv     real4  0.1428571
kSunX     real4  0.2507            ; sun direction (unit)
kSunY     real4  0.2206
kSunZ     real4  0.9426
kSunCos   real4  0.9976           ; disc ~4 degrees
kRingCos  real4  0.9925           ; ring ~7 degrees
sunR      real4  1.0
sunG      real4  0.95
sunB      real4  0.55
ringR     real4  1.0
ringG     real4  0.45
ringB     real4  0.15
kToon     real4  4.0              ; diffuse light is quantised into this many bands
kThird    real4  0.3333333
kSpecThr  real4  0.6              ; hard highlight threshold on (N.H)^32
kSpecHard real4  0.9
kRim      real4  1.3              ; neon rim light strength
kRimAdd   real4  0.3
kGlassSh  real4  0.55             ; light that passes through glass
kEtaOut   real4  0.6666667        ; 1 / index of refraction (air -> glass)
kEtaIn    real4  1.5              ; glass -> air
kF0       real4  0.04             ; Schlick reflectance at normal incidence
kOneMinusF0 real4 0.96
kAmbR     real4  0.26
kAmbG     real4  0.24
kAmbB     real4  0.34
kDiffI    real4  0.95
kFogK     real4  0.018

; ---- misc constants -----------------------------------------------------------
kOne      real4  1.0
kHalf     real4  0.5
kThreeHalves real4 1.5
k255      real4  255.0
kQuarter  real4  0.25
kEps      real4  0.001
kEps2     real4  0.002
kNegEps2  real4 -0.002
kNegEps   real4 -1.0e-6
kBig      real4  1.0e30
kNegBig   real4 -1.0e30
kMinAtten real4  0.01
kInvSqrt3 real4  0.57735027
kSignMask dd 80000000h
kAbsMask  dd 7FFFFFFFh
jit       real4  0.25,0.25, 0.75,0.25, 0.25,0.75, 0.75,0.75      ; 2x2 AA
laneX     real4  0.0,1.0,2.0,3.0, 0.0,1.0,2.0,3.0, 0.0,1.0,2.0,3.0, 0.0,1.0,2.0,3.0
laneY     real4  0.0,0.0,0.0,0.0, 1.0,1.0,1.0,1.0, 2.0,2.0,2.0,2.0, 3.0,3.0,3.0,3.0

.data?
padA        db 64 dup(?)         ; keep the counter alone on its cache line
unitCounter dd ?
padB        db 64 dup(?)
handles     dq 64 dup(?)
sysinfo     db 48 dup(?)
lpiBuf      db 4096 dup(?)       ; SYSTEM_LOGICAL_PROCESSOR_INFORMATION records
lpiLen      dd ?
qpf         dq ?
fbPtr       dq ?
nthr        dd ?
fPix        real4 ?              ; 1/IMG_H  (= 2*tan(fov/2)/IMG_H, tan = 0.5)
fHalfW      real4 ?              ; aspect * 0.5
numtmp      db 32 dup(?)
outbuf      db 256 dup(?)
camFx real4 ?                     ; camera basis (forward, right, up), rebuilt every frame
camFy real4 ?
camFz real4 ?
camRx real4 ?
camRy real4 ?
camRz real4 ?
camUx real4 ?
camUy real4 ?
camUz real4 ?
cosY real4 ?
sinY real4 ?
cosP real4 ?
sinP real4 ?
dtf  real4 ?                      ; last frame time, seconds
hinst       dq ?
hwnd        dq ?
hdc         dq ?
wc          db 80 dup(?)          ; WNDCLASSEXA
msgbuf      db 48 dup(?)          ; MSG
rectb       dd 4 dup(?)
ctrPt       dd 2 dup(?)           ; window centre in screen coordinates
curPt       dd 2 dup(?)
skipMouse   db ?
shiftFlag   db ?
lastTicks   dq ?
lastTitle   dq ?
tnow        dq ?
tmpUs       dq ?
tmpFrames   dq ?
frames      dd ?

.code

; -----------------------------------------------------------------------------
; intersect: closest hit for 16 rays.
;   in : zmm0-2 origin, zmm3-5 direction (normalised)
;   out: zmm6 = t (1e30 = none), zmm7 = object index (-1 none, 0..NOBJ-1, NOBJ = floor)
;   clobbers zmm8-24, k1, k4, eax and the s_ix..s_iz slots
; -----------------------------------------------------------------------------
intersect proc
    BC zmm6, kBig
    vpternlogd zmm7, zmm7, zmm7, 0FFh        ; all ones = -1
    INVDIR
    for sn,<0,1,2,3>
        SPHERE_T sn
        UPDBEST sn
    endm
    for bn,<4,5,6>
        BOX_T bn
        UPDBEST bn
    endm
    for dn,<7,8>
        DIAMOND_T dn
        UPDBEST dn
    endm
    PLANE_T
    UPDBEST NOBJ
    ret
intersect endp

; -----------------------------------------------------------------------------
; occluded: shadow rays, with early exit once every lane in k6 is blocked by an opaque object.
;   in : zmm0-2 origin, zmm3-5 direction, zmm6 = distance to light, k6 = lanes of interest
;   out: k2 = lanes blocked by an opaque object, k3 = lanes blocked by glass
;   clobbers zmm8-24, k1, k4
; -----------------------------------------------------------------------------
occluded proc
    kxorw k2, k2, k2
    kxorw k3, k3, k3
    INVDIR
    for sn,<0,1,2,3>
        SPHERE_T sn
        OCCTEST sn
    endm
    for bn,<4,5,6>
        BOX_T bn
        OCCTEST bn
    endm
    for dn,<7,8>
        DIAMOND_T dn
        OCCTEST dn
    endm
    PLANE_T
    vcmpps k1{k1}, zmm14, zmm6, 1
    korw k2, k2, k1
occ_done:
    ret
occluded endp

; -----------------------------------------------------------------------------
; trace_packet: ray in s_ox..s_dz slots -> colour in s_ar/s_ag/s_ab slots.
;   Persistent across calls inside a bounce: zmm30 = t, zmm31 = object index,
;   k7 = rays alive, k6 = rays that hit, r9d = glass lanes, r10d = lanes inside glass,
;   r11d = floor grid-line lanes. Everything else lives in the frame slots.
; -----------------------------------------------------------------------------
trace_packet proc
    vxorps zmm0, zmm0, zmm0
    VST s_ar, zmm0
    VST s_ag, zmm0
    VST s_ab, zmm0
    BC zmm0, kOne
    VST s_tr, zmm0                           ; attenuation = 1 in every channel
    VST s_tg, zmm0
    VST s_tb, zmm0
    kxnorw k7, k7, k7                        ; all 16 rays alive
    xor ebx, ebx                             ; depth

bounce:
    VLD zmm0, s_ox
    VLD zmm1, s_oy
    VLD zmm2, s_oz
    VLD zmm3, s_dx
    VLD zmm4, s_dy
    VLD zmm5, s_dz
    call intersect
    vmovdqa32 zmm31, zmm7                    ; object index per lane
    vmovaps zmm30, zmm6                      ; t per lane
    vpxord zmm8, zmm8, zmm8
    vpcmpd k6{k7}, zmm31, zmm8, 5            ; k6 = alive AND index >= 0  (hit)
    kandnw k5, k6, k7                        ; k5 = alive AND missed

    ; ---- missed rays pick up attenuated sky colour and die ----
    kortestw k5, k5
    jz nomiss
    VLD zmm3, s_dx
    VLD zmm4, s_dy
    VLD zmm5, s_dz
    SKY zmm0, zmm1, zmm2, zmm3, zmm4, zmm5, zmm6, zmm7
    VLD zmm6, s_tr
    VLD zmm7, s_ar
    vfmadd231ps zmm7{k5}, zmm0, zmm6
    VST s_ar, zmm7
    VLD zmm6, s_tg
    VLD zmm7, s_ag
    vfmadd231ps zmm7{k5}, zmm1, zmm6
    VST s_ag, zmm7
    VLD zmm6, s_tb
    VLD zmm7, s_ab
    vfmadd231ps zmm7{k5}, zmm2, zmm6
    VST s_ab, zmm7
nomiss:
    kortestw k6, k6
    jz finish                                ; nothing hit: packet is done

    ; ---- P = O + D*t ----
    VLD zmm16, s_ox
    VLD zmm0, s_dx
    vfmadd231ps zmm16, zmm0, zmm30
    VST s_px, zmm16
    VLD zmm17, s_oy
    VLD zmm0, s_dy
    vfmadd231ps zmm17, zmm0, zmm30
    VST s_py, zmm17
    VLD zmm18, s_oz
    VLD zmm0, s_dz
    vfmadd231ps zmm18, zmm0, zmm30
    VST s_pz, zmm18

    ; ---- per-lane object centre (16-way table lookup) and offset of P from it ----
    vpermps zmm0, zmm31, zmmword ptr [objCx]
    vpermps zmm1, zmm31, zmmword ptr [objCy]
    vpermps zmm2, zmm31, zmmword ptr [objCz]
    vsubps zmm3, zmm16, zmm0                 ; x' = P - C
    vsubps zmm4, zmm17, zmm1
    vsubps zmm5, zmm18, zmm2
    vxorps zmm19, zmm19, zmm19               ; normal (outward) accumulates here
    vxorps zmm20, zmm20, zmm20
    vxorps zmm21, zmm21, zmm21

    ; ---- which kind of object did each lane hit? ----
    mov eax, NS
    vpbroadcastd zmm8, eax
    mov eax, NS+NB
    vpbroadcastd zmm9, eax
    mov eax, NOBJ
    vpbroadcastd zmm10, eax
    vpcmpd k1{k6}, zmm31, zmm8, 1            ; k1 = spheres
    vpcmpd k2{k6}, zmm31, zmm8, 5            ; idx >= NS
    vpcmpd k3{k2}, zmm31, zmm9, 1            ; k3 = boxes
    vpcmpd k4{k2}, zmm31, zmm9, 5            ; idx >= NS+NB
    vpcmpd k4{k4}, zmm31, zmm10, 1           ; k4 = diamonds
    vpcmpeqd k2{k6}, zmm31, zmm10            ; k2 = floor

    ; sphere: N = (P-C)/r
    vpermps zmm6, zmm31, zmmword ptr [objIHx]
    vmulps zmm19{k1}, zmm3, zmm6
    vmulps zmm20{k1}, zmm4, zmm6
    vmulps zmm21{k1}, zmm5, zmm6

    ; box: the face with the largest |P-C|/halfsize is the one that was hit
    vpermps zmm7, zmm31, zmmword ptr [objIHy]
    vpermps zmm8, zmm31, zmmword ptr [objIHz]
    vmulps zmm11, zmm3, zmm6
    vmulps zmm12, zmm4, zmm7
    vmulps zmm13, zmm5, zmm8                 ; q = (P-C)/h per axis
    BC zmm14, kAbsMask
    vandps zmm22, zmm11, zmm14
    vandps zmm23, zmm12, zmm14
    vandps zmm24, zmm13, zmm14               ; |q|
    BC zmm25, kSignMask
    BC zmm26, kOne
    vcmpps k5{k3}, zmm22, zmm23, 13
    vcmpps k5{k5}, zmm22, zmm24, 13          ; x face
    vcmpps k1{k3}, zmm23, zmm24, 13
    kandnw k1, k5, k1                        ; y face
    vandps zmm27, zmm11, zmm25
    vorps zmm27, zmm27, zmm26                ; sign(qx)
    vmovaps zmm19{k5}, zmm27
    vandps zmm27, zmm12, zmm25
    vorps zmm27, zmm27, zmm26
    vmovaps zmm20{k1}, zmm27
    korw k5, k5, k1
    kandnw k5, k5, k3                        ; z face
    vandps zmm27, zmm13, zmm25
    vorps zmm27, zmm27, zmm26
    vmovaps zmm21{k5}, zmm27

    ; diamond: of the 4 slabs pick the one with the largest |n.(P-C)|
    vaddps zmm6, zmm3, zmm4
    vaddps zmm6, zmm6, zmm5                  ; u1 = x+y+z
    vaddps zmm7, zmm3, zmm4
    vsubps zmm7, zmm7, zmm5                  ; u2 = x+y-z
    vsubps zmm8, zmm3, zmm4
    vaddps zmm8, zmm8, zmm5                  ; u3 = x-y+z
    vsubps zmm9, zmm3, zmm4
    vsubps zmm9, zmm9, zmm5                  ; u4 = x-y-z
    BC zmm14, kAbsMask
    vandps zmm10, zmm6, zmm14
    vandps zmm11, zmm7, zmm14
    vandps zmm12, zmm8, zmm14
    vandps zmm13, zmm9, zmm14
    vpxord zmm15, zmm15, zmm15               ; selected slab (0..3)
    vcmpps k5, zmm11, zmm10, 14
    vmovaps zmm10{k5}, zmm11
    vmovaps zmm6{k5}, zmm7
    mov eax, 1
    vpbroadcastd zmm15{k5}, eax
    vcmpps k5, zmm12, zmm10, 14
    vmovaps zmm10{k5}, zmm12
    vmovaps zmm6{k5}, zmm8
    mov eax, 2
    vpbroadcastd zmm15{k5}, eax
    vcmpps k5, zmm13, zmm10, 14
    vmovaps zmm10{k5}, zmm13
    vmovaps zmm6{k5}, zmm9
    mov eax, 3
    vpbroadcastd zmm15{k5}, eax
    vandps zmm16, zmm6, zmm25
    vorps zmm16, zmm16, zmm26                ; sign of the chosen slab
    BC zmm17, kInvSqrt3
    vmulps zmm16, zmm16, zmm17               ; sign / sqrt3
    vmovaps zmm19{k4}, zmm16                 ; nx
    mov eax, 2
    vpbroadcastd zmm18, eax
    vpcmpd k5, zmm15, zmm18, 5               ; slab 2,3 have ny = -
    vxorps zmm22, zmm22, zmm22
    vmovaps zmm17, zmm16
    vsubps zmm17{k5}, zmm22, zmm16
    vmovaps zmm20{k4}, zmm17                 ; ny
    mov eax, 1
    vpbroadcastd zmm18, eax
    vptestmd k5, zmm15, zmm18                ; slab 1,3 have nz = -
    vmovaps zmm17, zmm16
    vsubps zmm17{k5}, zmm22, zmm16
    vmovaps zmm21{k4}, zmm17                 ; nz

    ; floor: N = (0,1,0)
    vbroadcastss zmm20{k2}, dword ptr [kOne]

    ; ---- make the normal face the ray; remember which lanes are inside an object ----
    VLD zmm22, s_dx
    VLD zmm23, s_dy
    VLD zmm24, s_dz
    vmulps zmm0, zmm19, zmm22
    vfmadd231ps zmm0, zmm20, zmm23
    vfmadd231ps zmm0, zmm21, zmm24           ; D.N
    vxorps zmm1, zmm1, zmm1
    vcmpps k1{k6}, zmm0, zmm1, 14            ; D.N > 0 : the ray is inside
    kmovw r10d, k1
    vsubps zmm19{k1}, zmm1, zmm19
    vsubps zmm20{k1}, zmm1, zmm20
    vsubps zmm21{k1}, zmm1, zmm21
    VST s_nx, zmm19
    VST s_ny, zmm20
    VST s_nz, zmm21

    ; ---- material lookup ----
    vpermps zmm25, zmm31, zmmword ptr [matAr]
    vpermps zmm26, zmm31, zmmword ptr [matAg]
    vpermps zmm27, zmm31, zmmword ptr [matAb]
    vpermps zmm28, zmm31, zmmword ptr [matRf]
    vpermps zmm29, zmm31, zmmword ptr [matTr]
    vcmpps k1{k6}, zmm29, zmm1, 14           ; transparency > 0 : glass
    kmovw r9d, k1

    ; floor: checkerboard colour and neon grid lines (k2 = floor lanes)
    VLD zmm0, s_px
    VLD zmm4, s_pz
    vrndscaleps zmm1, zmm0, 1                ; floor(x)
    vsubps zmm2, zmm0, zmm1                  ; fract(x)
    vcvttps2dq zmm3, zmm1
    vrndscaleps zmm5, zmm4, 1                ; floor(z)
    vsubps zmm6, zmm4, zmm5                  ; fract(z)
    vcvttps2dq zmm7, zmm5
    vpaddd zmm3, zmm3, zmm7
    mov eax, 1
    vpbroadcastd zmm8, eax
    vptestmd k3{k2}, zmm3, zmm8              ; odd -> dark square
    kandnw k4, k3, k2                        ; even -> light square
    vbroadcastss zmm25{k4}, dword ptr [chkAr]
    vbroadcastss zmm26{k4}, dword ptr [chkAg]
    vbroadcastss zmm27{k4}, dword ptr [chkAb]
    vbroadcastss zmm25{k3}, dword ptr [chkBr]
    vbroadcastss zmm26{k3}, dword ptr [chkBg]
    vbroadcastss zmm27{k3}, dword ptr [chkBb]
    BC zmm9, kOne
    vsubps zmm10, zmm9, zmm2
    vminps zmm10, zmm10, zmm2                ; distance to the nearest x grid line
    vsubps zmm11, zmm9, zmm6
    vminps zmm11, zmm11, zmm6                ; ... and z grid line
    vminps zmm10, zmm10, zmm11
    BC zmm11, kLineW
    BC zmm12, kLineK
    vfmadd231ps zmm11, zmm30, zmm12          ; half-width grows with distance (anti-aliasing)
    vcmpps k5{k2}, zmm10, zmm11, 1
    kmovw r11d, k5                           ; r11d = grid-line lanes
    VST s_lr, zmm25
    VST s_lg, zmm26
    VST s_lb, zmm27
    VST s_rf, zmm28

    ; ---- L = normalize(light - P), dist ----
    VLD zmm16, s_px
    VLD zmm17, s_py
    VLD zmm18, s_pz
    BC zmm8, lightX
    vsubps zmm8, zmm8, zmm16
    BC zmm9, lightY
    vsubps zmm9, zmm9, zmm17
    BC zmm10, lightZ
    vsubps zmm10, zmm10, zmm18
    vmulps zmm11, zmm8, zmm8
    vfmadd231ps zmm11, zmm9, zmm9
    vfmadd231ps zmm11, zmm10, zmm10          ; dist^2
    RSQRT zmm12, zmm11, zmm13, zmm14
    vmulps zmm8, zmm8, zmm12
    vmulps zmm9, zmm9, zmm12
    vmulps zmm10, zmm10, zmm12
    vmulps zmm15, zmm11, zmm12               ; dist = dist^2 * (1/dist)
    VST s_lx, zmm8
    VST s_ly, zmm9
    VST s_lz, zmm10
    VST s_ld, zmm15

    ; ---- shadow ray from P + N*eps toward the light ----
    BC zmm7, kEps2
    vmovaps zmm0, zmm16
    vfmadd231ps zmm0, zmm19, zmm7
    vmovaps zmm1, zmm17
    vfmadd231ps zmm1, zmm20, zmm7
    vmovaps zmm2, zmm18
    vfmadd231ps zmm2, zmm21, zmm7
    vmovaps zmm3, zmm8
    vmovaps zmm4, zmm9
    vmovaps zmm5, zmm10
    vmovaps zmm6, zmm15
    call occluded                            ; k2 = opaque blockers, k3 = glass blockers
    kandnw k5, k2, k6                        ; k5 = lit lanes (hit and not blocked)

    ; ---- reload the surface state (everything below is call-free) ----
    VLD zmm16, s_nx
    VLD zmm17, s_ny
    VLD zmm18, s_nz
    VLD zmm19, s_lx
    VLD zmm20, s_ly
    VLD zmm21, s_lz
    VLD zmm22, s_dx
    VLD zmm23, s_dy
    VLD zmm24, s_dz
    VLD zmm25, s_lr
    VLD zmm26, s_lg
    VLD zmm27, s_lb
    VLD zmm28, s_rf
    vxorps zmm29, zmm29, zmm29

    ; ---- toon diffuse: N.L quantised into bands; glass shadows only dim the light ----
    BC zmm0, kOne
    vbroadcastss zmm0{k3}, dword ptr [kGlassSh]
    vmulps zmm1, zmm16, zmm19
    vfmadd231ps zmm1, zmm17, zmm20
    vfmadd231ps zmm1, zmm18, zmm21
    vmaxps zmm1, zmm1, zmm29                 ; N.L
    BC zmm2, kToon
    vmulps zmm1, zmm1, zmm2
    vrndscaleps zmm1, zmm1, 1                ; floor(N.L * bands)
    BC zmm2, kThird
    vmulps zmm1, zmm1, zmm2
    BC zmm2, kOne
    vminps zmm1, zmm1, zmm2
    vmovaps zmm1{k5}{z}, zmm1                ; no light in opaque shadow
    vmulps zmm1, zmm1, zmm0                  ; ndlq * light factor

    ; ---- hard cartoon highlight: H = normalize(L - D), (N.H)^32 above a threshold ----
    vsubps zmm3, zmm19, zmm22
    vsubps zmm4, zmm20, zmm23
    vsubps zmm5, zmm21, zmm24
    vmulps zmm6, zmm3, zmm3
    vfmadd231ps zmm6, zmm4, zmm4
    vfmadd231ps zmm6, zmm5, zmm5
    RSQRT zmm7, zmm6, zmm8, zmm9
    vmulps zmm3, zmm3, zmm7
    vmulps zmm4, zmm4, zmm7
    vmulps zmm5, zmm5, zmm7
    vmulps zmm6, zmm16, zmm3
    vfmadd231ps zmm6, zmm17, zmm4
    vfmadd231ps zmm6, zmm18, zmm5            ; N.H
    vmaxps zmm6, zmm6, zmm29
    vmulps zmm6, zmm6, zmm6                  ; ^2
    vmulps zmm6, zmm6, zmm6                  ; ^4
    vmulps zmm6, zmm6, zmm6                  ; ^8
    vmulps zmm6, zmm6, zmm6                  ; ^16
    vmulps zmm6, zmm6, zmm6                  ; ^32
    BC zmm7, kSpecThr
    vcmpps k1{k5}, zmm6, zmm7, 14
    mov eax, NOBJ
    vpbroadcastd zmm12, eax
    vpcmpeqd k2, zmm31, zmm12                ; the floor gets no hard highlight (it would be a huge white blob)
    kandnw k1, k2, k1
    vxorps zmm8, zmm8, zmm8
    vbroadcastss zmm8{k1}, dword ptr [kSpecHard]     ; spec (white, hard edged)

    ; ---- neon rim light: (1 - cos)^3 * (albedo + 0.3), not on the floor ----
    vmulps zmm9, zmm22, zmm16
    vfmadd231ps zmm9, zmm23, zmm17
    vfmadd231ps zmm9, zmm24, zmm18           ; D.N (negative)
    vsubps zmm9, zmm29, zmm9
    vmaxps zmm9, zmm9, zmm29                 ; cosi = -D.N, kept in zmm9 for the glass code
    BC zmm10, kOne
    vsubps zmm10, zmm10, zmm9
    vmulps zmm11, zmm10, zmm10
    vmulps zmm11, zmm11, zmm10
    BC zmm12, kRim
    vmulps zmm11, zmm11, zmm12               ; rim intensity
    mov eax, NOBJ
    vpbroadcastd zmm12, eax
    vpcmpeqd k1, zmm31, zmm12
    vxorps zmm11{k1}, zmm11, zmm11           ; floor lanes: no rim
    BC zmm13, kRimAdd
    vaddps zmm2, zmm25, zmm13
    vmulps zmm2, zmm2, zmm11
    vaddps zmm2, zmm2, zmm8                  ; base R = rim + spec
    vaddps zmm3, zmm26, zmm13
    vmulps zmm3, zmm3, zmm11
    vaddps zmm3, zmm3, zmm8                  ; base G
    vaddps zmm4, zmm27, zmm13
    vmulps zmm4, zmm4, zmm11
    vaddps zmm4, zmm4, zmm8                  ; base B
    vmovaps zmm10, zmm2                      ; keep copies of the base for glass
    vmovaps zmm11, zmm3
    vmovaps zmm12, zmm4

    ; ---- opaque local colour = base + albedo*(ambient + toon diffuse) ----
    BC zmm5, kDiffI
    vmulps zmm5, zmm1, zmm5
    BC zmm6, kAmbR
    vaddps zmm6, zmm6, zmm5
    vfmadd231ps zmm2, zmm25, zmm6
    BC zmm6, kAmbG
    vaddps zmm6, zmm6, zmm5
    vfmadd231ps zmm3, zmm26, zmm6
    BC zmm6, kAmbB
    vaddps zmm6, zmm6, zmm5
    vfmadd231ps zmm4, zmm27, zmm6
    kmovw k1, r11d                           ; emissive neon grid lines on the floor
    vbroadcastss zmm2{k1}, dword ptr [lineR]
    vbroadcastss zmm3{k1}, dword ptr [lineG]
    vbroadcastss zmm4{k1}, dword ptr [lineB]

    ; ---- glass: Snell + Fresnel (computed for every lane, applied to glass lanes only) ----
    kmovw k1, r10d                           ; k1 = inside
    BC zmm6, kEtaOut
    vbroadcastss zmm6{k1}, dword ptr [kEtaIn]        ; eta = n1/n2
    vmulps zmm7, zmm9, zmm9
    BC zmm8, kOne
    vsubps zmm7, zmm8, zmm7                  ; sin^2(incident) = 1 - cos^2
    vmulps zmm13, zmm6, zmm6
    vmulps zmm13, zmm13, zmm7
    vsubps zmm13, zmm8, zmm13                ; k = 1 - eta^2 sin^2
    vcmpps k2, zmm13, zmm29, 1               ; k < 0 : total internal reflection
    vmaxps zmm13, zmm13, zmm29
    vsqrtps zmm13, zmm13                     ; sqrt(k) = cos(transmitted), kept in zmm13
    vmovaps zmm7, zmm9
    vmovaps zmm7{k1}, zmm13                  ; Schlick angle: transmitted when inside
    vsubps zmm7, zmm8, zmm7
    vmulps zmm14, zmm7, zmm7
    vmulps zmm14, zmm14, zmm14
    vmulps zmm14, zmm14, zmm7                ; (1-cos)^5
    BC zmm15, kF0
    BC zmm7, kOneMinusF0
    vfmadd231ps zmm15, zmm7, zmm14           ; F, kept in zmm15
    vmovaps zmm14, zmm15
    vxorps zmm14{k1}, zmm14, zmm14           ; reflected fraction seen from outside only
    ; environment reflected in the surface: R = D + 2*cosi*N
    vaddps zmm0, zmm9, zmm9
    vmovaps zmm1, zmm22
    vfmadd231ps zmm1, zmm0, zmm16
    vmovaps zmm5, zmm23
    vfmadd231ps zmm5, zmm0, zmm17
    vmovaps zmm7, zmm24
    vfmadd231ps zmm7, zmm0, zmm18
    SKY zmm19, zmm20, zmm21, zmm1, zmm5, zmm7, zmm0, zmm8
    vfmadd231ps zmm10, zmm14, zmm19          ; glass colour = base + Fenv * environment
    vfmadd231ps zmm11, zmm14, zmm20
    vfmadd231ps zmm12, zmm14, zmm21
    kmovw k4, r9d
    vmovaps zmm2{k4}, zmm10
    vmovaps zmm3{k4}, zmm11
    vmovaps zmm4{k4}, zmm12

    ; ---- distance fog toward the sky colour (solid lanes only): f = min(t*k, 1)^2 ----
    BC zmm0, kFogK
    vmulps zmm0, zmm0, zmm30
    BC zmm1, kOne
    vminps zmm0, zmm0, zmm1
    vmulps zmm0, zmm0, zmm0
    SKY zmm19, zmm20, zmm21, zmm22, zmm23, zmm24, zmm1, zmm5
    kmovw k4, r9d
    kandnw k3, k4, k6                        ; hit and not glass
    vsubps zmm19, zmm19, zmm2
    vfmadd231ps zmm2{k3}, zmm19, zmm0
    vsubps zmm20, zmm20, zmm3
    vfmadd231ps zmm3{k3}, zmm20, zmm0
    vsubps zmm21, zmm21, zmm4
    vfmadd231ps zmm4{k3}, zmm21, zmm0

    cmp ebx, MAXD-1
    jb nl0
    vxorps zmm28, zmm28, zmm28               ; last bounce: no reflection, full local colour
nl0:
    ; ---- accumulate: acc += atten * (1 - refl) * local ----
    BC zmm1, kOne
    vsubps zmm1, zmm1, zmm28
    VLD zmm5, s_tr
    vmulps zmm5, zmm5, zmm1
    VLD zmm7, s_ar
    vfmadd231ps zmm7{k6}, zmm5, zmm2
    VST s_ar, zmm7
    VLD zmm5, s_tg
    vmulps zmm5, zmm5, zmm1
    VLD zmm7, s_ag
    vfmadd231ps zmm7{k6}, zmm5, zmm3
    VST s_ag, zmm7
    VLD zmm5, s_tb
    vmulps zmm5, zmm5, zmm1
    VLD zmm7, s_ab
    vfmadd231ps zmm7{k6}, zmm5, zmm4
    VST s_ab, zmm7

    ; ---- what the continuing ray carries: mirror = refl; glass in = (1-F)*tint; glass out = 1 ----
    vmovaps zmm10, zmm28
    vmovaps zmm11, zmm28
    vmovaps zmm12, zmm28
    BC zmm8, kOne
    vsubps zmm8, zmm8, zmm15                 ; 1 - F
    kmovw k4, r9d
    kmovw k1, r10d
    vmulps zmm10{k4}, zmm8, zmm25
    vmulps zmm11{k4}, zmm8, zmm26
    vmulps zmm12{k4}, zmm8, zmm27
    kandw k3, k4, k1                         ; glass lanes that are inside
    BC zmm0, kOne
    vmovaps zmm10{k3}, zmm0
    vmovaps zmm11{k3}, zmm0
    vmovaps zmm12{k3}, zmm0
    cmp ebx, MAXD-1
    jb notlast
    vxorps zmm10, zmm10, zmm10               ; last bounce: nothing continues
    vxorps zmm11, zmm11, zmm11
    vxorps zmm12, zmm12, zmm12
notlast:
    VLD zmm5, s_tr
    vmulps zmm5{k6}, zmm5, zmm10
    VST s_tr, zmm5
    VLD zmm7, s_tg
    vmulps zmm7{k6}, zmm7, zmm11
    VST s_tg, zmm7
    VLD zmm14, s_tb
    vmulps zmm14{k6}, zmm14, zmm12
    VST s_tb, zmm14
    vmaxps zmm0, zmm5, zmm7
    vmaxps zmm0, zmm0, zmm14
    BC zmm8, kMinAtten
    vcmpps k7{k6}, zmm0, zmm8, 13            ; alive = hit AND some channel >= 1%
    kortestw k7, k7
    jz finish

    ; ---- next ray: mirror/TIR = reflect, glass = refract (T = eta*D + (eta*cosi - sqrt(k))*N) ----
    vaddps zmm0, zmm9, zmm9
    vmovaps zmm1, zmm22
    vfmadd231ps zmm1, zmm0, zmm16            ; reflected direction
    vmovaps zmm5, zmm23
    vfmadd231ps zmm5, zmm0, zmm17
    vmovaps zmm7, zmm24
    vfmadd231ps zmm7, zmm0, zmm18
    vmulps zmm8, zmm6, zmm9
    vsubps zmm8, zmm8, zmm13                 ; eta*cosi - sqrt(k)
    vmulps zmm10, zmm6, zmm22
    vfmadd231ps zmm10, zmm8, zmm16
    vmulps zmm11, zmm6, zmm23
    vfmadd231ps zmm11, zmm8, zmm17
    vmulps zmm12, zmm6, zmm24
    vfmadd231ps zmm12, zmm8, zmm18           ; refracted direction
    kmovw k4, r9d
    kandnw k3, k2, k4                        ; glass lanes without total internal reflection
    vmovaps zmm1{k3}, zmm10
    vmovaps zmm5{k3}, zmm11
    vmovaps zmm7{k3}, zmm12
    VST s_dx, zmm1
    VST s_dy, zmm5
    VST s_dz, zmm7
    BC zmm8, kEps2
    vbroadcastss zmm8{k3}, dword ptr [kNegEps2]      ; refracted rays start just behind the surface
    VLD zmm0, s_px
    vfmadd231ps zmm0, zmm16, zmm8
    VST s_ox, zmm0
    VLD zmm0, s_py
    vfmadd231ps zmm0, zmm17, zmm8
    VST s_oy, zmm0
    VLD zmm0, s_pz
    vfmadd231ps zmm0, zmm18, zmm8
    VST s_oz, zmm0
    inc ebx
    cmp ebx, MAXD
    jb bounce
finish:
    ret
trace_packet endp

; -----------------------------------------------------------------------------
; render_thread: pulls work units (16x4 pixels = 4 tiles) until none are left.
; -----------------------------------------------------------------------------
render_thread proc
    push rbx
    push rbp
    push rsi
    push rdi
    push r12
    push r13
    push r14
    push r15
    mov r15, rsp
    sub rsp, FRAME_SZ+64
    and rsp, -64
    mov rbp, rsp                             ; 64-byte aligned frame

    vmovaps xmmword ptr [rbp+s_xmm+0],   xmm6    ; xmm6-15 are callee-saved on Win64
    vmovaps xmmword ptr [rbp+s_xmm+16],  xmm7
    vmovaps xmmword ptr [rbp+s_xmm+32],  xmm8
    vmovaps xmmword ptr [rbp+s_xmm+48],  xmm9
    vmovaps xmmword ptr [rbp+s_xmm+64],  xmm10
    vmovaps xmmword ptr [rbp+s_xmm+80],  xmm11
    vmovaps xmmword ptr [rbp+s_xmm+96],  xmm12
    vmovaps xmmword ptr [rbp+s_xmm+112], xmm13
    vmovaps xmmword ptr [rbp+s_xmm+128], xmm14
    vmovaps xmmword ptr [rbp+s_xmm+144], xmm15
    vstmxcsr dword ptr [rbp+s_mx0]
    mov dword ptr [rbp+s_mx1], 9FC0h         ; + flush-to-zero, denormals-are-zero
    vldmxcsr dword ptr [rbp+s_mx1]

unitloop:
    mov eax, 1
    lock xadd dword ptr [unitCounter], eax   ; eax = my unit number
    cmp eax, NUNITS
    jae threaddone
    xor edx, edx
    mov ecx, UNITS_X
    div ecx                                  ; eax = tile row, edx = unit column
    mov r13d, eax                            ; ty
    lea r12d, [rdx*4]                        ; tx of the first tile
    mov edi, 4                               ; 4 tiles per unit

tileloop:
    vxorps zmm0, zmm0, zmm0
    VST s_sr, zmm0
    VST s_sg, zmm0
    VST s_sb, zmm0
    xor r14d, r14d                           ; sample index 0..3
sampleloop:
    lea rax, [jit]
    vbroadcastss zmm10, dword ptr [rax+r14*8]      ; sx
    vbroadcastss zmm11, dword ptr [rax+r14*8+4]    ; sy
    ; dx = (tx*4 + laneX + sx)*pix - halfW
    lea eax, [r12*4]
    vcvtsi2ss xmm0, xmm0, eax
    vbroadcastss zmm0, xmm0
    vmovups zmm1, zmmword ptr [laneX]
    vaddps zmm0, zmm0, zmm1
    vaddps zmm0, zmm0, zmm10
    BC zmm2, fPix
    vmulps zmm0, zmm0, zmm2
    BC zmm3, fHalfW
    vsubps zmm0, zmm0, zmm3
    ; dy = halfH - (ty*4 + laneY + sy)*pix
    lea eax, [r13*4]
    vcvtsi2ss xmm1, xmm1, eax
    vbroadcastss zmm1, xmm1
    vmovups zmm4, zmmword ptr [laneY]
    vaddps zmm1, zmm1, zmm4
    vaddps zmm1, zmm1, zmm11
    vmulps zmm1, zmm1, zmm2
    BC zmm4, kHalf
    vsubps zmm4, zmm4, zmm1                  ; py (zmm4), px is in zmm0
    ; ray = F + px*R + py*U  (camera basis, broadcast)
    BC zmm6, camFx
    BC zmm5, camRx
    vfmadd231ps zmm6, zmm0, zmm5
    BC zmm5, camUx
    vfmadd231ps zmm6, zmm4, zmm5
    BC zmm7, camFy
    BC zmm5, camRy
    vfmadd231ps zmm7, zmm0, zmm5
    BC zmm5, camUy
    vfmadd231ps zmm7, zmm4, zmm5
    BC zmm8, camFz
    BC zmm5, camRz
    vfmadd231ps zmm8, zmm0, zmm5
    BC zmm5, camUz
    vfmadd231ps zmm8, zmm4, zmm5
    ; normalise
    vmulps zmm9, zmm6, zmm6
    vfmadd231ps zmm9, zmm7, zmm7
    vfmadd231ps zmm9, zmm8, zmm8
    RSQRT zmm12, zmm9, zmm13, zmm14
    vmulps zmm6, zmm6, zmm12
    vmulps zmm7, zmm7, zmm12
    vmulps zmm8, zmm8, zmm12
    VST s_dx, zmm6
    VST s_dy, zmm7
    VST s_dz, zmm8
    BC zmm0, camX
    VST s_ox, zmm0
    BC zmm0, camY
    VST s_oy, zmm0
    BC zmm0, camZ
    VST s_oz, zmm0
    call trace_packet
    VLD zmm0, s_sr
    VLD zmm1, s_ar
    vaddps zmm0, zmm0, zmm1
    VST s_sr, zmm0
    VLD zmm0, s_sg
    VLD zmm1, s_ag
    vaddps zmm0, zmm0, zmm1
    VST s_sg, zmm0
    VLD zmm0, s_sb
    VLD zmm1, s_ab
    vaddps zmm0, zmm0, zmm1
    VST s_sb, zmm0
    inc r14d
    cmp r14d, 4
    jb sampleloop

    ; ---- average, gamma, pack to BGRA ----
    BC zmm4, kQuarter
    BC zmm5, kOne
    vxorps zmm6, zmm6, zmm6
    BC zmm7, k255
    PACKCH zmm0, s_sr
    PACKCH zmm1, s_sg
    PACKCH zmm2, s_sb
    vpslld zmm0, zmm0, 16                    ; R -> bits 16-23
    vpslld zmm1, zmm1, 8                     ; G -> bits 8-15
    vpord zmm0, zmm0, zmm1
    vpord zmm0, zmm0, zmm2                   ; B in bits 0-7
    mov eax, 0FF000000h
    vpbroadcastd zmm3, eax
    vpord zmm0, zmm0, zmm3                   ; alpha = 255
    ; framebuffer address of the tile's top-left pixel
    mov eax, r13d
    shl eax, 2
    imul eax, eax, IMG_W
    lea ecx, [r12*4]
    add eax, ecx
    mov rdx, qword ptr [fbPtr]
    lea rdx, [rdx+rax*4]
    vmovdqu xmmword ptr [rdx], xmm0                      ; tile row 0
    vextracti32x4 xmm1, zmm0, 1
    vmovdqu xmmword ptr [rdx+IMG_W*4], xmm1              ; row 1
    vextracti32x4 xmm1, zmm0, 2
    vmovdqu xmmword ptr [rdx+IMG_W*8], xmm1              ; row 2
    vextracti32x4 xmm1, zmm0, 3
    vmovdqu xmmword ptr [rdx+IMG_W*12], xmm1             ; row 3
    inc r12d
    dec edi
    jnz tileloop
    jmp unitloop

threaddone:
    vldmxcsr dword ptr [rbp+s_mx0]
    vmovaps xmm6,  xmmword ptr [rbp+s_xmm+0]
    vmovaps xmm7,  xmmword ptr [rbp+s_xmm+16]
    vmovaps xmm8,  xmmword ptr [rbp+s_xmm+32]
    vmovaps xmm9,  xmmword ptr [rbp+s_xmm+48]
    vmovaps xmm10, xmmword ptr [rbp+s_xmm+64]
    vmovaps xmm11, xmmword ptr [rbp+s_xmm+80]
    vmovaps xmm12, xmmword ptr [rbp+s_xmm+96]
    vmovaps xmm13, xmmword ptr [rbp+s_xmm+112]
    vmovaps xmm14, xmmword ptr [rbp+s_xmm+128]
    vmovaps xmm15, xmmword ptr [rbp+s_xmm+144]
    vzeroupper
    mov rsp, r15
    pop r15
    pop r14
    pop r13
    pop r12
    pop rdi
    pop rsi
    pop rbp
    pop rbx
    xor eax, eax
    ret
render_thread endp

; ---- tiny text helpers (rdi = output cursor) ---------------------------------
append_str proc                              ; rsi = zero-terminated string
as1:
    mov al, byte ptr [rsi]
    test al, al
    jz as2
    mov byte ptr [rdi], al
    inc rsi
    inc rdi
    jmp as1
as2:
    ret
append_str endp

append_u proc                                ; rax = unsigned value, appended in decimal
    lea r9, [numtmp+24]
    mov r8, r9
    mov ecx, 10
au1:
    xor edx, edx
    div rcx
    add dl, '0'
    dec r8
    mov byte ptr [r8], dl
    test rax, rax
    jnz au1
au2:
    mov al, byte ptr [r8]
    mov byte ptr [rdi], al
    inc rdi
    inc r8
    cmp r8, r9
    jb au2
    ret
append_u endp

append_ms proc                               ; rax = microseconds -> "ms.xx"
    xor edx, edx
    mov ecx, 1000
    div rcx                                  ; rax = ms, rdx = remaining us
    mov r10, rdx
    call append_u
    mov byte ptr [rdi], '.'
    inc rdi
    mov rax, r10
    xor edx, edx
    mov ecx, 10
    div rcx                                  ; rax = hundredths (0-99)
    xor edx, edx
    div rcx                                  ; rax = tens digit, rdx = units digit
    add al, '0'
    mov byte ptr [rdi], al
    add dl, '0'
    mov byte ptr [rdi+1], dl
    add rdi, 2
    ret
append_ms endp

; -----------------------------------------------------------------------------
; WndProc: window procedure. Esc closes, the cursor is hidden over the client area.
; -----------------------------------------------------------------------------
WndProc proc
    sub rsp, 28h
    cmp edx, 2                               ; WM_DESTROY
    je wp_destroy
    cmp edx, 100h                            ; WM_KEYDOWN
    je wp_key
    cmp edx, 20h                             ; WM_SETCURSOR
    je wp_cursor
    cmp edx, 14h                             ; WM_ERASEBKGND (we repaint everything)
    je wp_erase
wp_def:
    add rsp, 28h
    jmp DefWindowProcA                       ; tail call with the original arguments
wp_destroy:
    xor ecx, ecx
    call PostQuitMessage
    xor eax, eax
    add rsp, 28h
    ret
wp_key:
    cmp r8d, 1Bh                             ; VK_ESCAPE
    jne wp_def
    call DestroyWindow                       ; rcx still = hwnd
    xor eax, eax
    add rsp, 28h
    ret
wp_cursor:
    cmp r9w, 1                               ; HTCLIENT
    jne wp_def
    xor ecx, ecx
    call SetCursor                           ; NULL cursor = hidden
    mov eax, 1
    add rsp, 28h
    ret
wp_erase:
    mov eax, 1
    add rsp, 28h
    ret
WndProc endp

; -----------------------------------------------------------------------------
; update_camera: mouse look + WASD/QE/Shift, then rebuilds the camera basis.
;   Uses dtf (frame time in seconds). Only reacts while our window has focus.
; -----------------------------------------------------------------------------
update_camera proc
    push rsi
    push rdi
    push r12
    sub rsp, 20h
    xor esi, esi                             ; forward  (+1 W, -1 S)
    xor edi, edi                             ; strafe   (+1 D, -1 A)
    xor r12d, r12d                           ; vertical (+1 E, -1 Q)
    mov byte ptr [shiftFlag], 0
    call GetForegroundWindow
    cmp rax, qword ptr [hwnd]
    jne uc_nofocus

    ; ---- mouse look: delta from the window centre, then re-centre ----
    mov rcx, qword ptr [hwnd]
    lea rdx, [rectb]
    call GetClientRect
    mov eax, dword ptr [rectb+8]
    shr eax, 1
    mov dword ptr [ctrPt], eax
    mov eax, dword ptr [rectb+12]
    shr eax, 1
    mov dword ptr [ctrPt+4], eax
    mov rcx, qword ptr [hwnd]
    lea rdx, [ctrPt]
    call ClientToScreen
    lea rcx, [curPt]
    call GetCursorPos
    cmp byte ptr [skipMouse], 0
    jne uc_mousedone                         ; first frame after focus: no jump
    mov eax, dword ptr [curPt]
    sub eax, dword ptr [ctrPt]
    vcvtsi2ss xmm0, xmm0, eax
    vmulss xmm0, xmm0, dword ptr [kSens]
    vaddss xmm0, xmm0, dword ptr [yaw]
    vmovss dword ptr [yaw], xmm0             ; mouse right -> turn right
    mov eax, dword ptr [curPt+4]
    sub eax, dword ptr [ctrPt+4]
    vcvtsi2ss xmm0, xmm0, eax
    vmulss xmm0, xmm0, dword ptr [kSens]
    vmovss xmm1, dword ptr [pitch]
    vsubss xmm1, xmm1, xmm0                  ; mouse down -> look down
    vminss xmm1, xmm1, dword ptr [kPitchMax]
    vmaxss xmm1, xmm1, dword ptr [kPitchMin]
    vmovss dword ptr [pitch], xmm1
uc_mousedone:
    mov byte ptr [skipMouse], 0
    mov ecx, dword ptr [ctrPt]
    mov edx, dword ptr [ctrPt+4]
    call SetCursorPos

    ; ---- keyboard (high bit of GetAsyncKeyState = currently down) ----
    mov ecx, 'W'
    call GetAsyncKeyState
    test ax, ax
    jns @F
    inc esi
@@:
    mov ecx, 'S'
    call GetAsyncKeyState
    test ax, ax
    jns @F
    dec esi
@@:
    mov ecx, 'D'
    call GetAsyncKeyState
    test ax, ax
    jns @F
    inc edi
@@:
    mov ecx, 'A'
    call GetAsyncKeyState
    test ax, ax
    jns @F
    dec edi
@@:
    mov ecx, 'E'
    call GetAsyncKeyState
    test ax, ax
    jns @F
    inc r12d
@@:
    mov ecx, 'Q'
    call GetAsyncKeyState
    test ax, ax
    jns @F
    dec r12d
@@:
    mov ecx, 10h                             ; VK_SHIFT
    call GetAsyncKeyState
    test ax, ax
    jns uc_basis
    mov byte ptr [shiftFlag], 1
    jmp uc_basis
uc_nofocus:
    mov byte ptr [skipMouse], 1              ; swallow the first mouse delta on return

    ; ---- camera basis from yaw/pitch (x87 does sin/cos) ----
    ;   F = (cp*sy, sp, cp*cy)   R = (cy, 0, -sy)   U = (-sp*sy, cp, -sp*cy)
uc_basis:
    fld dword ptr [yaw]
    fsincos
    fstp dword ptr [cosY]
    fstp dword ptr [sinY]
    fld dword ptr [pitch]
    fsincos
    fstp dword ptr [cosP]
    fstp dword ptr [sinP]
    vmovss xmm0, dword ptr [cosP]
    vmovss xmm1, dword ptr [sinP]
    vmovss xmm2, dword ptr [cosY]
    vmovss xmm3, dword ptr [sinY]
    vxorps xmm5, xmm5, xmm5
    vmulss xmm4, xmm0, xmm3
    vmovss dword ptr [camFx], xmm4
    vmovss dword ptr [camFy], xmm1
    vmulss xmm4, xmm0, xmm2
    vmovss dword ptr [camFz], xmm4
    vmovss dword ptr [camRx], xmm2
    vmovss dword ptr [camRy], xmm5
    vsubss xmm4, xmm5, xmm3
    vmovss dword ptr [camRz], xmm4
    vmulss xmm4, xmm1, xmm3
    vsubss xmm4, xmm5, xmm4
    vmovss dword ptr [camUx], xmm4
    vmovss dword ptr [camUy], xmm0
    vmulss xmm4, xmm1, xmm2
    vsubss xmm4, xmm5, xmm4
    vmovss dword ptr [camUz], xmm4

    ; ---- move: pos += (F*fwd + R*strafe + up*vert) * speed * dt ----
    vmovss xmm6, dword ptr [dtf]
    vmulss xmm6, xmm6, dword ptr [kSpeed]
    cmp byte ptr [shiftFlag], 0
    je @F
    vmulss xmm6, xmm6, dword ptr [kShift]
@@:
    vcvtsi2ss xmm7, xmm7, esi
    vmulss xmm7, xmm7, xmm6                  ; forward distance
    vcvtsi2ss xmm8, xmm8, edi
    vmulss xmm8, xmm8, xmm6                  ; strafe distance
    vcvtsi2ss xmm9, xmm9, r12d
    vmulss xmm9, xmm9, xmm6                  ; vertical distance
    vmovss xmm0, dword ptr [camX]
    vfmadd231ss xmm0, xmm7, dword ptr [camFx]
    vfmadd231ss xmm0, xmm8, dword ptr [camRx]
    vmovss dword ptr [camX], xmm0
    vmovss xmm0, dword ptr [camY]
    vfmadd231ss xmm0, xmm7, dword ptr [camFy]
    vaddss xmm0, xmm0, xmm9
    vmaxss xmm0, xmm0, dword ptr [kMinY]     ; stay above the floor
    vmovss dword ptr [camY], xmm0
    vmovss xmm0, dword ptr [camZ]
    vfmadd231ss xmm0, xmm7, dword ptr [camFz]
    vfmadd231ss xmm0, xmm8, dword ptr [camRz]
    vmovss dword ptr [camZ], xmm0

    add rsp, 20h
    pop r12
    pop rdi
    pop rsi
    ret
update_camera endp

; -----------------------------------------------------------------------------
; render_frame: one full frame on nthr worker threads (blocks until finished).
; -----------------------------------------------------------------------------
render_frame proc
    push rbx
    push rsi
    sub rsp, 38h
    mov dword ptr [unitCounter], 0
    mov esi, dword ptr [nthr]
    xor ebx, ebx
rf_spawn:
    xor ecx, ecx
    xor edx, edx
    lea r8, [render_thread]
    xor r9d, r9d
    mov qword ptr [rsp+32], 0
    mov qword ptr [rsp+40], 0
    call CreateThread
    lea rdx, [handles]
    mov qword ptr [rdx+rbx*8], rax
    inc ebx
    cmp ebx, esi
    jb rf_spawn
    mov ecx, esi
    lea rdx, [handles]
    mov r8d, 1
    mov r9d, 0FFFFFFFFh
    call WaitForMultipleObjects
    xor ebx, ebx
rf_close:
    lea rdx, [handles]
    mov rcx, qword ptr [rdx+rbx*8]
    call CloseHandle
    inc ebx
    cmp ebx, esi
    jb rf_close
    add rsp, 38h
    pop rsi
    pop rbx
    ret
render_frame endp

; -----------------------------------------------------------------------------
main proc
    sub rsp, 78h                             ; 13-arg StretchDIBits needs 9 stack args

    ; pixel-size constants from IMG_W / IMG_H
    mov eax, IMG_H
    vcvtsi2ss xmm0, xmm0, eax
    vmovss xmm1, dword ptr [kOne]
    vdivss xmm1, xmm1, xmm0
    vmovss dword ptr [fPix], xmm1            ; 1/IMG_H
    mov eax, IMG_W
    vcvtsi2ss xmm2, xmm2, eax
    vmulss xmm2, xmm2, xmm1
    vmulss xmm2, xmm2, dword ptr [kHalf]
    vmovss dword ptr [fHalfW], xmm2          ; IMG_W/IMG_H * 0.5

    xor ecx, ecx
    mov edx, IMG_W*IMG_H*4
    mov r8d, 3000h
    mov r9d, 4
    call VirtualAlloc
    mov qword ptr [fbPtr], rax

    lea rcx, [qpf]
    call QueryPerformanceFrequency
    lea rcx, [sysinfo]
    call GetSystemInfo
    mov edi, dword ptr [sysinfo+32]          ; logical processors
    mov dword ptr [lpiLen], 4096
    lea rcx, [lpiBuf]
    lea rdx, [lpiLen]
    call GetLogicalProcessorInformation
    test eax, eax
    jz throk
    xor ecx, ecx
    lea rdx, [lpiBuf]
    mov r8d, dword ptr [lpiLen]
    add r8, rdx
cntcores:
    cmp rdx, r8
    jae cntdone
    cmp dword ptr [rdx+8], 0                 ; RelationProcessorCore
    jne cntnext
    inc ecx
cntnext:
    add rdx, 32
    jmp cntcores
cntdone:
    test ecx, ecx
    jz throk
    mov edi, ecx                             ; physical cores
throk:
    cmp edi, 64
    jbe throk2
    mov edi, 64
throk2:
    test edi, edi
    jnz throk3
    mov edi, 1
throk3:
    mov dword ptr [nthr], edi
    mov byte ptr [skipMouse], 1

    ; ---- create the window ----
    xor ecx, ecx
    call GetModuleHandleA
    mov qword ptr [hinst], rax
    lea rdx, [wc]
    mov dword ptr [rdx], 80                  ; WNDCLASSEX.cbSize
    lea rax, [WndProc]
    mov qword ptr [rdx+8], rax               ; lpfnWndProc
    mov rax, qword ptr [hinst]
    mov qword ptr [rdx+24], rax              ; hInstance
    lea rax, [clsName]
    mov qword ptr [rdx+64], rax              ; lpszClassName
    xor ecx, ecx
    mov edx, 32512                           ; IDC_ARROW
    call LoadCursorA
    lea rdx, [wc]
    mov qword ptr [rdx+40], rax              ; hCursor
    lea rcx, [wc]
    call RegisterClassExA
    mov dword ptr [rectb], 0
    mov dword ptr [rectb+4], 0
    mov dword ptr [rectb+8], IMG_W
    mov dword ptr [rectb+12], IMG_H
    lea rcx, [rectb]
    mov edx, 0CA0000h                        ; caption | sysmenu | minimize, fixed size
    xor r8d, r8d
    call AdjustWindowRect                    ; client size -> outer window size
    mov eax, dword ptr [rectb+8]
    sub eax, dword ptr [rectb]
    mov ecx, dword ptr [rectb+12]
    sub ecx, dword ptr [rectb+4]
    mov dword ptr [rsp+32], 80000000h        ; CW_USEDEFAULT x
    mov dword ptr [rsp+40], 80000000h        ; CW_USEDEFAULT y
    mov dword ptr [rsp+48], eax              ; width
    mov dword ptr [rsp+56], ecx              ; height
    mov qword ptr [rsp+64], 0                ; parent
    mov qword ptr [rsp+72], 0                ; menu
    mov rax, qword ptr [hinst]
    mov qword ptr [rsp+80], rax
    mov qword ptr [rsp+88], 0
    xor ecx, ecx
    lea rdx, [clsName]
    lea r8, [titleInit]
    mov r9d, 10CA0000h                       ; WS_VISIBLE | the style above
    call CreateWindowExA
    mov qword ptr [hwnd], rax
    mov rcx, rax
    call GetDC
    mov qword ptr [hdc], rax
    lea rcx, [lastTicks]
    call QueryPerformanceCounter
    mov rax, qword ptr [lastTicks]
    mov qword ptr [lastTitle], rax

; ---- the infinite loop: messages, input, render, present ----
frameloop:
pump:
    lea rcx, [msgbuf]
    xor edx, edx
    xor r8d, r8d
    xor r9d, r9d
    mov qword ptr [rsp+32], 1                ; PM_REMOVE
    call PeekMessageA
    test eax, eax
    jz nomsg
    cmp dword ptr [msgbuf+8], 12h            ; WM_QUIT
    je quit
    lea rcx, [msgbuf]
    call DispatchMessageA
    jmp pump
nomsg:
    ; frame time in seconds (clamped so a stall cannot teleport the camera)
    lea rcx, [tnow]
    call QueryPerformanceCounter
    mov rax, qword ptr [tnow]
    mov rdx, rax
    sub rax, qword ptr [lastTicks]
    mov qword ptr [lastTicks], rdx
    vcvtsi2sd xmm0, xmm0, rax
    vcvtsi2sd xmm1, xmm1, qword ptr [qpf]
    vdivsd xmm0, xmm0, xmm1
    vcvtsd2ss xmm0, xmm0, xmm0
    vminss xmm0, xmm0, dword ptr [kMaxDt]
    vmovss dword ptr [dtf], xmm0

    call update_camera
    call render_frame

    ; present: StretchDIBits(hdc, 0,0,W,H, 0,0,W,H, bits, bmi, DIB_RGB_COLORS, SRCCOPY)
    mov rcx, qword ptr [hdc]
    xor edx, edx
    xor r8d, r8d
    mov r9d, IMG_W
    mov qword ptr [rsp+32], IMG_H
    mov qword ptr [rsp+40], 0
    mov qword ptr [rsp+48], 0
    mov qword ptr [rsp+56], IMG_W
    mov qword ptr [rsp+64], IMG_H
    mov rax, qword ptr [fbPtr]
    mov qword ptr [rsp+72], rax
    lea rax, [bmphdr+14]                     ; BITMAPINFOHEADER inside the BMP header
    mov qword ptr [rsp+80], rax
    mov qword ptr [rsp+88], 0                ; DIB_RGB_COLORS
    mov qword ptr [rsp+96], 0CC0020h         ; SRCCOPY
    call StretchDIBits

    ; ---- update the title twice a second with fps and ms/frame ----
    inc dword ptr [frames]
    lea rcx, [tnow]
    call QueryPerformanceCounter
    mov rax, qword ptr [tnow]
    sub rax, qword ptr [lastTitle]
    mov rcx, qword ptr [qpf]
    shr rcx, 1
    cmp rax, rcx
    jb frameloop
    imul rax, rax, 1000000
    xor edx, edx
    div qword ptr [qpf]
    mov qword ptr [tmpUs], rax               ; elapsed microseconds
    mov eax, dword ptr [frames]
    mov qword ptr [tmpFrames], rax
    mov dword ptr [frames], 0
    mov rax, qword ptr [tnow]
    mov qword ptr [lastTitle], rax
    lea rdi, [outbuf]
    lea rsi, [titleA]
    call append_str
    mov rax, qword ptr [tmpFrames]
    imul rax, rax, 1000000
    xor edx, edx
    div qword ptr [tmpUs]                    ; fps = frames * 1e6 / us
    call append_u
    lea rsi, [titleB]
    call append_str
    mov rax, qword ptr [tmpUs]
    xor edx, edx
    div qword ptr [tmpFrames]                ; microseconds per frame
    call append_ms
    lea rsi, [titleC]
    call append_str
    mov byte ptr [rdi], 0                    ; NUL-terminate for SetWindowTextA
    mov rcx, qword ptr [hwnd]
    lea rdx, [outbuf]
    call SetWindowTextA
    jmp frameloop

quit:
    xor ecx, ecx
    call ExitProcess
main endp
end
