# CLAUDE.md — nmpc_controller

Paper: **"Adaptive MPCC via Lie-Invariant MHE for Agile Quadrotor Flight"**
Target: IEEE Robotics and Automation Letters (RA-L)

---

## Arquitectura implementada

```
Sensores (MuJoCo / HW)
        │ y_k ∈ ℝ¹³ = [p, v, q, ω]
        ▼
┌──────────────────┐
│  MHE (N_e=20)    │  Lie-invariant arrival cost, SQP-RTI, 100 Hz
│  x_a ∈ ℝ²¹       │  → x̂_k (físico) + m̂, τ̂_rc, d̂ + σ_k = tr(P̄_θ)
└────────┬─────────┘
         │   EMA filter (α_m=0.98, α_d=0.92)
         │   Gate: solo inyecta si σ_k < 0.05
         ▼
┌──────────────────┐        ┌─────────────────┐
│  Adaptive W_s    │        │  Path γ(θ)       │
│  W_s(k) =        │        │  B-spline, N=400 │
│  W_s_max/(1+ασ_k)│        │  s_max = 100 m   │
└────────┬─────────┘        └────────┬────────┘
         │ W_s(k), m̂, k̂_τ, d̂         │ p_d, q_d, ω̂
         ▼                            ▼
┌──────────────────────────────────────────────┐
│  MPCC (N=50, T=1.5s non-uniform, SQP-RTI)   │
│  x ∈ ℝ¹⁶ = [p,v,q,ω,θ,v_θ,f]               │
│  u ∈ ℝ⁵  = [Δf, ω_cmd, a_θ]                 │
│  NP = 21 runtime params                      │
│    p[0:3]=Q_ec  p[3:6]=Q_el  p[6:9]=Q_q     │
│    p[9:13]=U    p[13]=W_s    p[14]=v_θ_max  │
│    p[15]=W_df   p[16]=m̂      p[17:20]=d̂     │
│    p[20]=k̂_τ=1/τ̂_rc                         │
└──────────────────┬───────────────────────────┘
                   │ f_k, ω_cmd
                   ▼
            Drone / MuJoCo
```

---

## Estado de implementación

### ✅ COMPLETADO

#### OCP generation
- `ocp_generation/generate_mhe_ocp.py` — MHE 21-state, NP=59, N=20, dt=0.01s
  - Lie-invariant arrival cost (quat_log, hemisphere correction)
  - Measurement residual en ℝ¹² (no ℝ¹³ — quaternion como Log 3D)
  - hpipm_mode="ROBUST", levenberg_marquardt=1e-2, PROJECT_REDUC_HESS
- `ocp_generation/generate_mpcc_ocp.py` — MPCC 16-state, NP=21, N=50, T=1.5s
  - m̂ en dv: `dv = f/m̂ · Re₃ + d̂`
  - d̂ en dv: perturbación feedforward
  - k̂_τ = 1/τ̂_rc en dω: `dω = k̂_τ(ω_cmd − ω)` — lineal en el parámetro
  - Nota: k_τ en vez de τ_rc evita Jacobianos O(1/τ²) que colapsan HPIPM

#### C++ wrappers
- `include/quadrotor_mpc/mhe/mhe_controller.hpp` + `src/mhe/mhe_controller.cpp`
  - NX=21, NU=5 (process noise), NP=59, N=20
  - sliding window: y_window_, u_window_ (deque)
  - propagate_prior(bool solver_ok): guarda x_bar_ solo si solver convergió y |d|<15
  - P_MIN_PHYS=1e-4, P_MIN_PARAM=1e-3 (evita P̄_inv → ∞ → MINSTEP)
  - sigma_k = tr(P̄_θ) — suma de varianzas de [m, τ, d]
- `include/quadrotor_mpc/mpcc/mpcc_controller.hpp` + `src/mpcc/mpcc_controller.cpp`
  - NP=21, MpccWeights: Q_ec, Q_el, Q_q, U_mat, Q_s, vtheta_max, W_df, m_hat, d_hat, k_tau
  - weights_to_param: p[20] = k_tau

#### SiL executables
- `src/mhe/mhe_mpcc_sil.cpp` — main loop MHE + adaptive W_s + MPCC
  - Modo: `./mhe_mpcc_sil 10 v1` (open-loop) o `./mhe_mpcc_sil 10 v2` (closed-loop)
  - v1: MPCC usa m_nom=0.9 kg fijo, d=0 — nunca corrige
  - v2: MHE → EMA → gate σ_k<0.05 → m̂, d̂, k̂_τ al MPCC
  - EMA: α_param=0.98 (τ≈0.5s), α_dist=0.92 (τ≈0.12s)
  - CSV extendido: 56 columnas incl. estados estimados vs reales
  - Resultados en `results/mhe_mpcc_v1_v10.csv` y `results/mhe_mpcc_v2_v10.csv`

#### Scripts
- `scripts/plot_mhe_mpcc_sil.py` — 13 figuras: trayectoria, MHE, timing, real vs estimado
- `scripts/plot_mhe_v1_v2_comparison.py` — 5 figuras comparación v1 vs v2

#### Paper
- `paper/paper.tex` — Secciones II–V completas y coherentes con el código
  - Título: "Adaptive MPCC via Lie-Invariant MHE for Agile Quadrotor Flight"
  - Modelo de medición: h(x_a) no lineal, residuo ℝ¹²
  - k_τ = 1/τ_rc documentado con justificación numérica
  - Algoritmo 1: EMA + gate σ_k < σ_thr explícitos
  - Tabla: σ_thr=0.05, α_EMA,m=0.98, α_EMA,d=0.92
  - Timing actualizado: MPCC 3.84ms, MHE 1.73ms, total ~5.6ms

---

## Resultados SiL actuales (v=10 m/s, m_nom=0.9 kg, m_true=1.08 kg)

| Métrica | v1 (open-loop) | v2 (closed-loop) | Mejora |
|---|---|---|---|
| RMSE posición | 0.308 m | 0.228 m | **−25.9%** |
| Error máximo | 0.670 m | 0.540 m | −19.4% |
| m̂ final | 0.900 kg (fija) | 1.080 kg (convergida) | ✓ |
| σ_k final | 5.0e-3 | 5.0e-3 | — |
| MPCC avg | 3.97 ms | 3.84 ms | — |
| MHE avg | 1.73 ms | 1.73 ms | — |
| Completado | ✓ | ✓ | — |

---

## ❌ PENDIENTE para RA-L

### Prioritario (sin esto el paper no llega a RA-L)

1. **Experimento con viento (disturbance rejection)**
   - Añadir fuerza externa constante en MuJoCo (≈10% del peso ≈1N en x)
   - Correr v1 vs v2 con viento — d̂ debe capturarlo, v2 debe ganar por mayor margen
   - Es el "killer experiment": demuestra el aporte real de d̂ en dinámica real

2. **Estudio de ablación completo** (4 configuraciones)
   - Config A: MPCC baseline (sin MHE, masa nominal correcta)
   - Config B: MHE+MPCC estado (sin param→MPCC, sin W_s adaptativo) 
   - Config C: v1 actual (MHE+W_s adaptativo, sin param→MPCC)
   - Config D: v2 actual (loop cerrado completo)
   - Aisla la contribución de cada componente independientemente

3. **Múltiples corridas** (N≥10 por configuración)
   - Reportar media ± desviación estándar del RMSE
   - RA-L no acepta resultados de una sola corrida como conclusión

4. **Hardware real** (crítico — RA-L casi siempre lo exige para papers de control)
   - Al menos 5 corridas en el dron físico
   - El SiL solo no es suficiente para RA-L en control de UAVs

### Secundario (fortalecen pero no bloquean)

5. **Sensibilidad al error de masa**
   - Barrer Δm = 5%, 10%, 17%, 25%
   - Graficar RMSE(v1) vs RMSE(v2) vs Δm → muestra margen de robustez

6. **Múltiples velocidades**
   - Correr ablación completa a v=5, 8, 10, 12 m/s
   - Tabla RMSE vs velocidad para cada configuración

7. **Abstract y keywords** — vacíos en el paper, hay que escribirlos

8. **Sección de Resultados** — no escrita todavía

---

## Comandos clave

```bash
# Build (desde build/)
cmake --build . --target mhe_mpcc_sil -j$(nproc)

# Correr experimentos
source /opt/ros/humble/setup.bash
./mhe_mpcc_sil 10 v1   → results/mhe_mpcc_v1_v10.csv
./mhe_mpcc_sil 10 v2   → results/mhe_mpcc_v2_v10.csv

# Plots individuales
python3 ../scripts/plot_mhe_mpcc_sil.py 10        → figures_mhe_mpcc_sil/v10/ (13 figs)

# Comparación v1 vs v2
python3 ../scripts/plot_mhe_v1_v2_comparison.py 10 → figures_mhe_comparison/v10/ (5 figs)

# Regenerar OCPs (solo si se cambia la formulación)
cd ocp_generation
python3 generate_mhe_ocp.py    → c_generated_code_mhe/
python3 generate_mpcc_ocp.py   → c_generated_code_mpcc/
cd ../build && cmake .. && cmake --build . -j$(nproc)

# Compilar paper
cd paper && pdflatex paper.tex
```

---

## Decisiones de diseño importantes

| Decisión | Razón |
|---|---|
| k_τ = 1/τ_rc como parámetro MPCC | τ_rc directo → Jacobiano O(1/τ²)≈1111 → colapsa HPIPM |
| Gate σ_k < 0.05 antes de inyectar θ̂ | Evita que MPCC use estimaciones no convergidas |
| EMA α=0.98/0.92 en salida MHE | Suaviza step-changes al deslizar ventana, sin sesgo |
| propagate_prior(solver_ok) guarda | Previene que x_bar_ se corrompa con soluciones fallidas |
| P_MIN_PARAM = 1e-3 | P̄_inv ≤ 1000 — evita QP stiff → ACADOS_MINSTEP |
| hpipm_mode="ROBUST" en MHE | Mayor regularización interna → previene MINSTEP en OCP regenerado |
| M_HAT_INIT = 0.9 kg (ambos modos) | Error deliberado 17% para demostrar convergencia y ventaja v2 |
| Residuo medición ℝ¹² (no ℝ¹³) | Quaternion entra como Log 3D — consistente con arrival cost Lie-inv. |
