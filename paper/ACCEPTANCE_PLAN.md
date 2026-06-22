# Plan de aceptación — OA-MHE (target: ISA Transactions, Q1, sim-only)

## ⭐ DECISIÓN (esta sesión) — foco para maximizar Q1
**DROPEAR `τ_rc`.** Solo dio problemas: sin ground truth (no validable → flanco de
revisor), no reproducible, constante "efectiva" excitación-dependiente, canal aparte
que diluye. Sacarlo **revierte el MHE al 10D `[p,v,m,d]` limpio y probado** (el del −78%).

**Foco del paper = estimación de FUERZA externa observability-aware:**
- Estrella: `d` (sensor virtual) + rechazo.
- Masa: SOCIA de la observabilidad (Prop 1: ambigüedad m–d_z dicta cuándo `d` es
  observable → switching de régimen). NO "identificamos masa" como headline.
- `τ_rc` → future work / paper #2 (con HW + excitación controlada).

Estado: draft limpio en forma/teoría (notación, lint, Prop 1). Falta **rigor
experimental + posicionamiento** para Q1.

---

## A. RIGOR EXPERIMENTAL — los bloqueantes duros

### A1. Estadística N≥5 + media±std  ⭐ #1
- Correr **≥5 repeticiones por condición** (semillas de ruido distintas) para:
  identificación (masa, τ_rc), sensor de fuerza (corr), rechazo (RMSE).
- Reportar **media ± std** en todas las tablas, no corridas únicas.
- Costo: bajo (sim), días. **Sin esto no entra a ningún Q1.**

### A2. Baseline de comparación  ⭐ #2
- Implementar al menos UNO: **EKF aumentado** (estado+masa+fuerza) o
  **momentum-observer clásico**.
- Comparar: error de masa, corr de fuerza, RMSE de rechazo, costo por paso.
- Mostrar que el MHE observability-aware **gana o iguala con menos**.
- Costo: medio (implementar el baseline). Es lo que responde "¿vs qué?".

### A3. Gráfica de error de seguimiento de trayectoria  ⭐ (pedido)
- **RMSE de posición vs tiempo** (o boxplot por condición), **con y sin feedforward**.
- Cuantifica la performance de control (hoy solo está la traj 3D cualitativa).
- Datos: ya en los CSV (px..pz vs px_ref..pz_ref). Costo: bajo.

---

## B. CONTENIDO / FIGURAS

### B1. Figura de observabilidad experimental (la que falta)  ⭐
- **Masa d-libre, Prop 1**: 2 seeds de masa (0.6 y 1.5) →
  en **hover** se quedan separados (NO observable, ambigüedad m–d_z) →
  en **trayectoria** colapsan a 1.05 (observable).
- Es la frontera de observabilidad GENUINA (no la efectiva de τ_rc).
- Llena el hueco: la Sección III hoy es teoría sola (Prop 1/2) sin evidencia directa.
- Requiere: sim FRESCO (reproducibilidad) + ~6 runs. Costo: medio.

### B2. (opcional) Sensibilidad al error de masa
- Barrer Δm (5/10/17/25%) → RMSE vs Δm con/sin corrección. Muestra margen de robustez.

---

## C. POSICIONAMIENTO / ESCRITURA

### C1. Afilar el delta vs SOTA  ⭐ #3
- El intro debe **gritar** la novedad: NO es "otro MHE de fuerza" (NeuroMHE ya existe).
- Delta = **selección de régimen por OBSERVABILIDAD** (Prop 1 → bloqueo d para ID masa /
  libero d para medir fuerza) + ID conjunta masa+τ_rc en una sola máquina.
- Diferenciar explícito de: momentum observers (Tomic), NeuroMHE, EKF.

### C2. Honestidad de τ_rc
- Enmarcarlo como **constante efectiva** (excitación-graduada, satura ~32 ms, sin GT),
  no como parámetro verdadero validado. Ya está fig:tausat; ajustar abstract/intro.

### C3. Abstract + keywords
- Keywords ISA (2–5). Abstract sin overclaim (ya corregido el 78%).

---

## D. SUBMISSION PREP

### D1. Convertir a `elsarticle` (ISA)  — `\documentclass[final,5p]{elsarticle}`
- Dos columnas. Porte de preámbulo; ecuaciones/figuras/tablas pasan casi tal cual.
- Verificar Guide for Authors (si exigen formato `review` 1-columna para arbitraje).

### D2. ORCID + cover letter. Revisión single-blind (no anonimato → autor OK).

---

## E. FUTURO (no bloquea ISA, fortalece / paper #2)

### E1. Hardware: indoor + mocap + perturbación calibrada (ventilador/tether con celda).
### E2. Paper #2: wrench 6-DOF (fuerza+torque) en plataforma con inercia mayor / tether.
   - El canal rotacional (d_ω) brilla donde los torques son grandes. Justificado
     empíricamente: en quad chico no es robusto (4 señales de fragilidad).

---

## ORDEN RECOMENDADO
1. **A1** (estadística) — el bloqueante más duro y barato.
2. **A3** (tracking error) + **A2** (baseline) — el "¿vs qué?" + performance.
3. **B1** (observabilidad masa d-libre) — llena el hueco teoría↔experimento.
4. **C1/C2** (afilar novedad + honestidad τ_rc).
5. **D1** (elsarticle) + **C3/D2** (abstract/keywords/ORCID).

Todo A+B+C+D es sim-only → factible sin hardware. E es el siguiente salto.
