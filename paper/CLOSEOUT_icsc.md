# CLOSEOUT — Observability paper → conference (ICSC 2026)

Decidido 2026-06-25. El paper de sensor virtual / observability es **conference-grade**
(SiL-only, novedad incremental para journal — ver memoria `venue-icuas`). Se cierra como
**congreso**, NO journal. Listo para cerrar en sesión FRESCA (cargar skill `ral-reviewer`).

## Venue
- **ICSC 2026** (IEEE Int. Conf. on Systems and Control) — deadline **30 sep 2026**. Control
  conference, SiL es norma, encaja la contribución de estimación. Portal: controls.papercept.net.
- **Backup / upgrade:** ACC 2027 (portal abre ~sep 2026, deadline ~oct) o ECC 2027 (~nov) —
  más prestigio, mismo trim sirve. NO rushear ICARCV (30 jun, 5 días, no vale).

## Estado actual del draft (`paper.tex`)
- `\documentclass[lettersize,journal]{IEEEtran}`, **11 páginas**, 13 figs, 5 tablas, 8 secciones.
- Abstract ya acortado/de-jergado (~190 palabras). Compila limpio. Lint ral-reviewer pasa.

## Trabajo para cerrar (1 sesión focalizada, sin rush)
1. **`\documentclass` → conference** (ieeeconf, o template ICSC). 
2. **Reframe a ESTIMACIÓN/OBSERVABILIDAD** (clave pa venue de control): liderar con la
   contribución de *observability-aware self-calibration + virtual force sensor*; el dron es
   el BANCO DE PRUEBAS, NO "para vuelo ágil". Un revisor de control valora el gate
   observability-aware + el sabor CRB; si se vende como "app de dron" lo ve flojo.
3. **Trim 11p → ~6-8p:**
   - CONSERVAR: sensor virtual de fuerza (medir), **gate observability-aware (lo novel)**,
     1 resultado de rechazo, la Proposición de observabilidad.
   - CORTAR: sweeps de robustez, tabla multi-velocidad, ablaciones de más, figuras redundantes.
4. **Lint final** (ral-reviewer, `.claude/prose-ledger.txt` ya existe) + compilar.

## Skills a cargar en la sesión fresca
- `ral-reviewer` (revisión RA-L/congreso + prose linter, plugin huao).

## Honesto
Prestigio bajo-medio (ICSC) acorde a un paper SiL que ya sabíamos era congreso. Es un output
REAL en el tablero, sin rush, hecho limpio. La energía journal NO va acá — va a exact_mpcc.
