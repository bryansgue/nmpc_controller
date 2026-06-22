# writer_tuto.md — Escribir un paper con el sistema `ral-reviewer`

> **Copiá este archivo a la raíz de tu repo de paper nuevo.** Después decile a la IA:
> *"Leé `writer_tuto.md` y dejá el proyecto listo para escribir."* Con eso sabe qué hacer.
>
> El sistema tiene 3 piezas: un **plugin** (skill `/ral-reviewer` + **hook** de linting),
> un **motor general** de escritura (vale para todo paper) y un **ledger por-proyecto**
> (la jerga puntual de ESTE paper). Lo de abajo te dice cómo alimentar cada cosa.

---

## ⚡ TL;DR — Checklist que la IA ejecuta al leer este archivo

1. [ ] Verificar que el plugin `ral-reviewer@huao` esté instalado (si no → §1).
2. [ ] Crear `<repo>/.claude/prose-ledger.txt` (jerga del proyecto). Entrevistar al usuario
       por su dominio y términos prohibidos (§3). Sin ledger igual funciona el motor general.
3. [ ] (Opcional pero recomendado) Armar el corpus SOTA en `paper/ref_corpus/` (§4) — habilita
       el `--sweep` y es la fuente de verdad para "¿esta palabra es jerga válida?".
4. [ ] Confirmar el modo de escritura: leer §5 (reglas que SIEMPRE se aplican) y la regla cardinal.
5. [ ] Empezar a escribir en `paper/` (.tex). El hook lintea cada edición; usar `/ral-reviewer`
       para las pasadas de criterio (§6).

---

## 0. Cómo funciona (mapa mental)

| Pieza | Dónde vive | Qué hace | Quién la edita |
|---|---|---|---|
| **Hook** (trigger) | plugin `hooks/hooks.json` | corre el linter en cada `Edit/Write` de `.tex` | (raro) |
| **Motor general** | plugin `lint_prose.py` | spelling US, gramática, em-dash, AI-tells — **todo paper** | vía push al plugin |
| **Ledger de proyecto** | `<repo>/.claude/prose-ledger.txt` | términos coloquiales de ESTE paper | **vos, local** |
| **Corpus SOTA** | `<repo>/paper/ref_corpus/` | papers de referencia para validar jerga (`--sweep`) | vos, local |
| **Skill** `/ral-reviewer` | plugin `skills/` | criterio: novedad, claims, voz, posicionamiento | — |

**Regla de oro del reparto:** el **hook/motor** atrapa lo *mecánico* (un grep nunca se distrae);
el **skill + vos** ponen el *criterio*. Nunca delegues lo mecánico a un agente (es otro modelo
→ vuelve el drift).

---

## 1. Prerequisito — instalar el plugin (una vez por máquina)

```bash
# si la marketplace "huao" no está agregada:
claude plugin marketplace add git@github.com:bryansgue/claude-skills.git
# instalar / actualizar:
claude plugin marketplace update huao
claude plugin install ral-reviewer@huao
```

Verificar: `/ral-reviewer` aparece como skill, y editar un `.tex` con "behaviour" debería
bloquear (el hook se activa al **inicio de la próxima sesión** tras instalar).

---

## 2. Estructura del repo de paper

```
<repo>/
├── writer_tuto.md                 # este archivo
├── .claude/
│   └── prose-ledger.txt           # jerga del proyecto (§3)   ← lo crea la IA
├── paper/
│   ├── main.tex                   # el manuscrito
│   └── ref_corpus/                # corpus SOTA (§4)          ← opcional
│       ├── <ref1>/*.tex
│       └── <ref2>/*.tex
└── CLAUDE.md                      # contexto/estado del proyecto (lo puntual, NO va al skill)
```

Regla de separación: **lo general** (reglas de escritura) está en el plugin; **lo puntual del
paper** (jerga, claims, datos, video, etc.) va en `prose-ledger.txt` + `CLAUDE.md` de este repo.

---

## 3. Alimentar la JERGA del proyecto → `.claude/prose-ledger.txt`

Acá le decís al linter qué palabras son coloquiales/acuñadas **en tu dominio**. La IA debería
**entrevistarte**: "¿qué términos venís usando que no son jerga formal del área? ¿qué palabra
detesta tu asesor?". Cada término va con su reemplazo.

**Formato** (lo lee `lint_prose.py` por walk-up desde el `.tex` / `$CLAUDE_PROJECT_DIR`):

```text
# Comentario con #
# term => replacement       → HARD: BLOQUEA la edición (exit 2)
termino_malo => termino_bueno
otra frase mala => frase correcta

[soft]
# todo lo que va DESPUÉS de [soft] es ADVISORY: avisa, no bloquea
palabra_ambigua => sugerencia
```

**Criterio HARD vs SOFT:**
- **HARD** = término distintivo, claramente coloquial/inventado en tu campo (alta precisión,
  no da falsos positivos). Ej. en racing: `racing line => racing path`, `gate miss => gate-crossing offset`.
- **SOFT** = palabra común y ambigua que *a veces* es legítima (ej. `bridge`, `modulate`).
  Avisa para que la revises contra el corpus, pero no frena.

**Mantenelo chico y de alta precisión.** Una palabra común en HARD = bloqueos molestos.
Ante la duda → `[soft]`.

---

## 4. El CORPUS SOTA → `paper/ref_corpus/` (la fuente de verdad de la jerga)

El corpus son los **fuentes LaTeX de los papers de referencia de tu área**. Sirve para dos cosas:
(a) decidir si una palabra es jerga válida (si está en el corpus, se usa; si no, sospechá);
(b) el barrido `--sweep` que lista palabras del draft que NO aparecen en el corpus.

**Armarlo** (bajar fuentes de arXiv — vienen en `.tex`):
```bash
mkdir -p paper/ref_corpus/<nombre_corto>
cd paper/ref_corpus/<nombre_corto>
curl -L https://arxiv.org/e-print/<ARXIV_ID> -o src.tar
tar xf src.tar && rm src.tar
```
Repetí por cada paper clave de tu dominio (4–6 alcanza). Es `.gitignore`-eable (pesado).

**Usarlo:**
```bash
python3 "$(find ~/.claude/plugins/cache/huao/ral-reviewer -name lint_prose.py | head -1)" \
        paper/main.tex --sweep
```
`--sweep` imprime los términos fuera-de-corpus (ruidoso, manual): la mayoría son falsos
positivos (vocabulario experimental que el corpus teórico no tiene) → **triage a mano**, y los
genuinamente coloquiales los mandás al `prose-ledger.txt`.

> La **regla cardinal**: *nunca* juzgues una palabra de memoria. **Grep el corpus antes** de
> aprobar o cambiar cualquier término. El asesor detecta edición externa/IA por una sola
> palabra fuera de registro.

---

## 5. Reglas que la IA aplica SIEMPRE al escribir (resumen operativo)

Estas son criterio (las pone la IA/skill); el hook sólo refuerza lo mecánico.

- **Registro formal, cero coloquialismos.** Nada de palabras "de charla". Si dudás, grep corpus.
- **Inglés americano (US)**: `-ize/-ization`, `behavior`, `modeled`. (El hook bloquea británico.)
- **Em-dash `---` en prosa**: evitarlo (suena a IA / al asesor no le gusta). Usar `:` `;` o coma.
  El en-dash `--` en compuestos (p.ej. `speed--safety`) **sí** es válido.
- **AI-tells fuera**: delve, leverage, seamless, intricate, showcase, realm, pivotal… (el hook
  los marca advisory; preferí el sinónimo simple).
- **Claims = código.** Toda afirmación cuantitativa/mecanismo se verifica contra el script o
  los datos reales antes de escribirla. Nunca editar el código para que matchee un número del paper.
- **Notación**: vectores `\bm` minúscula, matrices `\bm` mayúscula, sets `\mathbb`; un símbolo =
  un objeto; cada `∈` nombra el espacio más ajustado y verdadero.
- **Abstract**: sin símbolos (o todos), sin `n=`, sin masa/plataforma. Prosa corrida.
- **No sobre-condensar** (frases de 6 cláusulas = ilegible) ni sobre-expandir. Una idea por oración.
- **Cada sección a su scope**: nada de resultados sim-to-real en "Simulation Results", etc.

Detalle completo y específico de RA-L: invocá **`/ral-reviewer`** (trae las reglas duras de
formato/anonimato, la rúbrica de revisor, el SOTA del dominio y la rúbrica de notación/escritura).

---

## 6. Flujo de trabajo diario

```
1. Escribís/editás paper/main.tex
        │
        ▼
2. (auto) el HOOK lintea → si hay HARD violation: exit 2 + lista → la IA corrige y sigue
        │
        ▼
3. Pasada de CRITERIO con /ral-reviewer  (cuando querés revisar de verdad):
     - vocabulario fuera-de-corpus (--sweep)        ← jerga
     - rúbrica de revisor (novedad, claims, stats)  ← contenido
     - notación / math-writing                      ← formato técnico
        │
        ▼
4. Pre-submit: reglas duras RA-L (8pp, doble-anónimo, keywords, PaperCept) — ver el skill.
```

---

## 7. Cómo se cambian las reglas (recordatorio)

- **Jerga de ESTE paper** → editás `<repo>/.claude/prose-ledger.txt`. Local, sin push.
- **Regla general** (vale para todos tus papers) → editás el plugin en
  `~/projects/claude-skills/plugins/ral-reviewer/`, `git push`, `claude plugin update ral-reviewer@huao`.
  ⚠ Nunca edites el cache `~/.claude/plugins/cache/...` (se sobreescribe en cada update).
- **Cuándo dispara el hook** → `matcher` en el `hooks.json` del plugin.

---

## 8. Plantilla de arranque para `prose-ledger.txt`

Copiá esto a `<repo>/.claude/prose-ledger.txt` y completá con tu dominio:

```text
# Project prose ledger — <TU PAPER>. Lo lee el plugin ral-reviewer (lint_prose.py).
# term => replacement  (HARD, bloquea) ; tras [soft] = advisory.

# ---- HARD (jerga acuñada/coloquial de este paper) ----
# <termino_malo> => <termino_correcto>

# ---- SOFT (palabras ambiguas; revisar vs corpus) ----
[soft]
# <palabra_ambigua> => <sugerencia>
```
