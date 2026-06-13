# L17 — améliorations proposées (au-delà du bonus livré)

Le bonus (greedy vs information-gain) est **implémenté + benchmarké**. Voici les
améliorations suivantes, classées par valeur/risque. Celles marquées ✅ sont déjà
faites ; les autres sont proposées (avec plan).

## Déjà implémenté (cette passe)
- ✅ **2 stratégies** greedy / info_gain (`gain−λ·cost`) + harness benchmark + plots + métriques (`time_to_X`, path, victimes).
- ✅ **SpinAndScan** : balayage caméra après chaque goal atteint.
- ✅ **Raffinement de goal** : snap sur la cellule libre la plus proche (`nearest_free_cell`) + filtre `info_gain_min_gain` (ignore les frontières à gain quasi-nul).

## Proposé — fiabilité Nav2 (le vrai reliquat, codex §5/§6)
1. **Pré-check `ComputePathToPose`** avant d'envoyer le goal : rejeter les frontières **sans chemin** (au lieu d'attendre l'échec NavigateToPose ~60 s) et utiliser la **longueur de chemin** comme `cost` de l'IG. *Gain :* moins de goals gaspillés → couverture 90 % plus vite. *Plan :* 2ᵉ action client, param `precheck_reachable`, async (non bloquant).
2. **Blacklist à durée de vie (TTL)** : une frontière blacklistée tôt peut redevenir joignable quand la carte grandit. *Plan :* blacklist = `{clé: expiry}`, nettoyage périodique ; param `blacklist_ttl_sec`.
3. **Tuning Nav2** (costmap/controller/progress-checker) : doc des paramètres + comparaison Navfn vs SmacPlanner2D sur `rescue_arena`.

## Proposé — preuve & artefacts (codex §4/§7/§11)
4. **Stats d'exploration dans `run_summary`** : goals envoyés/réussis/rejetés/blacklistés (le node publie `/exploration/stats`, `result_exporter` les fusionne).
5. **Carte finale annotée** auto en fin de run : `map_saver_cli` + `annotate_map.py` (consomme `victims.json`) → `final_map_annotated.png`. *Plan :* étape de finalisation dans `run_demo_tb4.sh`.
6. **Capture loop-closure** : log/figure depuis `slam_toolbox` pour le rapport.

## Proposé — qualité algo (mes ajouts)
7. **Gain par ray-casting** plutôt que comptage dans un rayon : compter les cellules `unknown` **visibles** depuis la frontière (lance des rayons jusqu'au premier obstacle) → gain plus fidèle (évite de compter l'inconnu derrière un mur).
8. **λ adaptatif** : augmenter λ quand la couverture stagne (favoriser le proche) et le baisser quand elle progresse (oser le lointain) → auto-tuning sans relancer.
9. **Frontière multi-résolution** : sous-échantillonner la grille pour la détection de frontières sur grandes cartes (perf), affiner localement.
10. **BT `SpinAndScan` / `RegisterVictims`** comme vrais nœuds BT (codex §9) au lieu d'une logique dans l'explorer → arbre de mission plus lisible/démontrable.

## Proposé — robustesse démo (codex §10/§14)
11. **Tests cas-limites** : caméra absente, 0 détection, TF indisponible, goal rejeté, carte figée, plus de frontière avant 90 %.
12. **Fallback détection couleur (HSV)** désactivé par défaut, comme filet de sécurité si l'AprilTag casse tardivement.
