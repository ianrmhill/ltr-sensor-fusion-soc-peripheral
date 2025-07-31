# Data-Acquisition IP — Scenario Catalogue  
*(Usage scnearios)*

---

## 1  Start-up / Power-up   (reference memories empty)

| # | CPU Command<br>(CMD0 shown) | Intended Action | RESULT<br>`ERR[15:13] / REF_TAG[12:10]` | Notes |
|---|-----------------------------|-----------------|-----------------------------------------|-------|
| 1 | **00 001 000**<br>Slow RAW | Auto-capture **REF1** | `110 / 000` | First-ever measurement of the instance |
| 2 | **01 001 000**<br>Slow RAW-REF1 | REF1 missing → error | `010 / 111` | Firmware learns it must initialise REF1 |
| 3 | **11 001 000**<br>FAST | No prior slow → *fast-mode error* | `000 / 111` | Wrapper sets `raw_err`, core converts |

---

## 2  After POR   (REF1 valid, REF2 empty)

| # | CPU Command | Intended Action | RESULT | Notes |
|---|-------------|-----------------|--------|-------|
| 4 | **00 001 000** | Auto-capture **REF2** | `110 / 000` | `por_seen` is high after reset |
| 5 | **10 001 000** | Slow RAW-REF2 | REF2 missing → `011 / 111` | |

---

## 3  Normal Operation   (both references valid)

| # | CPU Command | Intended Action | RESULT | Notes |
|---|-------------|-----------------|--------|-------|
| 6 | **00** … | Slow RAW | `001 / 000` | |
| 7 | **01** … | Slow RAW-REF1 | `001 / 001` | |
| 8 | **10** … | Slow RAW-REF2 | `001 / 010` | |
| 9 | **11** … | FAST (last slow) | `001 / 000 or 001 or 010` | REF_TAG echoes last slow type |

---

## 4  Reference-Configuration Nibble   (CMD3\[3:0\])

| `ref_cfg` | Operation | RESULT Codes |
|-----------|-----------|--------------|
| `0001` | **Print REF1** | OK → `001 / 011`<br>Missing → `010 / 111` |
| `0011` | **Print REF2** | OK → `001 / 100`<br>Missing → `011 / 111` |
| `0010` | Overwrite REF1 | Always `110 / 000` |
| `0100` | Overwrite REF2 | Always `110 / 000` |
| `0101` | Overwrite both | Always `110 / 000` |

---

## 5  ERR Code Quick Reference

| ERR[2:0] | Meaning |
|----------|---------|
| `000` | Fast-mode error (no data yet) |
| `001` | Measurement valid |
| `010` | REF1 missing |
| `011` | REF2 missing |
| `100` | Fast-acquisition underway (wrapper) |
| `101` | Slow-acquisition underway (wrapper) |
| `110` | Measurement valid **and** REF write/overwrite acknowledgment |
| `111` | General error / timeout |

## 6  REF_TAG Quick Reference

| REF_TAG[2:0] | Meaning |
|--------------|---------|
| `000` | RAW (no subtraction) |
| `001` | RAW − REF1 |
| `010` | RAW − REF2 |
| `011` | REF1 value (print) |
| `100` | REF2 value (print) |
| `111` | Error / undefined |
