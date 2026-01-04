# TMCC to MTH Speed Mapping

## Engine Number Mapping

### Direct Mapping (No Offset):
- **Lionel Engine 1** → **MTH Engine 1**
- **Lionel Engine 11** → **MTH Engine 11**
- **Lionel Engine 99** → **MTH Engine 99**

### Previous Incorrect Mapping:
- **Lionel Engine 1** → **MTH Engine 2** ❌ (had +1 offset)
- **Lionel Engine 11** → **MTH Engine 12** ❌ (had +1 offset)

## Speed Conversion Details

### TMCC Speed Steps: 0-31 (32 steps)
### MTH Speed Range: 0-120 Smph (121 steps)

## Custom Fine-Control Mapping Formula

**Ultra-Fine Range (Steps 1-4):**
```
Step 1 = 1 Smph
Step 2 = 3 Smph  
Step 3 = 5 Smph
Step 4 = 10 Smph
```

**Fine Control Range (Steps 5-15):**
```
MTH_Speed = 10 + ((TMCC_Speed - 4) * 3)  # 3 Smph per step
```

**Medium Control Range (Steps 16-25):**
```
MTH_Speed = 55 + ((TMCC_Speed - 15) * 4)  # 4 Smph per step
```

**High Speed Range (Steps 26-31):**
```
MTH_Speed = 85 + ((TMCC_Speed - 25) / 6.0 * 35)  # ~5.8 Smph per step
```

## Speed Mapping Table

| TMCC Step | MTH Smph | Description |
|-----------|-----------|-------------|
| 0         | 0         | Stop |
| 1         | 1         | Ultra-fine crawl |
| 2         | 3         | Very fine creep |
| 3         | 5         | Fine creep |
| 4         | 10        | Very slow |
| 5         | 13        | Slow approach |
| 6         | 16        | Slow |
| 7         | 19        | Slow switching |
| 8         | 22        | Switching speed |
| 9         | 25        | Medium slow |
| 10        | 28        | Medium |
| 11        | 31        | Medium |
| 12        | 34        | Medium fast |
| 13        | 37        | Fast |
| 14        | 40        | Fast |
| 15        | 43        | Mainline speed |
| 16        | 47        | Mainline |
| 17        | 51        | High speed |
| 18        | 55        | High speed |
| 19        | 59        | Very fast |
| 20        | 63        | Normal maximum |
| 21        | 67        | Testing speed |
| 22        | 71        | Testing speed |
| 23        | 75        | Testing speed |
| 24        | 79        | Testing speed |
| 25        | 83        | Testing speed |
| 26        | 87        | Testing speed |
| 27        | 91        | Testing speed |
| 28        | 95        | Testing speed |
| 29        | 99        | Testing speed |
| 30        | 103       | Maximum testing |
| 31        | 120       | Absolute maximum |

## Why This Fine-Control Mapping?

### Ultra-Fine Range (Steps 1-4):
- **1 Smph minimum** for realistic movement
- **Perfect for switching** and yard operations
- **Step 4 = 10 Smph** gives you a good "creep" speed

### Fine Control Range (Steps 5-15):
- **3 Smph increments** for precise speed control
- **Your 25 Smph target**: TMCC Step 12 = 34 Smph
- **Great for mainline running** with fine adjustments

### Medium/High Ranges:
- **Linear progression** through remaining speed ranges
- **Preserves testing capability** up to 120 Smph
- **Smooth transitions** between speed ranges

## Operating Examples:

### Ultra-Fine Control (Steps 1-4):
- **1 Smph**: TMCC Step 1 (perfect crawl)
- **3 Smph**: TMCC Step 2 (very fine creep)
- **5 Smph**: TMCC Step 3 (fine creep)
- **10 Smph**: TMCC Step 4 (very slow)

### Typical Operation (Steps 5-15):
- **25 Smph**: TMCC Step 12 (34 Smph)
- **35 Smph**: TMCC Step 15 (43 Smph)
- **50 Smph**: TMCC Step 18 (59 Smph)

### High-Speed Testing (Steps 26-31):
- **80 Smph**: TMCC Step 27 (91 Smph)
- **100 Smph**: TMCC Step 29 (99 Smph)
- **120 Smph**: TMCC Step 31 (120 Smph)

## Relative Speed Commands

TMCC relative speed commands work in +/- 5 Smph increments:
- `+5` to `-5` Smph adjustments
- Maintains position within current speed range

## Benefits:
1. **Ultra-fine control**: 1 Smph minimum for realistic crawling
2. **Your exact preferences**: Steps 1-4 match your specifications
3. **Smooth progression**: Logical speed increases through all ranges
4. **Full testing range**: Maintains 120 Smph maximum capability
5. **Switching precision**: Excellent yard speed control

## Notes:
- TMCC Step 0 always = 0 Smph (stop)
- Steps 1-4: Ultra-fine control for precise operations
- Steps 5-15: Fine control with 3 Smph increments
- Steps 16-25: Medium control with 4 Smph increments  
- Steps 26-31: High-speed testing with ~5.8 Smph increments
