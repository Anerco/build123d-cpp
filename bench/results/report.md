# C++ build123d-cpp vs Python build123d: results

Machine: AMD Ryzen 7 9800X3D 8-Core Processor (16 threads), Ubuntu 24.04.3 LTS on WSL2 (kernel 6.18.54.1-microsoft-standard-WSL2), 49332088 kB RAM.
C++: x86_64-conda-linux-gnu-c++ (conda-forge gcc 15.3.0-7) 15.3.0; library -O3 -DNDEBUG (CMake Release, libbuild123d.a), programs -std=c++20 -O2 -DNDEBUG; OCCT 8.0.0 (conda-forge, GCC: (conda-forge gcc 14.3.0-19) 14.3.0). Repo at b40258a.
Python 3.12.3: build123d 0.13.0, bd_warehouse 0.4.0, OCP 8.0.1.1.0 (OCCT 8.0.1), numpy 2.5.4. Measured 2026-11-10 23:41.

## Per model, combined

Build: in a running process, median (details below). Whole run: a fresh process that builds the model and writes one STEP file, median of 5; memory: its peak RSS.

**The 24 build123d examples**

| model | build C++ | build Python | Py / C++ | whole run C++ | whole run Python | C++ memory | Python memory |
|---|---:|---:|---:|---:|---:|---:|---:|
| tea_cup | 182 ms | 190 ms | 1.05 | 0.254 s | 1.50 s | 101 MB | 455 MB |
| vase | 50.8 ms | 55.5 ms | 1.09 | 0.098 s | 1.31 s | 74 MB | 427 MB |
| heat_exchanger | 2.95 s | 2.80 s | 0.95 | 2.860 s | 4.19 s | 142 MB | 495 MB |
| handle | 43.4 ms | 71.1 ms | 1.64 | 0.092 s | 1.34 s | 65 MB | 418 MB |
| lego | 92.7 ms | 103 ms | 1.11 | 0.150 s | 1.37 s | 64 MB | 417 MB |
| key_cap | 759 ms | 747 ms | 0.98 | 0.828 s | 2.03 s | 168 MB | 527 MB |
| pillow_block | 21.9 ms | 29.3 ms | 1.33 | 0.064 s | 1.28 s | 60 MB | 414 MB |
| din_rail | 126 ms | 172 ms | 1.36 | 0.207 s | 1.47 s | 74 MB | 428 MB |
| pegboard_j_hook | 33.0 ms | 38.1 ms | 1.16 | 0.077 s | 1.29 s | 62 MB | 415 MB |
| circuit_board | 42.5 ms | 58.0 ms | 1.36 | 0.107 s | 1.36 s | 63 MB | 415 MB |
| intersecting_chamfers | 41.9 ms | 51.7 ms | 1.23 | 0.087 s | 1.31 s | 61 MB | 414 MB |
| holes | 30.1 ms | 33.6 ms | 1.12 | 0.070 s | 1.29 s | 59 MB | 412 MB |
| boxes_on_faces | 33.4 ms | 36.5 ms | 1.09 | 0.077 s | 1.31 s | 58 MB | 410 MB |
| multiple_workplanes | 6.02 ms | 6.96 ms | 1.16 | 0.045 s | 1.27 s | 58 MB | 410 MB |
| loft | 1.05 s | 984 ms | 0.94 | 1.077 s | 2.26 s | 62 MB | 414 MB |
| extrude | 1.35 s | 1.37 s | 1.01 | 1.589 s | 2.85 s | 134 MB | 501 MB |
| clock | 342 ms | 360 ms | 1.05 | 0.536 s | 1.81 s | 76 MB | 432 MB |
| roller_coaster | 1.07 ms | 2.84 ms | 2.65 | 0.039 s | 1.26 s | 56 MB | 409 MB |
| canadian_flag | 225 ms | 238 ms | 1.06 | 0.297 s | 1.51 s | 78 MB | 432 MB |
| custom_sketch_objects | 84.6 ms | 106 ms | 1.25 | 0.149 s | 1.39 s | 76 MB | 433 MB |
| projection | 1.02 s | 1.05 s | 1.03 | 1.181 s | 2.43 s | 91 MB | 446 MB |
| build123d_logo | 12.8 ms | 24.6 ms | 1.92 | 0.120 s | 1.37 s | 62 MB | 417 MB |
| build123d_customizable_logo | 28.1 ms | 42.9 ms | 1.53 | 0.140 s | 1.39 s | 64 MB | 419 MB |
| joints | 123 ms | 139 ms | 1.13 | 0.188 s | 1.43 s | 71 MB | 424 MB |

**bd_warehouse parts**

| model | build C++ | build Python | Py / C++ | whole run C++ | whole run Python | C++ memory | Python memory |
|---|---:|---:|---:|---:|---:|---:|---:|
| gear_spur | 114 ms | 119 ms | 1.04 | 0.207 s | 1.56 s | 70 MB | 422 MB |
| gear_helical | 104 ms | 107 ms | 1.03 | 0.176 s | 1.53 s | 70 MB | 422 MB |
| gear_worm | 221 ms | 232 ms | 1.05 | 0.345 s | 1.70 s | 82 MB | 434 MB |
| thread_iso_external | 30.4 ms | 62.1 ms | 2.04 | 0.096 s | 1.36 s | 65 MB | 418 MB |
| thread_iso_internal | 30.6 ms | 62.8 ms | 2.05 | 0.094 s | 1.35 s | 66 MB | 418 MB |
| screw_shcs_simple | 30.1 ms | 38.4 ms | 1.27 | 0.071 s | 1.31 s | 65 MB | 419 MB |
| screw_shcs_threaded | 58.0 ms | 88.2 ms | 1.52 | 0.141 s | 1.43 s | 74 MB | 429 MB |
| nut_hex_simple | 25.1 ms | 27.9 ms | 1.11 | 0.069 s | 1.30 s | 63 MB | 415 MB |
| nut_hex_threaded | 57.8 ms | 101 ms | 1.75 | 0.116 s | 1.39 s | 67 MB | 420 MB |
| bearing_deep_groove | 51.0 ms | 64.0 ms | 1.26 | 0.093 s | 1.34 s | 61 MB | 415 MB |
| flange_weld_neck | 64.1 ms | 81.5 ms | 1.27 | 0.109 s | 1.55 s | 64 MB | 417 MB |
| sprocket | 197 ms | 206 ms | 1.04 | 0.297 s | 1.74 s | 95 MB | 448 MB |
| o_ring | 5 µs | 82 µs | 17.00 | 0.037 s | 1.26 s | 54 MB | 407 MB |
| openbuilds_cbeam_assembly | 900 ms | 1.09 s | 1.21 | 1.120 s | 2.70 s | 106 MB | 486 MB |

**Single operations**

| model | build C++ | build Python | Py / C++ | whole run C++ | whole run Python | C++ memory | Python memory |
|---|---:|---:|---:|---:|---:|---:|---:|
| box | 25 µs | 117 µs | 4.70 | 0.037 s | 1.24 s | 53 MB | 405 MB |
| plate | 17.7 ms | 20.2 ms | 1.14 | 0.058 s | 1.26 s | 61 MB | 413 MB |
| op_holes | 189 ms | 202 ms | 1.07 | 0.388 s | 1.60 s | 77 MB | 430 MB |
| op_fillet | 148 ms | 148 ms | 1.00 | 0.232 s | 1.44 s | 67 MB | 420 MB |
| op_text | 2.54 ms | 10.1 ms | 3.98 | 0.115 s | 1.36 s | 61 MB | 416 MB |
| occt_holes | 84.4 ms | 83.0 ms | 0.98 | 0.195 s | 1.40 s | 76 MB | 429 MB |
| occt_fillet | 136 ms | 133 ms | 0.98 | 0.220 s | 1.42 s | 67 MB | 419 MB |

## Startup (fresh process, median of the runs, peak RSS)

| what | wall | peak memory |
|---|---:|---:|
| empty C++ program linked to OCCT's libraries | 25.3 ms (n=15, 24.2 ms–26.3 ms) | 36 MB |
| C++ program start (no model) | 28.9 ms (n=15, 28.0 ms–30.1 ms) | 44 MB |
| C++ program start, bd_warehouse linked | 28.9 ms (n=15, 28.4 ms–30.1 ms) | 45 MB |
| python -I -c pass | 7.20 ms (n=15, 7.07 ms–7.41 ms) | 9 MB |
| import build123d | 1.25 s (n=15, 1.22 s–1.25 s) | 391 MB |
| import bd_warehouse | 33.9 ms (n=15, 33.2 ms–34.8 ms) | 16 MB |
| import bd_warehouse.thread | 1.36 s (n=15, 1.32 s–1.43 s) | 391 MB |
| import bd_warehouse.fastener | 1.35 s (n=15, 1.32 s–1.47 s) | 392 MB |
| import bd_warehouse.gear | 1.25 s (n=15, 1.22 s–1.31 s) | 391 MB |
| import bd_warehouse.bearing | 1.32 s (n=15, 1.29 s–1.44 s) | 393 MB |
| import bd_warehouse.flange | 1.24 s (n=15, 1.21 s–1.39 s) | 391 MB |
| import bd_warehouse.sprocket | 1.24 s (n=15, 1.22 s–1.27 s) | 391 MB |
| import bd_warehouse.o_rings | 1.38 s (n=15, 1.29 s–1.52 s) | 393 MB |
| import bd_warehouse.open_builds | 1.39 s (n=15, 1.32 s–1.53 s) | 409 MB |
| import build123d, timed in-process | 1.07 s (n=15, 1.03 s–1.16 s) |  |

`python -X importtime -c 'import build123d'`, cumulative, largest top-level imports: build123d 1191 ms, sklearn 314 ms, OCP 300 ms, scipy 218 ms, ezdxf 75 ms, requests 55 ms, svgpathtools 48 ms, numpy 45 ms.

## Build time inside a running process

Warmed up (2 untimed builds), then at least 10 builds and 2 s, at most 100; median, with ± half the interquartile range as a share of the median. The result is destroyed outside the timed region. 'same geometry': volume and area within 1e-6, same face count.

### The 24 build123d examples

| model | objects | C++ | Python | Python / C++ | runs C++/Py | same geometry |
|---|---:|---:|---:|---:|---:|:---:|
| tea_cup | 1 | 182 ms ±0% | 190 ms ±1% | 1.05 | 11/11 | yes |
| vase | 1 | 50.8 ms ±3% | 55.5 ms ±1% | 1.09 | 39/36 | yes |
| heat_exchanger | 1 | 2.95 s ±3% | 2.80 s ±2% | 0.95 | 10/10 | yes |
| handle | 2 | 43.4 ms ±3% | 71.1 ms ±1% | 1.64 | 45/29 | yes |
| lego | 1 | 92.7 ms ±3% | 103 ms ±0% | 1.11 | 22/20 | yes |
| key_cap | 1 | 759 ms ±0% | 747 ms ±1% | 0.98 | 10/10 | yes |
| pillow_block | 1 | 21.9 ms ±1% | 29.3 ms ±1% | 1.33 | 91/68 | yes |
| din_rail | 1 | 126 ms ±1% | 172 ms ±1% | 1.36 | 16/12 | yes |
| pegboard_j_hook | 1 | 33.0 ms ±1% | 38.1 ms ±1% | 1.16 | 61/53 | yes |
| circuit_board | 1 | 42.5 ms ±1% | 58.0 ms ±1% | 1.36 | 47/35 | yes |
| intersecting_chamfers | 1 | 41.9 ms ±0% | 51.7 ms ±1% | 1.23 | 48/39 | yes |
| holes | 4 | 30.1 ms ±2% | 33.6 ms ±1% | 1.12 | 67/60 | yes |
| boxes_on_faces | 1 | 33.4 ms ±1% | 36.5 ms ±1% | 1.09 | 60/55 | yes |
| multiple_workplanes | 1 | 6.02 ms ±2% | 6.96 ms ±1% | 1.16 | 100/100 | yes |
| loft | 1 | 1.05 s ±1% | 984 ms ±0% | 0.94 | 10/10 | yes |
| extrude | 8 | 1.35 s ±0% | 1.37 s ±0% | 1.01 | 10/10 | yes |
| clock | 1 | 342 ms ±1% | 360 ms ±1% | 1.05 | 10/10 | yes |
| roller_coaster | 1 | 1.07 ms ±4% | 2.84 ms ±3% | 2.65 | 100/100 | yes |
| canadian_flag | 2 | 225 ms ±1% | 238 ms ±1% | 1.06 | 10/10 | yes |
| custom_sketch_objects | 2 | 84.6 ms ±1% | 106 ms ±1% | 1.25 | 24/19 | yes |
| projection | 5 | 1.02 s ±0% | 1.05 s ±1% | 1.03 | 10/10 | yes |
| build123d_logo | 1 | 12.8 ms ±2% | 24.6 ms ±1% | 1.92 | 100/82 | yes |
| build123d_customizable_logo | 1 | 28.1 ms ±1% | 42.9 ms ±0% | 1.53 | 71/47 | yes |
| joints | 7 | 123 ms ±0% | 139 ms ±1% | 1.13 | 17/15 | yes |
| **sum; median ratio** | | **8.65 s** | **8.71 s** | **1.01** (sum), **1.12** (median) | | |

### bd_warehouse parts

| model | objects | C++ | Python | Python / C++ | runs C++/Py | same geometry |
|---|---:|---:|---:|---:|---:|:---:|
| gear_spur | 1 | 114 ms ±1% | 119 ms ±1% | 1.04 | 18/17 | yes |
| gear_helical | 1 | 104 ms ±1% | 107 ms ±1% | 1.03 | 20/19 | yes |
| gear_worm | 1 | 221 ms ±0% | 232 ms ±1% | 1.05 | 10/10 | yes |
| thread_iso_external | 1 | 30.4 ms ±1% | 62.1 ms ±1% | 2.04 | 63/33 | yes |
| thread_iso_internal | 1 | 30.6 ms ±0% | 62.8 ms ±3% | 2.05 | 66/31 | yes |
| screw_shcs_simple | 1 | 30.1 ms ±1% | 38.4 ms ±1% | 1.27 | 67/53 | yes |
| screw_shcs_threaded | 1 | 58.0 ms ±1% | 88.2 ms ±1% | 1.52 | 35/23 | yes |
| nut_hex_simple | 1 | 25.1 ms ±1% | 27.9 ms ±1% | 1.11 | 80/72 | yes |
| nut_hex_threaded | 1 | 57.8 ms ±1% | 101 ms ±1% | 1.75 | 35/20 | yes |
| bearing_deep_groove | 1 | 51.0 ms ±0% | 64.0 ms ±1% | 1.26 | 40/32 | yes |
| flange_weld_neck | 1 | 64.1 ms ±1% | 81.5 ms ±1% | 1.27 | 32/25 | yes |
| sprocket | 1 | 197 ms ±0% | 206 ms ±1% | 1.04 | 11/10 | yes |
| o_ring | 1 | 5 µs ±2% | 82 µs ±12% | 17.00 | 100/100 | yes |
| openbuilds_cbeam_assembly | 1 | 900 ms ±0% | 1.09 s ±1% | 1.21 | 10/10 | yes |
| **sum; median ratio** | | **1.88 s** | **2.28 s** | **1.21** (sum), **1.26** (median) | | |

### Single operations

| model | objects | C++ | Python | Python / C++ | runs C++/Py | same geometry |
|---|---:|---:|---:|---:|---:|:---:|
| box | 1 | 25 µs ±2% | 117 µs ±10% | 4.70 | 100/100 | yes |
| plate | 1 | 17.7 ms ±1% | 20.2 ms ±1% | 1.14 | 100/99 | yes |
| op_holes | 1 | 189 ms ±0% | 202 ms ±1% | 1.07 | 11/10 | yes |
| op_fillet | 1 | 148 ms ±1% | 148 ms ±0% | 1.00 | 14/14 | yes |
| op_text | 1 | 2.54 ms ±3% | 10.1 ms ±1% | 3.98 | 100/100 | yes |
| occt_holes | 1 | 84.4 ms ±1% | 83.0 ms ±1% | 0.98 | 24/25 | yes |
| occt_fillet | 1 | 136 ms ±1% | 133 ms ±0% | 0.98 | 15/15 | yes |
| **sum; median ratio** | | **578 ms** | **597 ms** | **1.03** (sum), **1.07** (median) | | |

### Export and import (op_holes' plate: 1 solid, 231 faces)

| model | objects | C++ | Python | Python / C++ | runs C++/Py | same geometry |
|---|---:|---:|---:|---:|---:|:---:|
| export_step | 1 | 151 ms ±1% | 151 ms ±0% | 1.00 | 14/14 | yes |
| export_stl | 1 | 1.83 s ±1% | 1.86 s ±1% | 1.02 | 10/10 | yes |
| export_gltf | 1 | 1.87 s ±1% | 1.83 s ±2% | 0.98 | 10/10 | yes |
| import_step | 1 | 204 ms ±1% | 200 ms ±1% | 0.98 | 10/10 | yes |
| **sum; median ratio** | | **4.05 s** | **4.05 s** | **1.00** (sum), **0.99** (median) | | |

## Whole run: a fresh process that builds the model and writes one STEP file

C++: the program; Python: `python -I` running the same model. Median of the runs (alternated C++/Python, after one untimed run of each), peak RSS from `/usr/bin/time -f %M`.

| model | C++ | C++ memory | Python | Python memory | Python / C++ | Python − C++ |
|---|---:|---:|---:|---:|---:|---:|
| tea_cup | 0.254 s ±3% | 101 MB | 1.496 s ±3% | 455 MB | 5.9 | +1.24 s |
| vase | 0.098 s ±1% | 74 MB | 1.315 s ±2% | 427 MB | 13.5 | +1.22 s |
| heat_exchanger | 2.860 s ±1% | 142 MB | 4.188 s ±1% | 495 MB | 1.5 | +1.33 s |
| handle | 0.092 s ±1% | 65 MB | 1.337 s ±1% | 418 MB | 14.6 | +1.25 s |
| lego | 0.150 s ±0% | 64 MB | 1.370 s ±0% | 417 MB | 9.2 | +1.22 s |
| key_cap | 0.828 s ±1% | 168 MB | 2.028 s ±0% | 527 MB | 2.4 | +1.20 s |
| pillow_block | 0.064 s ±1% | 60 MB | 1.277 s ±1% | 414 MB | 19.9 | +1.21 s |
| din_rail | 0.207 s ±0% | 74 MB | 1.474 s ±3% | 428 MB | 7.1 | +1.27 s |
| pegboard_j_hook | 0.077 s ±1% | 62 MB | 1.291 s ±1% | 415 MB | 16.8 | +1.21 s |
| circuit_board | 0.107 s ±1% | 63 MB | 1.362 s ±1% | 415 MB | 12.8 | +1.26 s |
| intersecting_chamfers | 0.087 s ±1% | 61 MB | 1.311 s ±1% | 414 MB | 15.1 | +1.22 s |
| holes | 0.070 s ±2% | 59 MB | 1.289 s ±1% | 412 MB | 18.3 | +1.22 s |
| boxes_on_faces | 0.077 s ±2% | 58 MB | 1.311 s ±2% | 410 MB | 17.0 | +1.23 s |
| multiple_workplanes | 0.045 s ±2% | 58 MB | 1.266 s ±1% | 410 MB | 28.4 | +1.22 s |
| loft | 1.077 s ±0% | 62 MB | 2.265 s ±1% | 414 MB | 2.1 | +1.19 s |
| extrude | 1.589 s ±1% | 134 MB | 2.851 s ±1% | 501 MB | 1.8 | +1.26 s |
| clock | 0.536 s ±3% | 76 MB | 1.807 s ±1% | 432 MB | 3.4 | +1.27 s |
| roller_coaster | 0.039 s ±2% | 56 MB | 1.262 s ±1% | 409 MB | 32.7 | +1.22 s |
| canadian_flag | 0.297 s ±1% | 78 MB | 1.510 s ±0% | 432 MB | 5.1 | +1.21 s |
| custom_sketch_objects | 0.149 s ±1% | 76 MB | 1.390 s ±3% | 433 MB | 9.4 | +1.24 s |
| projection | 1.181 s ±0% | 91 MB | 2.432 s ±0% | 446 MB | 2.1 | +1.25 s |
| build123d_logo | 0.120 s ±1% | 62 MB | 1.369 s ±1% | 417 MB | 11.4 | +1.25 s |
| build123d_customizable_logo | 0.140 s ±1% | 64 MB | 1.392 s ±1% | 419 MB | 9.9 | +1.25 s |
| joints | 0.188 s ±0% | 71 MB | 1.434 s ±3% | 424 MB | 7.6 | +1.25 s |
| gear_spur | 0.207 s ±1% | 70 MB | 1.562 s ±1% | 422 MB | 7.5 | +1.35 s |
| gear_helical | 0.176 s ±1% | 70 MB | 1.533 s ±1% | 422 MB | 8.7 | +1.36 s |
| gear_worm | 0.345 s ±1% | 82 MB | 1.702 s ±1% | 434 MB | 4.9 | +1.36 s |
| thread_iso_external | 0.096 s ±1% | 65 MB | 1.364 s ±2% | 418 MB | 14.2 | +1.27 s |
| thread_iso_internal | 0.094 s ±2% | 66 MB | 1.354 s ±3% | 418 MB | 14.4 | +1.26 s |
| screw_shcs_simple | 0.071 s ±1% | 65 MB | 1.315 s ±1% | 419 MB | 18.6 | +1.24 s |
| screw_shcs_threaded | 0.141 s ±0% | 74 MB | 1.426 s ±1% | 429 MB | 10.1 | +1.28 s |
| nut_hex_simple | 0.069 s ±1% | 63 MB | 1.298 s ±1% | 415 MB | 18.9 | +1.23 s |
| nut_hex_threaded | 0.116 s ±1% | 67 MB | 1.386 s ±0% | 420 MB | 12.0 | +1.27 s |
| bearing_deep_groove | 0.093 s ±2% | 61 MB | 1.340 s ±0% | 415 MB | 14.4 | +1.25 s |
| flange_weld_neck | 0.109 s ±1% | 64 MB | 1.553 s ±2% | 417 MB | 14.3 | +1.44 s |
| sprocket | 0.297 s ±1% | 95 MB | 1.741 s ±1% | 448 MB | 5.9 | +1.44 s |
| o_ring | 0.037 s ±1% | 54 MB | 1.262 s ±1% | 407 MB | 34.0 | +1.22 s |
| openbuilds_cbeam_assembly | 1.120 s ±0% | 106 MB | 2.698 s ±2% | 486 MB | 2.4 | +1.58 s |
| box | 0.037 s ±1% | 53 MB | 1.240 s ±1% | 405 MB | 33.8 | +1.20 s |
| plate | 0.058 s ±1% | 61 MB | 1.255 s ±1% | 413 MB | 21.6 | +1.20 s |
| op_holes | 0.388 s ±0% | 77 MB | 1.602 s ±1% | 430 MB | 4.1 | +1.21 s |
| op_fillet | 0.232 s ±1% | 67 MB | 1.440 s ±0% | 420 MB | 6.2 | +1.21 s |
| op_text | 0.115 s ±1% | 61 MB | 1.356 s ±1% | 416 MB | 11.8 | +1.24 s |
| occt_holes | 0.195 s ±1% | 76 MB | 1.401 s ±1% | 429 MB | 7.2 | +1.21 s |
| occt_fillet | 0.220 s ±0% | 67 MB | 1.421 s ±1% | 419 MB | 6.5 | +1.20 s |
| **median of the 45 models** | **0.140 s** | **66 MB** | **1.390 s** | **419 MB** | **10.1** | **+1.24 s** |

## bd_warehouse comparison suite, per module

Every case of tests/compare/cases_bdw_*, timed in-process by cpp/suite_main.cpp and py/suite.py: one untimed build, then one timed; sums of the timed builds. A shape case builds one object on each side (equal work). A value case returns values; several Python twins build the part once per value they read (e.g. bearing `*_default_parts` builds it 7 times, worm gear `*_values` 4 times) where the C++ twin builds it once, so value cases are not equal work. Every case gives the same result on both sides (volumes checked here too).

| module | shape cases | C++ | Python | Py / C++ | value cases | C++ | Python | Py / C++ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| bearing | 153 | 13.19 s | 15.76 s | 1.19 | 120 | 8.97 s | 13.86 s | 1.55 |
| bushing | 8 | 0.23 s | 0.27 s | 1.17 | 7 | 0.17 s | 0.20 s | 1.18 |
| fastener | 131 | 9.06 s | 10.64 s | 1.17 | 124 | 7.91 s | 8.74 s | 1.11 |
| flange | 562 | 35.40 s | 43.15 s | 1.22 | 24 | 35.40 s | 44.69 s | 1.26 |
| gear | 45 | 2.65 s | 3.17 s | 1.19 | 45 | 2.76 s | 7.41 s | 2.68 |
| o_rings | 81 | 0.09 s | 0.34 s | 3.85 | 98 | 0.09 s | 0.38 s | 4.39 |
| open_builds | 82 | 14.96 s | 22.10 s | 1.48 | 84 | 16.08 s | 23.30 s | 1.45 |
| pipe | 26 | 0.42 s | 0.50 s | 1.17 | 20 | 0.58 s | 0.77 s | 1.33 |
| retaining_ring | 188 | 5.04 s | 6.78 s | 1.35 | 196 | 5.03 s | 6.79 s | 1.35 |
| shaft_key | 142 | 0.08 s | 0.26 s | 3.24 | 141 | 0.08 s | 0.25 s | 3.26 |
| sprocket | 16 | 2.19 s | 2.38 s | 1.09 | 15 | 2.00 s | 2.53 s | 1.27 |
| thread | 39 | 3.51 s | 4.53 s | 1.29 | 57 | 0.46 s | 0.71 s | 1.53 |
| **all** | **1473** | **86.8 s** | **109.9 s** | **1.27** | **931** | **79.5 s** | **109.6 s** | **1.38** |

Whole timing programs (start, imports, every case built twice), C++/Python: bearing 45/61 s, bushing 1/2 s, fastener 35/41 s, flange 142/178 s, gear 11/23 s, o_rings 0/3 s, open_builds 63/93 s, pipe 2/4 s, retaining_ring 20/29 s, shaft_key 0/3 s, sprocket 9/11 s, thread 9/13 s.

### The suite as compare.py runs it

`build/compare_cases <names>` against `python cases_py.py <names>` (imports every case module, builds each case once, computes its metrics), whole processes:

| module | cases | C++ | Python | Py / C++ | C++ memory | Python memory |
|---|---:|---:|---:|---:|---:|---:|
| fastener | 255 | 20.2 s | 24.2 s | 1.20 | 150 MB | 568 MB |
| thread | 96 | 8.1 s | 10.7 s | 1.33 | 93 MB | 503 MB |
| bearing | 273 | 30.0 s | 39.1 s | 1.30 | 104 MB | 527 MB |
| gear | 90 | 9.4 s | 16.3 s | 1.74 | 104 MB | 519 MB |

## Where a Python build's time goes

py/occt_share.py counts the calls into OCP (OCCT) in a Python build and times them (sys.monitoring, 5 builds, median). 'in OCP' is measured while monitored, which adds about 1 µs per call, so it overstates OCCT's time; the per-call column uses only the call count and the unmonitored times above. results/explain.txt has the per-function breakdowns and the C++ call counts behind the notes.

| model | C++ build | Python build | Py − C++ | OCP calls per build | (Py − C++) per OCP call | in OCP (monitored) |
|---|---:|---:|---:|---:|---:|---:|
| tea_cup | 182 ms | 190 ms | 8.40 ms | 10825 | +0.8 µs | 200 ms |
| heat_exchanger | 2.95 s | 2.80 s | −151 ms | 158932 | -1.0 µs | 2.69 s |
| handle | 43.4 ms | 71.1 ms | 27.7 ms | 38262 | +0.7 µs | 62.0 ms |
| din_rail | 126 ms | 172 ms | 46.0 ms | 46085 | +1.0 µs | 160 ms |
| roller_coaster | 1.07 ms | 2.84 ms | 1.77 ms | 1852 | +1.0 µs | 2.29 ms |
| build123d_logo | 12.8 ms | 24.6 ms | 11.8 ms | 11287 | +1.0 µs | 21.3 ms |
| gear_spur | 114 ms | 119 ms | 4.31 ms | 5427 | +0.8 µs | 117 ms |
| thread_iso_external | 30.4 ms | 62.1 ms | 31.7 ms | 23515 | +1.3 µs | 50.3 ms |
| screw_shcs_threaded | 58.0 ms | 88.2 ms | 30.2 ms | 24633 | +1.2 µs | 78.5 ms |
| nut_hex_threaded | 57.8 ms | 101 ms | 43.1 ms | 34308 | +1.3 µs | 86.3 ms |
| bearing_deep_groove | 51.0 ms | 64.0 ms | 13.1 ms | 16222 | +0.8 µs | 60.5 ms |
| openbuilds_cbeam_assembly | 900 ms | 1.09 s | 190 ms | 143839 | +1.3 µs | 1.03 s |
| box | 25 µs | 117 µs | 92 µs | 120 | +0.8 µs | 133 µs |
| plate | 17.7 ms | 20.2 ms | 2.49 ms | 2402 | +1.0 µs | 19.6 ms |
| op_holes | 189 ms | 202 ms | 12.6 ms | 9526 | +1.3 µs | 194 ms |
| op_text | 2.54 ms | 10.1 ms | 7.57 ms | 623 | +12.1 µs | 9.86 ms |

## Reproducibility

A second in-process pass (45 models, results/repeat/): the medians moved by a median of 0.6% (C++) and 0.5% (Python), at most 5.9% and 7.0%; the Python/C++ ratios by a median of 1.0%, at most 5.7%.

