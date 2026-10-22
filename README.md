# Stylized LEAP-1A turbofan

One procedural Blender script builds a stylized CFM LEAP-1A high-bypass turbofan. Casings are spun from 2D station profiles. Each blade row is lofted once and arrayed about the engine axis. Nothing is downloaded: no models, textures, or HDRIs. There is no text, no render stamp, and no animation.

Two scenes live in `engine.blend`:

- **Exterior** — full nacelle, inlet, fan, and exhaust
- **Cutaway** — the same core, with a 115° wedge removed and the cut faces capped in red

## How to run

Requires Blender 4.3.x. These images were rendered with Blender 4.3.2, Cycles on CPU, 4 threads, OpenImageDenoise.

```bash
blender -b -P engine.py -- MODE VIEW [samples]
```

| MODE | What it does |
| --- | --- |
| `preview` | 960×540, 28 samples, writes `previews/` |
| `mid` | 1280×720, 64 samples, writes `previews/` |
| `still` | 1920×1080, 128 samples, writes `renders/` |
| `setup` | builds both scenes and saves `engine.blend` (no render) |

| VIEW | Camera |
| --- | --- |
| `cut` | cutaway, from inside the open wedge |
| `ext` | exterior three-quarter, uncut side |
| `side` | orthographic section of the cutaway, for checking proportions |

```bash
blender -b -P engine.py -- still cut
blender -b -P engine.py -- still ext
blender -b -P engine.py -- setup
blender -b -P engine.py -- preview side 16
```

`still cut` writes `renders/cutaway.png`. `still ext` writes `renders/exterior.png`. An optional third argument overrides the sample count.

## Knobs

All of these are constants at the top of `engine.py`. Change one and run the script again.

- `CUT_DEG` — degrees removed from the cutaway. It is 115, inside the 110–120 range used when a 90° wedge hides the core. The fan blade ring stays a full 360° either way. Exterior shells ignore this and spin the full circle.
- `FAN_BLADES` — fan blade count. The reference figure is 18.
- `LPC_ROTOR_N`, `LPC_STATOR_N` — three booster stages.
- `HPC_STAGES`, `HPC_ROTOR_N`, `HPC_STATOR_N` — ten-stage compressor.
- `HPT_STAGES`, `HPT_ROTOR_N`, `HPT_STATOR_N` and the matching `LPT_*` values.
- `OGV_BLADES`, `REAR_STRUTS`, `FUEL_NOZZLES`.

## LEAP-1A figures

Published numbers only. Internal flowpath radii are stylized so the bypass reads as a high-bypass duct and the ten modules stay readable. The certified length is flange to flange: the inlet lip is forward of x = 0 and the plug extends aft of x = 3.328 m.

| Figure | Value used | Source |
| --- | --- | --- |
| Fan diameter | 78 in = 1.9812 m | Safran, [LEAP-1A product page](https://www.safran-group.com/products-services/leap-1a-new-generation-engine-single-aisle-commercial-jets), “Fan diameter (in) 78” |
| Length | 3328 mm, fan-case forward flange to turbine-rear-frame aft flange | EASA type certificate data sheet EASA.E.110, LEAP-1A & LEAP-1C, Section 4 Dimensions, row LEAP-1A..G01 and G02+. PDF: [EASA.E.110](https://www.easa.europa.eu/en/downloads/20086/en) |
| Arrangement | 1 fan, 3-stage LPC, 10-stage HPC, annular combustor, 2-stage HPT, 7-stage LPT | Same TCDS, type description. The LEAP-1C cowl length (4505 mm) is a different measurement and is not used. |
| Fan blades | 18 | Aircraft Commerce, “In-service performance of the PW1100G & CFM LEAP-1A”, 2019, specification table. PDF: [issue 123](https://www.aircraft-commerce.com/wp-content/uploads/aircraft-commerce-docs/General%20Articles/2019/123_MTCE_B.pdf) |
| Bypass ratio | 11:1 | Same Aircraft Commerce table (fan diameter 78 in, BPR 11:1, LPC 3, HPC 10, HPT 2, LPT 7, 18 fan blades) |

One sentence in that article says “nine HPC”. The table in the same article, and the EASA description, both say 10. This model uses 10. Wikipedia’s CFM LEAP page agrees with 78 in, 11:1, 3.328 m, and an 18-blade fan; it is only a secondary check.

## Color

- Nacelle: white paint
- Cold section: titanium grey (LPC slightly lighter, HPC mid, stators darker)
- Fan blades: dark graphite, for the woven composite
- Hot section: bronze, with a lighter high-pressure turbine and a darker exhaust
- Low-pressure shaft: blue
- High-pressure shaft: orange
- Cut faces: red
- Combustor: orange emission inside the annulus

## What was rendered

Cycles CPU, 4 threads, 128 samples, 1920×1080, OpenImageDenoise, AgX.

| Image | Time | Size |
| --- | --- | --- |
| `renders/cutaway.png` | 97.5 s | 1.6 MB |
| `renders/exterior.png` | 85.8 s | 1.6 MB |
| `engine.blend` | build 0.44 s | 647 KB |

Evaluated vertex counts: Cutaway 90,276, Exterior 94,593. Both are far under the 2 million budget. The blend reopens in Blender 4.3.2 with both scenes, their cameras, and Cycles.

## Scope

A pylon stub, four bearing rings, and a procedural spinner stripe are included. The spinner stripe is a shader, not an image. The accessory gearbox, thrust-reverser cascades, and external plumbing are left off. The stills carry no labels, render stamp, or animation.
