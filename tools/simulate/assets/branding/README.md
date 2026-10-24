# Arena branding

Downloaded from the official HULKs website on **2026-10-24**. The live
[sponsors page](https://hulks.de/sponsors/) and its linked application bundle
`/assets/index-DCikal5l.js` were checked that day. The site groups KDAB,
cc.systems, Dräger, GEA, Haspa, and Techniker Krankenkasse as **Industry Partners**
and Gegenstrom GmbH as a **Supporter**.

| Organization | Original local file | Official asset source | Arena texture |
| --- | --- | --- | --- |
| HULKs | `bighulk.png` | <https://hulks.de/bighulk.png> | `hulks_panel.png` |
| KDAB | `kdab_logo.svg` | <https://hulks.de/sponsors/kdab_logo.svg> | `kdab_panel.png` |
| cc.systems | `cc_logo_blue.svg` | <https://hulks.de/sponsors/cc_logo_blue.svg> | `cc_panel.png` |
| Dräger | `Draeger_L_Standard_RGB_Dark-Blue.svg` | <https://hulks.de/sponsors/Draeger_L_Standard_RGB_Dark-Blue.svg> | `draeger_panel.png` |
| GEA | `GEA_Logo_w_Claim_sRGB_VibrantBlue.svg` | <https://hulks.de/sponsors/GEA_Logo_w_Claim_sRGB_VibrantBlue.svg> | `gea_panel.png` |
| Haspa | `haspa.png` | <https://hulks.de/sponsors/haspa.png> | `haspa_panel.png` |
| Techniker Krankenkasse | `tk.png` | <https://hulks.de/sponsors/tk.png> | `tk_panel.png` |
| Gegenstrom GmbH | `gegenstrom.png` | <https://hulks.de/sponsors/gegenstrom.png> | `gegenstrom_panel.png` |

The original files are preserved unchanged. Each `*_panel.png` is an opaque
768 × 256 RGB derivative, used by both renderers. SVGs were rasterized with
ImageMagick at 192 DPI; the logos were fitted proportionally inside a
640 × 184 pixel area and centered on white with Pillow's Lanczos resampler.
The HULKs panel instead fits the team mascot inside 220 × 220 pixels alongside
the team name and “HAMBURG · ROBOCUP,” set in the installed Noto Sans font.
Sponsor artwork and colors are not redrawn or generated.

Textures are local: rendering does not contact the website. Bevy embeds the
panel PNGs; MuJoCo loads the same files from this asset directory. Each
renderer reuses the eight brand materials across all perimeter boards.
Original artwork and trademarks remain attributed to their respective owners.
