# Arena branding

The existing originals were downloaded from the official HULKs website on
**2026-10-24**. The live
[sponsors page](https://hulks.de/sponsors/) and its linked application bundle
`/assets/index-DCikal5l.js` were checked that day. The site groups KDAB,
cc.systems, Dräger, GEA, Haspa, and Techniker Krankenkasse as **Industry Partners**
and Gegenstrom GmbH as a **Supporter**. TUHH is also a current partner, with
artwork at <https://hulks.de/sponsors/tuhh.png>. For a legible horizontal board,
the unchanged TUHH letter logo was downloaded from the university's own website
on **2026-10-25** instead of cropping or rearranging that stacked partner logo.

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
| TU Hamburg (TUHH) | `tuhh_logo.svg` | <https://www.tuhh.de/_assets/1af0d9ff6f1306e42935444821553917/Assets/images/logo-new.svg> | `tuhh_panel.png` |

The original files are preserved unchanged. Each `*_panel.png` is an opaque
768 × 256 RGB derivative, used by both renderers. The colored panels are
**local simulator derivatives**, made with the user's explicit authorization
for deterministic inversion and white-mask compositing. They are **not official
downloaded negative logo versions**. No sponsor shapes were redrawn, rearranged,
or generated.

| Panel | Background | Treatment |
| --- | --- | --- |
| HULKs | Green `#2FAC66` | Original mascot colors; white silhouette outline and white team text |
| KDAB | White | Original blue, white, and black artwork |
| cc.systems | Blue `#0C1A27` | White inversion mask, retaining the monogram's interior cutouts |
| Dräger | Navy `#002766` | Original alpha coverage composited in white |
| GEA | Blue `#0303B8` | Original alpha coverage composited in white, including the claim |
| Haspa | Red `#D9261C` | White inversion mask, retaining the subtitle's interior cutouts |
| Techniker Krankenkasse | Cyan `#009FE3` | White inversion mask of the complete health-partner lockup |
| Gegenstrom GmbH | White | Original gray and red artwork, enlarged proportionally |
| TUHH | Cyan `#00C1D4` | Original alpha coverage composited in white |

Background colors come from the original artwork. TUHH's primary cyan is also
specified on printed pages 36–37 of its
[brand identity guide](https://www.tuhh.de/t3resources/tuhh/download/universitaet/material/211018_GWW_tuhh_brand-identity_de.pdf);
printed pages 20–21 describe the letter logo. Other primary visual references:
[Dräger corporate redesign](https://www.draeger.com/de_de/Newsroom/about-draeger/Corporate-redesign),
[GEA presentation](https://cdn.gea.com/-/media/investors/annual-report/2024/esg-roadshow-presentation-en.pdf?rev=2866720405de45e391cd1f9b1c164148),
and [TK logo handbook](https://tk-markenportal.de/app/uploads/TK-Handbuch-Logoeinsatz.pdf).
These references provide context; the local masks are not downloaded from them.

Rebuild all nine panels offline with:

```sh
python3 tools/simulate/assets/branding/generate_panels.py
```

The script uses Pillow, ImageMagick's `magick` with SVG support, Fontconfig,
and the installed variable Noto Sans font (Bold for the team name, Regular for
the subtitle). With the same dependencies and originals, generation is
deterministic. SVG viewport dimensions are bounded proportionally to 1600 × 480
in memory before rasterization, then fitted inside 640 × 184 with Lanczos
resampling. The HULKs panel
uses a 210 × 210 mascot inside a 220 × 220 area: only the connected exterior
white backing is removed, and a four-pixel outline follows the existing
silhouette. The team name and “HAMBURG · ROBOCUP” are typeset in white.
Generation asserts RGB dimensions, inset nonempty artwork, and files under
100 kB each.

Textures are local: rendering does not contact the website. Bevy embeds the
panel PNGs; MuJoCo loads the same files from this asset directory. Each
renderer reuses the nine brand materials across all perimeter boards.
Original artwork and trademarks remain attributed to their respective owners.
