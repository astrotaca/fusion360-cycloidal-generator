# Cycloidal Dual-Disc Sketch Generator (Fusion 360)

![Dual-disc cycloidal reducer](preview.png)

Small Fusion 360 script for generating cycloidal reducer sketches.

The main goal was to properly support **dual-disc cycloidal drives** with correct phasing and identical output hole positions.  
Most generators I found only handle a single disc, so I ended up making my own.

It generates:
- Ring pins
- Output pins
- One or two cycloidal discs (optional)
- Automatic phase for dual-disc setups

Works for higher ratios as well (20:1+), though high settings can be heavy on slower PCs.  
This is mainly due to dense curve sampling and adaptive subdivision used to keep the cycloidal profile stable and printable.


This is a practical tool, not a polished product. Feel free to give feedback or tweak it to your needs.

<img src="examplesketch.png" width="400">

