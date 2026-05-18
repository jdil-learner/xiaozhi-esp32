; Laser Engraving G-code
; Character: 'x'  Size: 5.0mm  Power: 800
G21 ; mm
G90 ; absolute
G0 X-1.190 Y-1.190
M3 S800
G1 X1.429 Y2.143 F800
M5
G0 X1.429 Y-1.190
M3 S800
G1 X-1.190 Y2.143 F800
M5
G0 X0 Y0
; End
