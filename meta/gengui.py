#!/usr/bin/python
# coding: utf-8
import cairo, math

surface = cairo.ImageSurface(cairo.FORMAT_A8, 128, 128)
cr = cairo.Context(surface)

#cr.select_font_face("dejavu sans mono", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
cr.select_font_face("dejavu sans mono", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
cr.set_font_size(11)

glyphs = [" "] * 32 + map(chr, range(32, 128)) + [" "] * 128
glyphs[128] = "•"


W = H = 0
XB = YB = 0
for g in glyphs:
	xb, yb, w, h, dx, dy = cr.text_extents(g)
	W = max(W, int(w))
	H = max(H, int(h) + 1)
	XB = min(XB, xb)
	YB = min(YB, yb)

print "size =", (W, H)

for i, g in enumerate(glyphs):
	cr.move_to(i % 16 * W - XB, i / 16 * H - YB)
	cr.show_text(g)

# rounded corners
S = 12
for r in [0, 4, 6, 8]:
	cr.move_to(0, S)
	cr.arc(r, r, r, -math.pi, -math.pi / 2)
	cr.line_to(S, 0)
	cr.line_to(S, S)
	cr.close_path()
	cr.fill()
	cr.stroke()
	cr.translate(S + 4, 0)

L = 2
cr.set_line_width(L)
cr.translate(L * 0.5, L * 0.5)
for r in [0, 2, 4, 6]:
	cr.move_to(0, S)
	cr.arc(r, r, r, -math.pi, -math.pi / 2)
	cr.line_to(S, 0)
	cr.stroke()
	cr.translate(S + 4, 0)

cr.identity_matrix()

surface.write_to_png("gui.png")
