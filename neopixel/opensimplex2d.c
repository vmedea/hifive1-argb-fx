/**
 * Copyright (c) 2020 A. Svensson
 * Copyright (c) 2021 Mara Huldra
 * SPDX-License-Identifier: MIT
 * From: https://github.com/lmas/opensimplex
 * Based on: https://gist.github.com/KdotJPG/b1270127455a94ac5d19
 * Simplified fixed-point 2D-only version.
  */
#include "opensimplex2d.h"

const osp_fixp STRETCH_CONSTANT_2D = OSP_FCONST(-0.211324865405187); // (1/Math.sqrt(2+1)-1)/2
const osp_fixp SQUISH_CONSTANT_2D = OSP_FCONST(0.366025403784439);  // (Math.sqrt(2+1)-1)/2
const osp_fixp NORM_CONSTANT_2D = 47;

/**
 * Gradients for 2D. They approximate the directions to the
 * vertices of an octagon from the center.
 */
const int8_t GRADIENTS_2D[] = {
     5,  2,    2,  5,
    -5,  2,   -2,  5,
     5, -2,    2, -5,
    -5, -2,   -2, -5,
};

void opensimplex2d_init(struct opensimplex2d *self, uint64_t seed)
{
    uint8_t source[256];
    for (int i = 0; i < 256; ++i) {
        source[i] = i;
    }
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    for (int i = 255; i >= 0; --i) {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        int r = (seed + 31) % (i + 1);
        self->perm[i] = source[r];
        source[r] = source[i];
    }
}

static osp_fixp extrapolate2d(const struct opensimplex2d *self, osp_fixp xsb, osp_fixp ysb, osp_fixp dx, osp_fixp dy)
{
    /*
    xsb and ysb are not in fixed-point
    dx and dy are
    returns fixed-point
    */
    int index = self->perm[(self->perm[xsb & 0xFF] + ysb) & 0xFF] & 0x0E;

    osp_fixp g1 = GRADIENTS_2D[index + 0];
    osp_fixp g2 = GRADIENTS_2D[index + 1];
    /* does not need OSP_MULT(…) because g1 and g2 aren't fixed-point */
    return g1 * dx + g2 * dy;
}

osp_fixp opensimplex2d_noise(const struct opensimplex2d *self, osp_fixp x, osp_fixp y)
{
    /* Place input coordinates onto grid. */
    osp_fixp stretch_offset = OSP_MULT(x + y, STRETCH_CONSTANT_2D);
    osp_fixp xs = x + stretch_offset;
    osp_fixp ys = y + stretch_offset;

    /* Floor to get grid coordinates of rhombus (stretched square) super-cell origin. */
    /* xwb and ysb are not in fixed-point */
    osp_fixp xsb = OSP_FLOOR(xs);
    osp_fixp ysb = OSP_FLOOR(ys);

    /* Skew out to get actual coordinates of rhombus origin. We'll need these later. */
    osp_fixp squish_offset = (xsb + ysb) * SQUISH_CONSTANT_2D;
    osp_fixp xb = OSP_FIXED(xsb) + squish_offset;
    osp_fixp yb = OSP_FIXED(ysb) + squish_offset;

    /* Compute grid coordinates relative to rhombus origin. */
    osp_fixp xins = xs - OSP_FIXED(xsb);
    osp_fixp yins = ys - OSP_FIXED(ysb);

    /* Sum those together to get a value that determines which region we're in. */
    osp_fixp in_sum = xins + yins;

    /* Positions relative to origin point. */
    osp_fixp dx0 = x - xb;
    osp_fixp dy0 = y - yb;

    osp_fixp value = 0;

    /* Contribution (1,0) */
    osp_fixp dx1 = dx0 - OSP_FIXED(1) - SQUISH_CONSTANT_2D;
    osp_fixp dy1 = dy0 - OSP_FIXED(0) - SQUISH_CONSTANT_2D;
    osp_fixp attn1 = OSP_FIXED(2) - OSP_MULT(dx1, dx1) - OSP_MULT(dy1, dy1);
    if (attn1 > 0) {
        attn1 = OSP_MULT(attn1, attn1);
        value += OSP_MULT(OSP_MULT(attn1, attn1), extrapolate2d(self, xsb + 1, ysb + 0, dx1, dy1));
    }

    /* Contribution (0,1) */
    osp_fixp dx2 = dx0 - OSP_FIXED(0) - SQUISH_CONSTANT_2D;
    osp_fixp dy2 = dy0 - OSP_FIXED(1) - SQUISH_CONSTANT_2D;
    osp_fixp attn2 = OSP_FIXED(2) - OSP_MULT(dx2, dx2) - OSP_MULT(dy2, dy2);
    if (attn2 > 0) {
        attn2 = OSP_MULT(attn2, attn2);
        value += OSP_MULT(OSP_MULT(attn2, attn2), extrapolate2d(self, xsb + 0, ysb + 1, dx2, dy2));
    }

    /* xsv_ext and ysv_ext are not in fixed-point */
    osp_fixp xsv_ext, ysv_ext;
    osp_fixp dx_ext, dy_ext;
    if (in_sum <= OSP_FIXED(1)) { /* We're inside the triangle (2-Simplex) at (0,0) */
        osp_fixp zins = OSP_FIXED(1) - in_sum;
        if (zins > xins || zins > yins) /* (0,0) is one of the closest two triangular vertices */
            if (xins > yins) {
                xsv_ext = xsb + 1;
                ysv_ext = ysb - 1;
                dx_ext = dx0 - OSP_FIXED(1);
                dy_ext = dy0 + OSP_FIXED(1);
            } else {
                xsv_ext = xsb - 1;
                ysv_ext = ysb + 1;
                dx_ext = dx0 + OSP_FIXED(1);
                dy_ext = dy0 - OSP_FIXED(1);
            }
        else { /* (1,0) and (0,1) are the closest two vertices. */
            xsv_ext = xsb + 1;
            ysv_ext = ysb + 1;
            dx_ext = dx0 - OSP_FIXED(1) - 2 * SQUISH_CONSTANT_2D;
            dy_ext = dy0 - OSP_FIXED(1) - 2 * SQUISH_CONSTANT_2D;
        }
    } else { /* We're inside the triangle (2-Simplex) at (1,1) */
        osp_fixp zins = OSP_FIXED(2) - in_sum;
        if (zins < xins || zins < yins) { /* (0,0) is one of the closest two triangular vertices */
            if (xins > yins) {
                xsv_ext = xsb + 2;
                ysv_ext = ysb + 0;
                dx_ext = dx0 - OSP_FIXED(2) - 2 * SQUISH_CONSTANT_2D;
                dy_ext = dy0 + OSP_FIXED(0) - 2 * SQUISH_CONSTANT_2D;
            } else {
                xsv_ext = xsb + 0;
                ysv_ext = ysb + 2;
                dx_ext = dx0 + OSP_FIXED(0) - 2 * SQUISH_CONSTANT_2D;
                dy_ext = dy0 - OSP_FIXED(2) - 2 * SQUISH_CONSTANT_2D;
            }
        } else { /* (1,0) and (0,1) are the closest two vertices. */
            dx_ext = dx0;
            dy_ext = dy0;
            xsv_ext = xsb;
            ysv_ext = ysb;
        }
        xsb += 1;
        ysb += 1;
        dx0 = dx0 - OSP_FIXED(1) - 2 * SQUISH_CONSTANT_2D;
        dy0 = dy0 - OSP_FIXED(1) - 2 * SQUISH_CONSTANT_2D;
    }

    /* Contribution (0,0) or (1,1) */
    osp_fixp attn0 = OSP_FIXED(2) - OSP_MULT(dx0, dx0) - OSP_MULT(dy0, dy0);
    if (attn0 > 0) {
        attn0 = OSP_MULT(attn0, attn0);
        value += OSP_MULT(OSP_MULT(attn0, attn0), extrapolate2d(self, xsb, ysb, dx0, dy0));
    }

    /* Extra Vertex */
    osp_fixp attn_ext = OSP_FIXED(2) - OSP_MULT(dx_ext, dx_ext) - OSP_MULT(dy_ext, dy_ext);
    if (attn_ext > 0) {
        attn_ext = OSP_MULT(attn_ext, attn_ext);
        value += OSP_MULT(OSP_MULT(attn_ext, attn_ext), extrapolate2d(self, xsv_ext, ysv_ext, dx_ext, dy_ext));
    }

    /* No need for fixed-point division as NORM_CONSTANT_2D is integer */
    return value / NORM_CONSTANT_2D;
}

#if 0
int main()
{
    struct opensimplex2d t;
    opensimplex2d_init(&t, 1);
    /*
    for (int i=0; i<256; ++i) {
        printf("%d\n", t.perm[i]);
    }
    */
    printf("stretch %d\n", STRETCH_CONSTANT_2D);
    printf("squish %d\n", SQUISH_CONSTANT_2D);
    printf("%d\n", opensimplex2d_noise(&t, 0, 0));
    printf("%d\n", opensimplex2d_noise(&t, 0x3ffff, 0x3ffff));
    printf("%d\n", opensimplex2d_noise(&t, 0x3ffff, 0));
    printf("%d\n", opensimplex2d_noise(&t, 0, 0x3ffff));
    printf("%d\n", opensimplex2d_noise(&t, 0xffff, 0xffff));
    printf("%d\n", opensimplex2d_noise(&t, 0xffff, 0));
    printf("%d\n", opensimplex2d_noise(&t, 0, 0xffff));
}
#endif
