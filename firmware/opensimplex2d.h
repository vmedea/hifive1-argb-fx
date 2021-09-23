#ifndef H_OPENSIMPLEX2D
#define H_OPENSIMPLEX2D

#include <stdint.h>

#define OSP_SHIFT (16)
#define OSP_PRECISION (1 << (OSP_SHIFT))
typedef int64_t osp_fixp;

#define OSP_FLOOR(x) ((x) >> OSP_SHIFT)
#define OSP_MULT(x, y) (((x) * (y)) >> OSP_SHIFT)
#define OSP_FIXED(x) ((x) << OSP_SHIFT)
#define OSP_FCONST(x) ((osp_fixp)((x) * OSP_PRECISION))

struct opensimplex2d {
    uint8_t perm[256];
};

/** Initiate the class using a permutation array generated from a 64-bit seed number.
 */
void opensimplex2d_init(struct opensimplex2d *self, uint64_t seed);

/** Generate 2D OpenSimplex noise from X,Y coordinates.
 */
osp_fixp opensimplex2d_noise(const struct opensimplex2d *self, osp_fixp x, osp_fixp y);

#endif
