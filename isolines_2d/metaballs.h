#ifndef _METABALLS_H_
#define _METABALLS_H_

/* a point (position vector) in a 2d plane */
struct point2d_t
{
  float x;
  float y;
};

/* a vector in a 2d plane */
struct vector2d_t
{
  float x;
  float y;
};

void
init_metaballs(struct point2d_t grid_pos_w_m);

void
tick_metaballs(void);

void
draw_metaballs(void);


#endif
