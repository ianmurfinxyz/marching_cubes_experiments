#ifndef _ISOLINES_H_
#define _ISOLINES_H_

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
init_isolines(struct point2d_t grid_pos_w_m);

void
tick_isolines(void);

void
draw_isolines(void);


#endif
