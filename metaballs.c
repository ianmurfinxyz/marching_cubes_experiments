#include <SDL2/SDL_opengl.h>
#include <inttypes.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "metaballs.h"

/*** SAMPLES *************************************************************************************/

/* value of each color component of a sample when that sample is inactive */
#define SAMPLE_INACTIVE_GREY 0.4f

/* convenience macros defining component offsets for accessing sample data */
#define SAMPLE_VERTEX_COMPONENT_COUNT 2
#define SAMPLE_COLOR_COMPONENT_COUNT 3
#define SAMPLE_VERTEX_X_OFFSET 0
#define SAMPLE_VERTEX_Y_OFFSET 1
#define SAMPLE_COLOR_R_OFFSET  0
#define SAMPLE_COLOR_G_OFFSET  1
#define SAMPLE_COLOR_B_OFFSET  2

#define SAMPLE_DRAW_DIAMETER_PX 4

/*** CELLS ***************************************************************************************/

/* width/height of cells; same as distance between samples (unit: meters) */
#define CELL_SIZE_M 1.f

#define CORNER_BL 0b0001
#define CORNER_BR 0b0010
#define CORNER_TR 0b0100
#define CORNER_TL 0b1000

#define WEIGHT_BL 0
#define WEIGHT_BR 1
#define WEIGHT_TL 2
#define WEIGHT_TR 3

#define GET_CORNER(corner_mask, state_mask) (state_mask & corner_mask)
#define SET_CORNER(corner_mask, state_mask)  (state_mask |= corner_mask)
#define UNSET_CORNER(corner_mask, state_mask) (state_mask &= 0b1111 ^ corner_mask)

/*** GLOBBERS ************************************************************************************/

/* the number of globs moving around the simulation; the interaction
 * between these globs and the sample grid creates the metaballs */
#define GLOB_COUNT 2

/* number of vertices used in the glob (circle) mesh */
#define GLOB_MESH_RESOLUTION 32 

/* width of glob circle mesh lines */
#define GLOB_DRAW_WIDTH_PX 2

#define GLOB_COLOR_R 1.f
#define GLOB_COLOR_G 0.f
#define GLOB_COLOR_B 0.f

/* change in position of globs each tick, precomputation of:
 *    glob_speed * TICK_DELTA_S      */
#define GLOB_POS_DELTA_M 0.01f

/*** GRID ****************************************************************************************/

/* dimensions of the grid (unit: lines of samples) */
#define SAMPLE_GRID_ROW_COUNT 10
#define SAMPLE_GRID_COL_COUNT 10

/* grid_sample_rows * grid_sample_cols */
#define SAMPLE_COUNT 100 

/* dimensions of the grid in meters */
static const float sample_grid_width_m = (SAMPLE_GRID_ROW_COUNT - 1) * CELL_SIZE_M;
static const float sample_grid_height_m = (SAMPLE_GRID_COL_COUNT - 1) * CELL_SIZE_M;

/*** SAMPLES *************************************************************************************/

/* a sample is a point on the simulation grid in which a weight field is sampled, the size and 
 * resolution (cell_width, i.e. sample seperation) is of the grid determines the number of
 * sample points.
 *
 * The grid position of a sample is maintained by the grid.
 */
struct sample_t
{
  float weight;
};

/* opengl 2.1 gfx data 
 *
 * each sample has a set of two vertex components (x and y) and a set of three color components
 * (r, g, b).
 *
 * vertices and colors are stored as a flattened 2d array so each sample can access it's data
 * as:
 *    sample_data[(col * (col_size * components_per_sample)) + 
 *                                         (row * components_per_sample) + component_id]
 * where:
 *    components_per_sample = 2 (for vertices), and = 3 (for colors)
 *    component_id = 0(->x) or 1(->y) (for vertices), and = 0(->r) or 1(->g) or 2(->b) (for colors).
 */
static GLfloat sample_vertices[SAMPLE_COUNT * 2];
static GLfloat sample_colors[SAMPLE_COUNT * 3];

/*** CELLS ***************************************************************************************/

/* a cell is composed of a group of 4 adjacent samples; each corner of a cell is
 * a sample. Each sample is shared by 4 cells in total. */
struct cell_t
{
  /* weight calculated for each corner, elements mapped to corners as:
   *  0b0001 -> [0]  (bottom-left)
   *  0b0010 -> [1]  (bottom-right)
   *  0b0100 -> [2]  (top-right)
   *  0b1000 -> [3]  (top-left)
   */
  struct sample_t samples[4];

  /* mask of which corners of the cell are active, illustrated below:
   *
   * where:
   *    + = inactive corner
   *    a = active corner
   *
   *  +--+           +--+          +--a          a--+
   *  |  |  = 0b0001 |  | = 0b0010 |  | = 0b0100 |  | = 0b1000
   *  a--+           +--a          +--+          +--+
   *
   *  i.e. corners wrapped anticlockwise starting from bottom-left.
   *
   * multipe active corners representated by multiple bits being set, e.g:
   *
   *   a--+
   *   |  | = 0b1001
   *   +--a
   *
   * note - most significant 4 bits are unused but must be zeroed.
  */
  uint8_t state_mask; 

  uint8_t _pad[3];
};

/*** GLOBBERS ************************************************************************************/

struct globber_t
{
  /* coordinates of the circle's center point w.r.t the grid (unit: meters) */
  struct point2d_t center_g_m;

  /* radius of globber (unit: meters) */
  float radius_m;

  /* direction globber is moving (unit vector) */
  struct vector2d_t dir;
};

/* the glob mesh drawn by opengl */
static GLfloat glob_vertices[GLOB_MESH_RESOLUTION * 2];

/* the globbers that move around the grid, shaping the metaballs */
static struct globber_t globbers[GLOB_COUNT] = {
  {{3.f, 3.f}, 1.f, {0.f, 1.f}},                        //TODO: generate globs with randomness 
  {{2.f, 1.f}, 1.f, {1.414213562f, 1.414213562f}}
};

/*** GRID ****************************************************************************************/

/* a grid of sample points. The square area between every set of adjacent samples is a cell. The 
 * grid defines the root coordinate frame which the entire simulation is relative to. */
struct sample_grid_t
{
  /* position of the grid origin w.r.t world space coordinates (unit: meters) */
  struct point2d_t pos_w_m;

  /* the grid samples, stored in column-major format, accessed [col][row] */
  struct sample_t samples[SAMPLE_GRID_COL_COUNT][SAMPLE_GRID_ROW_COUNT];
};

/* the simulation grid */
static struct sample_grid_t grid;

/*** SAMPLES *************************************************************************************/

static void
set_sample_vertex(int sample_col, int sample_row, float x_g, float y_g);

static void
init_sample_gfx_data(void)
{
  float sx_g, sy_g;

  /* precompute sample points w.r.t grid space */
  for(int col = 0; col < SAMPLE_GRID_COL_COUNT; col++)
  {
    for(int row = 0; row < SAMPLE_GRID_ROW_COUNT; row++)
    {
      sx_g = (float)col * (float)CELL_SIZE_M;
      sy_g = (float)row * (float)CELL_SIZE_M;

      set_sample_vertex(col, row, sx_g, sy_g);
    }
  }

  /* set every color component of every sample to the same value; all colors grey */
  for(int i = 0; i < (SAMPLE_COUNT * 3); i++)
    sample_colors[i] = SAMPLE_INACTIVE_GREY;
}

static void
set_sample_vertex(int sample_col, int sample_row, float x_g, float y_g)
{
  assert(0 <= sample_col && sample_col < SAMPLE_GRID_COL_COUNT);
  assert(0 <= sample_row && sample_row < SAMPLE_GRID_ROW_COUNT);

  int sample_offset = ((sample_col * SAMPLE_GRID_ROW_COUNT * SAMPLE_VERTEX_COMPONENT_COUNT) +
                       (sample_row * SAMPLE_VERTEX_COMPONENT_COUNT));

  sample_vertices[sample_offset + SAMPLE_VERTEX_X_OFFSET] = x_g;
  sample_vertices[sample_offset + SAMPLE_VERTEX_Y_OFFSET] = y_g;
}

static struct point2d_t
get_sample_vertex(int sample_col, int sample_row)
{
  assert(0 <= sample_col && sample_col < SAMPLE_GRID_COL_COUNT);
  assert(0 <= sample_row && sample_row < SAMPLE_GRID_ROW_COUNT);

  int sample_offset = ((sample_col * SAMPLE_GRID_ROW_COUNT * SAMPLE_VERTEX_COMPONENT_COUNT) +
                       (sample_row * SAMPLE_VERTEX_COMPONENT_COUNT));

  return (struct point2d_t){
    sample_vertices[sample_offset + SAMPLE_VERTEX_X_OFFSET],
    sample_vertices[sample_offset + SAMPLE_VERTEX_Y_OFFSET]
  };
}

static void
set_sample_color(int sample_col, int sample_row, float r, float g, float b)
{
  assert(0 <= sample_col && sample_col < SAMPLE_GRID_COL_COUNT);
  assert(0 <= sample_row && sample_row < SAMPLE_GRID_ROW_COUNT);

  int sample_offset = ((sample_col * SAMPLE_GRID_ROW_COUNT * SAMPLE_COLOR_COMPONENT_COUNT) +
                       (sample_row * SAMPLE_COLOR_COMPONENT_COUNT));

  sample_colors[sample_offset + SAMPLE_COLOR_R_OFFSET] = r;
  sample_colors[sample_offset + SAMPLE_COLOR_G_OFFSET] = g;
  sample_colors[sample_offset + SAMPLE_COLOR_B_OFFSET] = b;
}

/* weight function to calculate the weight contribution from a single glob */
static float
calculate_sample_weight(struct point2d_t sample_pos_g, struct globber_t *glob)
{
#define x0 glob->center_g_m.x
#define y0 glob->center_g_m.y
#define x1 sample_pos_g.x
#define y1 sample_pos_g.y
#define r2 (glob->radius_m * glob->radius_m)

  return r2 / (powf(x1 - x0, 2) + powf(y1 - y0, 2));

#undef x0 
#undef y0 
#undef x 
#undef y 
}

/* sums the weight contributions from all globs */
static float
calculate_sample_weights_sum(struct point2d_t sample_pos_g_m)
{
  struct globber_t *glob;
  float weight = 0.f;

  for(int i = 0; i < GLOB_COUNT; i++)
  {
    glob = &globbers[i]; 
    weight += calculate_sample_weight(sample_pos_g_m, glob);
  }

  return weight;
}

/* maps weight ranges to different sample colors to indicate the values of the weights, 
 * mapping as follows:
 *    0.f - 1.f  -> grey
 *    1.f - 10.f -> linear increase in green, so 1.f = 0 green, 20.f = max green
 *    >10.f      -> blue (indicates 'very high' value) */
static void
weight_to_color(float weight, float *r, float *g, float *b)
{
  *r = *g = *b = 0.f;

  if(weight < 1.f)
    *r = *g = *b = SAMPLE_INACTIVE_GREY;

  else if(weight < 20.f)
    *g = (((weight - 1.f) * 0.05263157895f) * 0.6f) + 0.4f;  /* 1.0/19.0 = 0.05263157895 */

  else
    *r = *g = *b = 1.f;
}

static void
draw_samples(void)
{
  glPointSize(SAMPLE_DRAW_DIAMETER_PX);
  glVertexPointer(2, GL_FLOAT, 0, sample_vertices);
  glColorPointer(3, GL_FLOAT, 0, sample_colors);
  glDrawArrays(GL_POINTS, 0, SAMPLE_COUNT);
  glPointSize(1.f);
}

/*** GLOBBERS ************************************************************************************/

/* the glob mesh is a simple circle of radius 1 centered about a local origin */
static void
generate_glob_mesh(void)
{
  float angle_rad = 0.f, delta_angle_rad = 2 * M_PI / GLOB_MESH_RESOLUTION; 
  int i = 0;
  while(i != GLOB_MESH_RESOLUTION * 2)
  {
    glob_vertices[i++] = cos(angle_rad);
    glob_vertices[i++] = sin(angle_rad);
    angle_rad += delta_angle_rad; 
  }
}

/* handles collisions between the glob and the 4 grid boundary planes: 
 *    x = 0
 *    x = sample_grid_width_m
 *    y = 0
 *    y = sample_grid_height_m          */
static void
handle_glob_collisions(struct globber_t *glob)
{
  /* plane: x = 0 */
  if((glob->center_g_m.x < glob->radius_m) && (glob->dir.x < 0.f))
  {
    glob->dir.x = -glob->dir.x;
  }

  /* plane: x = sample_grid_width_m */
  else if(((sample_grid_width_m - glob->center_g_m.x) < glob->radius_m) && (glob->dir.x > 0.f))
  {
    glob->dir.x = -glob->dir.x;
  }

  /* plane: y = 0 */
  if((glob->center_g_m.y < glob->radius_m) && (glob->dir.y < 0.f))
  {
    glob->dir.y = -glob->dir.y;
  }

  /* plane: x = sample_grid_width_m */
  else if(((sample_grid_height_m - glob->center_g_m.y) < glob->radius_m) && (glob->dir.y > 0.f))
  {
    glob->dir.y = -glob->dir.y;
  }
}

static void
tick_globs(void)
{
  struct globber_t *glob;

  for(int i = 0; i < GLOB_COUNT; i++)
  {
    glob = &globbers[i]; 
    glob->center_g_m.x += glob->dir.x * GLOB_POS_DELTA_M;
    glob->center_g_m.y += glob->dir.y * GLOB_POS_DELTA_M;

    handle_glob_collisions(glob);
  }
}

static void
draw_globs(void)
{
  struct globber_t *glob;

  glDisableClientState(GL_COLOR_ARRAY);
  glColor3f(GLOB_COLOR_R, GLOB_COLOR_G, GLOB_COLOR_B);
  glLineWidth(GLOB_DRAW_WIDTH_PX);
  glVertexPointer(2, GL_FLOAT, 0, glob_vertices);
  for(int i = 0; i < GLOB_COUNT; i++)
  {
    glob = &globbers[i]; 
    glPushMatrix();
    glTranslatef(glob->center_g_m.x, glob->center_g_m.y, 0.f);
    glScalef(glob->radius_m, glob->radius_m, glob->radius_m);
    glDrawArrays(GL_LINE_LOOP, 0, GLOB_MESH_RESOLUTION);
    glPopMatrix();
  }
  glLineWidth(1.f);
  glEnableClientState(GL_COLOR_ARRAY);
}

/*** GRID ****************************************************************************************/

static void
init_grid(void)
{

}

static void
tick_grid(void)
{
  struct point2d_t sample_pos_g_m;
  float r, g, b;
  float *weight;

  for(int col = 0; col < SAMPLE_GRID_COL_COUNT; col++)
  {
    for(int row = 0; row < SAMPLE_GRID_ROW_COUNT; row++)
    {
      weight = &grid.samples[col][row].weight;
      sample_pos_g_m = get_sample_vertex(col, row);
      *weight = calculate_sample_weights_sum(sample_pos_g_m);
      weight_to_color(*weight, &r, &g, &b);
      set_sample_color(col, row, r, g, b);
    }
  }
}

/*** MODULE INTERFACE  ***************************************************************************/

void
init_metaballs(struct point2d_t grid_pos_w_m)
{

  grid.pos_w_m = grid_pos_w_m;

  /* zero all sample weights */
  memset((void *)grid.samples, 0, sizeof(struct sample_t) * SAMPLE_GRID_COL_COUNT * SAMPLE_GRID_ROW_COUNT);

  init_sample_gfx_data();

  generate_glob_mesh();

  /* set initial positions and directions for the globbers */

}

void
tick_metaballs(void)
{
  tick_globs();
  tick_grid();


  /* recalculate all weights */
}

void
draw_metaballs(void)
{
  glPushMatrix();
  glTranslatef(grid.pos_w_m.x, grid.pos_w_m.y, 0.f);

  glEnableClientState(GL_COLOR_ARRAY);

  draw_samples();

  /* draw cell borders */

  draw_globs();

  glPopMatrix();
}

