#include <inttypes.h>

/* the number of circles moving around the simulation; the interaction
 * between these circles and the grid creates the globs */
#define GLOB_COUNT 10

/* width/height of cells; same as distance between samples (unit: meters) */
#define CELL_SIZE_M 1.f

#define CORNER_BL 0b0001
#define CORNER_BR 0b0010
#define CORNER_TR 0b0100
#define CORNER_TL 0b1000

#define WEIGHT_BL 0
#define WEIGHT_BL 1
#define WEIGHT_BL 2
#define WEIGHT_BL 3

#define GET_CORNER(corner_mask, state_mask) (state_mask & corner_mask)
#define SET_CORNER(corner_mask, state_mask)  (state_mask |= corner_mask)
#define UNSET_CORNER(corner_mask, state_mask) (state_mask &= 0b1111 ^ corner_mask)

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

/* a sample is a point on the simulation grid in which a weight field is sampled, the size and 
 * resolution (cell_width, i.e. sample seperation) is of the grid determines the number of
 * sample points.
 *
 * The grid position of a sample is maintained by the grid.
 */
struct sample_t
{
  float weight;
}

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

/* change in position of the circle each tick, precomputation of:
 *    circle_speed * TICK_DELTA_S      */
#define CIRCLE_POS_DELTA_M_P_S 1.f

struct globber_t
{
  /* coordinates of the circle's center point w.r.t the grid */
  struct point2d_t center_g;

  /* radius of globber (unit: meters) */
  float r_m;

  /* direction globber is moving (unit vector) */
  struct vector2d_t dir;
};

/* dimensions of the grid (unit: lines of samples) */
#define GRID_SAMPLE_ROWS 100
#define GRID_SAMPLE_COLS 100

/* grid_sample_rows * grid_sample_cols */
#define SAMPLE_COUNT 10000 

/* the grid defines the root coordinate frame which the entire simulation is relative to. */
struct grid_t
{
  /* position of the grid origin w.r.t world space coordinates (unit: meters) */
  struct point2d_t pos_w_m;

  /* the grid samples, stored in column-major format, accessed [col][row] */
  struct sample_t samples[GRID_SAMPLES_COLS][GRID_SAMPLES_ROWS];
};

/* value of each color component of a sample when that sample is inactive */
#define SAMPLE_INACTIVE_GREY 0.4f

#define SAMPLE_VERTEX_COMPONENT_COUNT 3
#define SAMPLE_COLOR_COMPONENT_COUNT 2

/* convenience macros defining component ids for accessing sample data */
#define SAMPLE_VERTEX_X 0
#define SAMPLE_VERTEX_Y 1
#define SAMPLE_COLOR_R  0
#define SAMPLE_COLOR_G  1
#define SAMPLE_COLOR_B  2

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

/* the globbers that move around the grid, shaping the metaballs */
static struct globber_t globbers[GLOB_COUNT];

/* the simulation grid */
static struct grid_t grid;

/* min/max values of a sample weight. max=glob_count because each glob can contribute a 
 * max sample value of 1.f, the weight is the sum of all glob contributions */
static const weight_min = 0.f;
static const weight_max = (float)GLOB_COUNT;

static void
set_sample_vertex(int sample_col, int sample_row, float x_g, float y_g)
{
  assert(0 <= sample_col && sample_col < GRID_SAMPLE_COLS);
  assert(0 <= sample_row && sample_row < GRID_SAMPLE_ROWS);

  int sample_offset = ((sample_col * GRID_SAMPLE_ROWS * SAMPLE_VERTEX_COMPONENT_COUNT) +
                       (sample_row * SAMPLE_VERTEX_COMPONENT_COUNT));

  sample_vertices[sample_offset + SAMPLE_VERTEX_X] = x_g;
  sample_vertices[sample_offset + SAMPLE_VERTEX_Y] = y_g;
}

static void
set_sample_color(int sample_col, int sample_row, float r, float g, float b)
{
  assert(0 <= sample_col && sample_col < GRID_SAMPLE_COLS);
  assert(0 <= sample_row && sample_row < GRID_SAMPLE_ROWS);

  int sample_offset = ((sample_col * GRID_SAMPLE_ROWS * SAMPLE_COLOR_COMPONENT_COUNT) +
                       (sample_row * SAMPLE_COLOR_COMPONENT_COUNT));

  sample_colors[sample_offset + SAMPLE_COLOR_R] = r;
  sample_colors[sample_offset + SAMPLE_COLOR_G] = g;
  sample_colors[sample_offset + SAMPLE_COLOR_B] = b;
}

/* will map weight ranges to different sample colors to indicate the values of the weights, 
 * mapping as follows:
 *    0.f - 1.f  -> grey
 *    1.f - 10.f -> linear increase in green, so 1.f = 0 green, 20.f = max green
 *    >10.f      -> blue (indicates 'very high' value) */
static void
weight_to_color(float weight, float *r, float *g, float *b)
{
  assert(weight >= weight_min);
  assert(weight <= weight_max);

  *r = *g = *b = 0.f;

  if(weight < 1.f)
    *r = *g = *b = SAMPLE_INACTIVE_GREY;

  else if(weight < 10.f)
    *g = (weight - 1.f) * 0.11111111111111111f;  /* 1.0/9.0 = 0.11111111111 */

  else
    *b = 1.f;
}

static float
weight_function(struct point2d_t cell_pos_g, struct point2d_t glob_pos_g)
{

}

static float
calculate_sample_weight(struct sample_t sample)
{

}

void
init_metaballs(struct point2d_t grid_pos_w_m)
{
  float sx_g, sy_g;

  grid.pos_w_m = grid_pos_w_m;

  /* zero all weights */
  memset((void *)grid.samples, 0, sizeof(struct sample_t * GRID_SAMPLES_COLS * GRID_SAMPLES_ROWS));

  /* precompute sample point w.r.t grid space */
  for(int col = 0; col < GRID_SAMPLE_COLS; col++)
  {
    for(int row = 0; row < GRID_SAMPLE_ROWS; row++)
    {
      sx = col * CELL_SIZE_M;
      sy = row * CELL_SIZE_M;
      set_sample_vertex(col, row, sx_g, sy_g)
    }
  }

  /* set every color component of every sample to the same value; all colors grey */
  for(int i = 0; i < (SAMPLE_COUNT * 3); i++)
    sample_colors[i] = SAMPLE_INACTIVE_GREY;

  /* set initial positions and directions for the globbers */

}

void
tick_metaballs()
{
  /* integrate globbers */

  /* recalculate all weights */
}

void
draw_metaballs()
{
  /* draw cell corner points */

  /* draw cell borders */

  /* draw circles */

}





