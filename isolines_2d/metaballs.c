#include <SDL2/SDL_opengl.h>
#include <inttypes.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

#include "metaballs.h"

/*** SAMPLES *************************************************************************************/

/* value of each color component of a sample when that sample is inactive */
#define SAMPLE_INACTIVE_GREY 0.3f

/* convenience macros defining component offsets for accessing sample data */
#define SAMPLE_VERTEX_COMPONENT_COUNT 2
#define SAMPLE_COLOR_COMPONENT_COUNT 3
#define SAMPLE_VERTEX_X_OFFSET 0
#define SAMPLE_VERTEX_Y_OFFSET 1
#define SAMPLE_COLOR_R_OFFSET  0
#define SAMPLE_COLOR_G_OFFSET  1
#define SAMPLE_COLOR_B_OFFSET  2

/* controls the size of rendered sample points */
#define SAMPLE_DRAW_DIAMETER_PX 3

/*** CELLS ***************************************************************************************/

/* width/height of cells; same as distance between samples (unit: meters) */
#define CELL_SIZE_M 0.3f

/*** GLOBBERS ************************************************************************************/

/* the number of globs moving around the simulation; the interaction
 * between these globs and the sample grid creates the metaballs (isolines) */
#define GLOB_COUNT 15 

/* range of randomly generated glob radi */
#define GLOB_MAX_RADIUS_M 3.f
#define GLOB_MIN_RADIUS_M 1.f

/* number of vertices used in the glob (circle) mesh */
#define GLOB_MESH_RESOLUTION 32 

/* width of glob circle mesh lines */
#define GLOB_DRAW_WIDTH_PX 3

/* color of the roaming globbers */
#define GLOB_COLOR_R 0.f
#define GLOB_COLOR_G 1.f
#define GLOB_COLOR_B 1.f

/* change in position of globs each tick, precomputation of:
 *    glob_speed * TICK_DELTA_S      */
#define GLOB_POS_DELTA_M 0.01f

/*** GRID ****************************************************************************************/

/* dimensions of the grid (unit: lines of samples) 
 *
 * note - IF YOU CHANGE THESE VALUES YOU MUST ALSO UPDATE 'SAMPLE_COUNT' */
#define SAMPLE_GRID_ROW_COUNT 100
#define SAMPLE_GRID_COL_COUNT 100

/* GRID_SAMPLE_ROW_COUNT * GRID_SAMPLE_COL_COUNT */
#define SAMPLE_COUNT 10000 

/* the maximum number of vertex components in the grid mesh; the size is twice the max number
 * of vertices; two components per vertex (2D). Should be precomputed value of:
 *      (SAMPLE_GRID_ROW_COUNT - 1) * (SAMPLE_GRID_COL_COUNT - 1) * 4 * 2 
 * why?
 *  SAMPLE_GRID_ROW/COL_COUNT = number of cell rows/columns 
 *  4 = max vertices per cell    
 *  2 = max components per vertex    
 *
 * However such a mesh will be significantly too large; a grid of 200x200 cells would have a limit
 * of:
 *      199 * 199 * 4 * 2 = 316808 floats
 *
 * This would only occur if every sample was alternately actived, which will never occur with
 * globbers. Thus set the size to a guesstimate and use trial and error to find a good value for
 * a given mesh size and glob count. Under normal conditions the generated meshes will be 
 * considerably smaller than the max. If the mesh is too small the program will abort and tell
 * you. */
#define ISOLINES_MESH_MAX_SIZE 13000

#define ISOLINES_MESH_COLOR_R 1.f
#define ISOLINES_MESH_COLOR_G 0.f
#define ISOLINES_MESH_COLOR_B 0.4f

#define ISOLINES_MESH_DRAW_WIDTH_PX 3

/* dimensions of the grid in meters */
static const float sample_grid_width_m = (SAMPLE_GRID_ROW_COUNT - 1) * CELL_SIZE_M;
static const float sample_grid_height_m = (SAMPLE_GRID_COL_COUNT - 1) * CELL_SIZE_M;

/* the number of threshold levels (or isovalues) for which to generate and render isolines. The
 * greater this number the more memory will be required for the isolines mesh, thus make sure
 * to increase that too (set with ISOLINES_MESH_MAX_SIZE) */
#define THRESHOLD_COUNT 5

/* the threshold (isovalues) to generate contour lines for */
static const float thresholds[THRESHOLD_COUNT] = {0.6f, 0.8f, 1.f, 1.3f, 2.f};

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

#define SET_CORNER(corner_mask, state_mask) (state_mask |= corner_mask)

#define CELL_WEIGHT_BL 0
#define CELL_WEIGHT_BR 1
#define CELL_WEIGHT_TR 2
#define CELL_WEIGHT_TL 3

#define CELL_POINT_L 0
#define CELL_POINT_B 1
#define CELL_POINT_R 2
#define CELL_POINT_T 3
#define CELL_POINT_NULL -1

/* marching squares lookup table. Note that this is a table of indices not points. */
int8_t const cell_lookup[16][4] = {
  {CELL_POINT_NULL, CELL_POINT_NULL, CELL_POINT_NULL, CELL_POINT_NULL}, /* case 0 */
  {CELL_POINT_L   , CELL_POINT_B   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 1 */
  {CELL_POINT_B   , CELL_POINT_R   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 2 */
  {CELL_POINT_L   , CELL_POINT_R   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 3 */
  {CELL_POINT_R   , CELL_POINT_T   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 4 */
  {CELL_POINT_L   , CELL_POINT_T   , CELL_POINT_B   , CELL_POINT_R   }, /* case 5 */
  {CELL_POINT_B   , CELL_POINT_T   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 6 */
  {CELL_POINT_L   , CELL_POINT_T   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 7 */
  {CELL_POINT_L   , CELL_POINT_T   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 8 */
  {CELL_POINT_B   , CELL_POINT_T   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 9 */
  {CELL_POINT_L   , CELL_POINT_B   , CELL_POINT_R   , CELL_POINT_T   }, /* case 10 */
  {CELL_POINT_R   , CELL_POINT_T   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 11 */
  {CELL_POINT_L   , CELL_POINT_R   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 12 */
  {CELL_POINT_B   , CELL_POINT_R   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 13 */
  {CELL_POINT_L   , CELL_POINT_B   , CELL_POINT_NULL, CELL_POINT_NULL}, /* case 14 */
  {CELL_POINT_NULL, CELL_POINT_NULL, CELL_POINT_NULL, CELL_POINT_NULL}  /* case 15 */
};

/* a cell is composed of a group of 4 adjacent samples; each corner of a cell is
 * a sample. Each sample is shared by 4 cells in total. */
struct cell_t
{
  /* weight of each corner, elements mapped to corners as:
   *  0b0001 -> [0]  (bottom-left)
   *  0b0010 -> [1]  (bottom-right)
   *  0b0100 -> [2]  (top-right)
   *  0b1000 -> [3]  (top-left)
   */
  struct sample_t samples[4];

  /* geometry generated by the cell, each element represents a specific generated point:
   *        Pt                                             
   *    +----x----+                                             y
   *    |         |       Pl = left point   -> element 0        ^
   * Pl x         x Pr    Pb = bottom point -> element 1        |
   *    |         |       Pr = right point  -> element 2        |
   *    o----x----+       Pt = top point    -> element 3        o------> x
   *        Pb                                               coordinate space of the cell
   *
   * by default points are simply set to the intermediate positions between the corners. Point
   * values are taken w.r.t a local origin defined to coincide with the bottom-left corner. In
   * the above diagram the origin is represented with 'o'. When generating the final mesh from
   * this data the points must be translated into the grid space, i.e. the position of the cell
   * within the grid must be taken into account. The size of the cell is given by CELL_SIZE_M.
   */
  struct point2d_t points[4];

  /* indicies into the points array which define how points are connected to form lines, this is
   * very similar to an index buffer for a vertex array, as illustrated below:
   *
   * where:
   *    + = inactive corner
   *    a = active corner
   *
   *   +-------x-------a
   *   |       |       | 
   *   |       |       |
   *   |       |       | = 0b0110    ==>   indices = {1, 3, -1, -1}  
   *   |       |       |                         i.e. {bottom_point, top_point, null, null}
   *   |       |       |
   *   +-------x-------a
   *
   * or
   *
   *   +-------x-------a
   *   |      -        | 
   *   |   -           |
   *   x-             -x = 0b0101    ==>   indices = {0, 3, 1, 2} 
   *   |           -   |                i.e. {left_point, top_point, bottom_point, right_point}
   *   |        -      |
   *   a-------x-------+
   *
   * this information is used to inform the user of the cell which points are valid and in what
   * order they should be extracted to construct the mesh.
   */
  int8_t indices[4];

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
};

/* compute the geometry of the cell based on the 4 sample values and the threshold; the threshold 
 * is the boundary between weights considered 'active' and 'inactive'. Samples are expected in
 * the order:
 *    {bottom_left, bottom_right, top_right, top_left} */
static void
compute_cell(struct sample_t samples[4], float threshold, struct cell_t *cell)
{
  static struct point2d_t default_points[4] = {
    {0.f               , CELL_SIZE_M * 0.5f},
    {CELL_SIZE_M * 0.5f, 0.f               },
    {CELL_SIZE_M       , CELL_SIZE_M * 0.5f},
    {CELL_SIZE_M * 0.5f, CELL_SIZE_M       }
  };

  cell->state_mask = 0;
  memcpy((void *)&cell->samples[0], (void *)&samples[0], sizeof(struct sample_t) * 4);
  memcpy((void *)&cell->points[0], (void *)&default_points[0], sizeof(struct point2d_t) * 4);

  /* compute active corner mask */
  for(int i = CELL_WEIGHT_BL; i <= CELL_WEIGHT_TL; i++)
    if(samples[i].weight >= threshold)
      SET_CORNER((0b0001 << i), cell->state_mask);

  assert(cell->state_mask <= 15);

  /* lookup indices */ 
  memcpy((void *)&cell->indices[0], (void *)&cell_lookup[cell->state_mask][0], sizeof(uint8_t) * 4);
}

/* used in function 'lerp_cell' */
static float
lerp(float threshold, float minor_weight, float major_weight)
{
  return CELL_SIZE_M * ((threshold - minor_weight) / (major_weight - minor_weight));
}

/* linearly interpolate the points of a computed cell; cell must be created with function
 * 'compute_cell' prior to the lerp. Interpolation is performed as follows:
 *
 * For the cell:
 *
   *  TL    Pt     TR                                      
   *    +----x----+                                             y
   *    |         |       Pl = left point   -> element 0        ^
   * Pl x         x Pr    Pb = bottom point -> element 1        |
   *    |         |       Pr = right point  -> element 2        |
   *    o----x----+       Pt = top point    -> element 3        o------> x
   *  BL    Pb     BR                                        coordinate space of the cell
   *
   * each point has two components: x and y.
   *
   * Need to interpolate the points along their respective edge as a function of the weights at
   * the vertices which define their edge, for example, we must interpolate the point Pl along 
   * the edge BL-TL, as a function of the weights at the points BL and TL.
   *
   * Note that not all components of each point need to change. For the points on a horizontal
   * edge (i.e. Pb and Pt) only the x component will change, and visa versa for the points on a 
   * vertical edge (i.e. Pl and Pr).
   *
   * The interpolation equations are as follows:
   *
   *    Pb.x = cell_size * (threshold - weight(BL) / (weight(BR) - weight(BL))
   *    Pt.x = cell_size * (threshold - weight(TL) / (weight(TR) - weight(TL))
   *    Pl.y = cell_size * (threshold - weight(BL) / (weight(TL) - weight(BL))
   *    Pr.y = cell_size * (threshold - weight(BR) / (weight(TR) - weight(BR))
   *
   * where:
   *    'weight(point)' is the function which returns the weight at the point.
   *
   * note - for the precomputed cell, we already have the weight at each point BL, BR, TR, and TL.
   *
   * note - the threshold must be the same as that passed to 'compute_cell' or else the results 
   *   are undefined.
   */
static void
lerp_cell(float threshold, struct cell_t *cell, struct cell_t *bottom, struct cell_t *left)
{
  int8_t index;

  for(int i = 0; i < 4; i++)
  {
    index = cell->indices[i];

    /* only interpolate the points that are in use */ 
    if(index == CELL_POINT_NULL)
      break;

    switch(index)
    {
    case CELL_POINT_L:
      /* any cell with a left point, will have a cell left of it with a coincident and already 
       * lerped right point; we can reuse that. The only exception is cells at col == 0 */
      if(left != NULL)
      {
        cell->points[index].y = left->points[CELL_POINT_R].y;
      }
      else
      {
        cell->points[index].y = lerp(threshold, 
                                     cell->samples[CELL_WEIGHT_BL].weight, 
                                     cell->samples[CELL_WEIGHT_TL].weight);
      }
      break;
    case CELL_POINT_B:
      /* any cell with a bottom point, will have a cell below it with a coincident and already 
       * lerped top point; we can reuse that. The only exception is cells at row == 0 */
      if(bottom != NULL)
      {
        cell->points[index].x = bottom->points[CELL_POINT_T].x;
      }
      else
      {
        cell->points[index].x = lerp(threshold, 
                                     cell->samples[CELL_WEIGHT_BL].weight, 
                                     cell->samples[CELL_WEIGHT_BR].weight);
      }
      break;
    case CELL_POINT_R:
      cell->points[index].y = lerp(threshold, 
                                   cell->samples[CELL_WEIGHT_BR].weight, 
                                   cell->samples[CELL_WEIGHT_TR].weight);
      break;
    case CELL_POINT_T:
      cell->points[index].x = lerp(threshold, 
                              cell->samples[CELL_WEIGHT_TL].weight, 
                              cell->samples[CELL_WEIGHT_TR].weight);
      break;
    }
  }
}

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
static struct globber_t globbers[GLOB_COUNT];

/*** GRID ****************************************************************************************/

/* A grid of sample points. The square area between every set of 4 adjacent samples is a cell. The 
 * grid defines the root coordinate space which the entire simulation is relative to. The grid
 * coordinate space is defined as follows:
 *
 *       y
 *       ^                           [ 6x6 grid of samples ]
 *       |
 *    r6 +---+---+---+---+---+---+         where:
 *       |   |   |   |   |   |   |            + = sample point
 *    r5 +---+---+---+---+---+---+         
 *       |   |   |   |   |   |   |            +---+
 *    r4 +---+---+---+---+---+---+            |   | = a cell of the grid
 *       |   |   |   |   |   |   |            +---+
 *    r3 +---+---+---+---+---+---+
 *       |   |   |   |   |   |   |         note that for a nxm sample grid there are 
 *    r2 +---+---+---+---+---+---+         n-1 columns of cells and m-1 rows of cells.
 *       |   |   |   |   |   |   |
 *    r1 +---+---+---+---+---+---+         cN = column N
 *       |   |   |   |   |   |   |         rN = row N
 *    r0 o---+---+---+---+---+---+---> x   
 *      c0  c1  c2  c3  c4  c5  c6         o = origin of the grid w.r.t opengl world space.
 */
struct sample_grid_t
{
  /* position of the grid origin w.r.t world space coordinates (unit: meters) */
  struct point2d_t pos_w_m;

  /* the grid samples, stored in column-major format, accessed [col][row] */
  struct sample_t samples[SAMPLE_GRID_COL_COUNT][SAMPLE_GRID_ROW_COUNT];
};

/* the simulation grid */
static struct sample_grid_t grid;

/* the current number of vertex components in the grid mesh */
static int isolines_mesh_component_count;

/* vertex buffer to store generated sample grid mesh */
static GLfloat isolines_mesh[ISOLINES_MESH_MAX_SIZE];

/* cell caches used to optimise cell processing (in function 'generate_isolines_mesh'). Avoids 
 * the naive approach of performing every linear interpolation twice, which results from processing
 * all cells independently. Note that we only need to cache two columns at a time; the currently
 * being processed column and the prior (left) column. This is because we process cells column per
 * column, from bottom (row 0) to top (row max), and each cell only needs data from the cell below
 * it or to the left of it to avoid duplicate lerps */
static struct cell_t cell_column_cache[2][SAMPLE_GRID_ROW_COUNT];

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
  static const float cutoff = 0.7f;
  static const float limit = 20.f;
  static const float inverse_limit = 1.f / limit;

  *r = *g = *b = 0.f;

  /* linear ramp of 2 colors with cut-off boundary */
  if(weight < cutoff)
    *r = *g = *b = SAMPLE_INACTIVE_GREY;

  else if(weight < limit)
  {
    *b = (weight * inverse_limit * (1.f - SAMPLE_INACTIVE_GREY)) + SAMPLE_INACTIVE_GREY;
    *r = ((1.f - weight) * inverse_limit * (1.f - SAMPLE_INACTIVE_GREY)) + SAMPLE_INACTIVE_GREY;
  }

  else
    *r = *g = *b = 1.f;
  
  //static const float inverse_limit = 1.f / 10.f;
  ////static const float half_pi = M_PI * 0.5f;
  //
  //float value = weight * inverse_limit;
  //if(value > 1.f)
  //  value = 1.f;

  // approximate exponential ramp of 2 colors
  ////*r = -cos(value * half_pi) + 1.f;
  ////*b = cos(value * half_pi);
  //
  // linear ramp of 2 colors
  //*r = value;
  //*b = 1.f - value;
  //if(weight > 1.f)
  //  printf("weight=%f, value=%f, color={r:%f, g:%f, b:%f}\n", weight, value, *r, *g, *b);
}

static void
draw_samples(void)
{
  glEnableClientState(GL_COLOR_ARRAY);
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

static void
rand_direction(struct vector2d_t *direction)
{
  static const int angle_resolution = 100;
  static const float angle_quantum_rad = (2 * M_PI) / (float)angle_resolution;

  float angle_rad = (rand() % angle_resolution) * angle_quantum_rad;

  direction->x = cos(angle_rad);
  direction->y = sin(angle_rad);
}

static void
rand_position_and_radius(struct point2d_t *pos_g_m, float *radius_m)
{
  static const int pos_resolution = 400;
  static const int radius_resolution = 100;
  static const float radius_quantum_m = (GLOB_MAX_RADIUS_M - GLOB_MIN_RADIUS_M) / (float)radius_resolution;

  float pos_x_quantum_g_m, pos_y_quantum_g_m;

  *radius_m = ((rand() % radius_resolution) * radius_quantum_m) + GLOB_MIN_RADIUS_M;

  pos_x_quantum_g_m = (sample_grid_width_m - (2 * (*radius_m))) / (float)pos_resolution;
  pos_y_quantum_g_m = (sample_grid_height_m - (2 * (*radius_m))) / (float)pos_resolution;

  pos_g_m->x = ((rand() % pos_resolution) * pos_x_quantum_g_m) + (*radius_m);
  pos_g_m->y = ((rand() % pos_resolution) * pos_y_quantum_g_m) + (*radius_m);
}

/* generates a random set of globbers to roam the simulation */
static void
generate_globs(void)
{
  srand(time(NULL));

  for(int i = 0; i < GLOB_COUNT; i++)
  {
    rand_direction(&(globbers[i].dir));
    rand_position_and_radius(&(globbers[i].center_g_m), &(globbers[i].radius_m));
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
}

/*** GRID ****************************************************************************************/

static void
init_grid(struct point2d_t grid_pos_w_m)
{
  grid.pos_w_m = grid_pos_w_m;

  /* zero all sample weights */
  memset((void *)grid.samples, 0, sizeof(struct sample_t) * SAMPLE_GRID_COL_COUNT * SAMPLE_GRID_ROW_COUNT);
}

static inline void
reset_isolines_mesh()
{
  isolines_mesh_component_count = 0;
}

/* generates a vertex mesh from the sample grid; uses marching cubes. The mesh will consist of
 * a set of disconnected lines. */
static void
generate_isolines_mesh(float threshold)
{
  struct point2d_t point;
  struct cell_t *current_cell, *bottom_cell, *left_cell;
  struct sample_t samples[4];
  struct cell_t *left_column_cache, *current_column_cache;
  bool cell_column_cache_id = 0; /* bool used to easily flip between 0 and 1 */

  left_column_cache = NULL;
  current_column_cache = cell_column_cache[(int)cell_column_cache_id];

  for(int col = 0; col < (SAMPLE_GRID_COL_COUNT - 1); col++)
  {
    for(int row = 0; row < (SAMPLE_GRID_ROW_COUNT - 1); row++)
    {
      samples[CELL_WEIGHT_BL].weight = grid.samples[col  ][row  ].weight;
      samples[CELL_WEIGHT_BR].weight = grid.samples[col+1][row  ].weight;
      samples[CELL_WEIGHT_TR].weight = grid.samples[col+1][row+1].weight;
      samples[CELL_WEIGHT_TL].weight = grid.samples[col  ][row+1].weight;

      current_cell = &current_column_cache[row];

      compute_cell(samples, threshold, current_cell); 

      bottom_cell = (row > 0) ? &current_column_cache[row - 1] : NULL;
      left_cell = (left_column_cache != NULL) ? &left_column_cache[row] : NULL;

      lerp_cell(threshold, current_cell, bottom_cell, left_cell);

      for(int i = 0; i < 4; i++)
      {
        if(current_cell->indices[i] == CELL_POINT_NULL)
        {
          /* ensure indicies come in pairs as intended */
          assert(i % 2 == 0); 

          /* no more indicies */
          break;
        }
        
        /* local cell space point */
        point = current_cell->points[current_cell->indices[i]];

        /* translate to grid space */
        point.x += col * CELL_SIZE_M;
        point.y += row * CELL_SIZE_M;

        /* add point to the mesh */
        isolines_mesh[isolines_mesh_component_count++] = point.x;
        isolines_mesh[isolines_mesh_component_count++] = point.y;

        /* inform the user if the mesh is too large; will need to increase the size limit define 
         * to handle the current simulation parameters */
        if(isolines_mesh_component_count > (ISOLINES_MESH_MAX_SIZE - 2))
        {
          fprintf(stderr, "fatal: the generated metaballs mesh is too large for the vertex buffer\n"
                          "info: increase the buffer size by changing the define:\n"
                          "                  ISOLINES_MESH_MAX_SIZE\n");
          printf("generated vertex component count: %d\n", isolines_mesh_component_count);
          exit(EXIT_FAILURE);
        }
      }
    }

    /* swap the caches so we will overrite the old left column with the next column of cells we
     * are due to process in the next loop iteration; only need to cache two columns */
    left_column_cache = current_column_cache;
    cell_column_cache_id = !cell_column_cache_id;
    current_column_cache = cell_column_cache[(int)cell_column_cache_id];
  }

  /* uncomment to check the mesh vertex buffer is large enough */
  //printf("generated vertex component count: %d\n", isolines_mesh_component_count);

  assert(isolines_mesh_component_count % 2 == 0);
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

static void
draw_isolines_mesh(void)
{
  glDisableClientState(GL_COLOR_ARRAY);
  glColor3f(ISOLINES_MESH_COLOR_R, ISOLINES_MESH_COLOR_G, ISOLINES_MESH_COLOR_B);
  glLineWidth(ISOLINES_MESH_DRAW_WIDTH_PX);
  glVertexPointer(2, GL_FLOAT, 0, isolines_mesh);
  glDrawArrays(GL_LINES, 0, isolines_mesh_component_count >> 1);
  glLineWidth(1.f);
}

/*** MODULE INTERFACE  ***************************************************************************/

void
init_metaballs(struct point2d_t grid_pos_w_m)
{
  init_grid(grid_pos_w_m);
  init_sample_gfx_data();
  generate_glob_mesh();
  generate_globs();
}

void
tick_metaballs(void)
{
  tick_globs();
  tick_grid();

  reset_isolines_mesh();

  for(int i = 0; i < THRESHOLD_COUNT; ++i)
    generate_isolines_mesh(thresholds[i]);
}

void
draw_metaballs(void)
{
  glPushMatrix();
  glTranslatef(grid.pos_w_m.x, grid.pos_w_m.y, 0.f);

  draw_samples();
  draw_isolines_mesh();
  draw_globs();

  glPopMatrix();
}

