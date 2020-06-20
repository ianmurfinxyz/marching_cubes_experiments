#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <GL/glu.h>
#include <stdbool.h>
#include <assert.h>
#include "util/log.h"
#include "util/clock.h"
#include "spaceship.h"
#include "spaceship_camera.h"
#include "config.h"

#define SCREEN_WIDTH_PX 1500
#define SCREEN_HEIGHT_PX 800

static SDL_GLContext glcontext;
static SDL_Window *window;

/**** CUBE MODEL *************************************************************/

static GLfloat cube_vertices[] = {
  -1.0f, -1.0f,  1.0f,  
   1.0f, -1.0f,  1.0f,
   1.0f,  1.0f,  1.0f,
  -1.0f,  1.0f,  1.0f,
  -1.0f, -1.0f, -1.0f,  
   1.0f, -1.0f, -1.0f,
   1.0f,  1.0f, -1.0f,
  -1.0f,  1.0f, -1.0f
}; /* size = 12 */

static GLfloat cube_colors[] = {
  1.0f, 0.f, 0.f,
  1.0f, 0.f, 0.f,
  1.0f, 0.f, 0.f,
  1.0f, 0.f, 0.f,
  0.0f, 1.f, 0.f,
  0.0f, 1.f, 0.f,
  0.0f, 1.f, 0.f,
  0.0f, 1.f, 0.f
}; /* size = 12 */

static GLubyte cube_indices[] = {
  0, 1, 2,
  0, 2, 3,
  1, 5, 6,
  1, 6, 2,
  5, 4, 7,
  5, 7, 6, 
  4, 0, 3,
  4, 3, 7,
  3, 2, 6,
  3, 6, 7,
  1, 0, 4, 
  1, 4, 5
}; /* size == 36 */

/**** spaceship MODEL *************************************************************/

static GLfloat spaceship_vertices[] = {
  -0.5f,  0.0f, -1.0f,
  -1.0f,  0.0f,  1.0f,
   1.0f,  0.0f,  1.0f,
   0.5f,  0.0f, -1.0f,
  -0.75f, 0.3f,  0.5f,
   0.75f, 0.3f,  0.5f,
   0.75f,-0.3f,  0.5f,
  -0.75f,-0.3f,  0.5f
}; /* size = 8 */

static GLfloat spaceship_colors[] = {
  0.0f, 1.0f, 0.0f,
  1.0f, 1.0f, 0.0f,
  1.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 1.0f
}; /* size = 8 */

static GLubyte spaceship_indices[] = {
  0, 4, 1,
  4, 5, 1,
  1, 5, 2,
  2, 5, 3,
  0, 3, 4,
  4, 3, 5,
  0, 1, 7,
  1, 6, 7,
  1, 2, 6,
  0, 7, 3,
  3, 7, 6,
  2, 3, 6
}; /* size = 36 */

static GLubyte spaceship_bottom_indices[] = {
  0, 7, 1,
  1, 7, 6,
  1, 6, 2,
  0, 3, 7,
  3, 6, 7,
  2, 6, 3,
};

static GLubyte spaceship_top_indices[] = {
  0, 1, 4,
  4, 1, 5,
  1, 2, 5,
  2, 3, 5,
  0, 4, 3,
  4, 5, 3
};

static GLubyte spaceship_wireframe_indices[] = {
  0, 1,
  1, 2,
  2, 3,
  0, 3,
  0, 4,
  1, 4,
  2, 5,
  4, 5,
  3, 5,
  0, 7,
  1, 7,
  2, 6,
  6, 7,
  6, 3
};

/*** WORLD AXES **************************************************************/

static GLfloat axis_vertices[] = {
   0.f  , 0.f  , 0.f  ,
   200.f, 0.f  , 0.f  ,   /* (+)x-axis */
   0.f  , 0.f  , 0.f  ,
  -200.f, 0.f  , 0.f  ,   /* (-)x-axis */
   0.f  , 0.f  , 0.f  ,
   0.f  , 200.f, 0.f  ,   /* (+)y-axis */
   0.f  , 0.f  , 0.f  ,
   0.f  ,-200.f, 0.f  ,   /* (-)y-axis */
   0.f  , 0.f  , 0.f  ,
   0.f  , 0.f  , 200.f,   /* (+)z-axis */
   0.f  , 0.f  , 0.f  ,
   0.f  , 0.f  ,-200.f    /* (-)z-axis */
}; /* size = 36 */

static GLfloat axis_colors[] = {
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,       /* (+)x-axis */
  1.0f, 0.5f, 0.0f,
  1.0f, 0.5f, 0.0f,       /* (-)x-axis */
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,       /* (+)y-axis */
  1.0f, 1.0f, 0.0f,
  1.0f, 1.0f, 0.0f,       /* (-)y-axis */
  0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 1.0f,       /* (+)z-axis */
  0.0f, 1.0f, 1.0f,
  0.0f, 1.0f, 1.0f,       /* (-)z-axis */
}; /* size = 12 */

/*****************************************************************************/

static const int xz_grid_lines = 101; /* num lines per dimension; square grid */
static const int xz_grid_vertex_count = (xz_grid_lines * 2 * 2);
static const int xz_grid_vertex_component_count = xz_grid_vertex_count * 3;
static GLfloat xzgrid[1212]; 

/* generate_xz_grid - generates a vertex array which can be used to render
 * a grid in the world space x-z plane. The grid is centered about it's origin.
 *
 * @vertex_array - buffer to store generated vertices.
 * @ox,oy,oz - position vector to origin of the grid.
 * @d - distance between grid lines (unit: meters).
 * @nx - num lines perpendicular to x-axis in grid.
 * @nz - num lines perpendicular to z-axis in grid.
 *
 * note - the @vertex_array MUST be large enough to store all the generated
 * vertices or a seg fault will occur. The number of generated vertices can
 * be calculated as:
 *    
 *    num_vertices = (nx + ny) * 2 * 3
 *
 * why? nx + ny = num lines in grid, then have 2 points per line and 3 vertices
 * per point.
 *
 * note - nx and nz should be odd values to allow the grid to be evenly spaced
 * about the origin; the odd, extra line in each dimension is the center line
 * in that dimension. This approach is taken to allow the center line to be
 * rendered as the axes of the grid and have the grid evenly spaced about the
 * axes. If nx or nz is even, they are made odd by subtracting 1.
 */
void
generate_xz_grid(GLfloat *vertex_array, float d_m, int nx, int nz)
{
  assert(vertex_array != NULL);

  int nhx, nhz, v = 0;

  if(nx % 2 == 0)
    nx--;
  if(nz % 2 == 0)
    nz--;

  nhx = (nx - 1) / 2;
  nhz = (nz - 1) / 2;

  /* generate the lines perpendicular to the x-axis */
  for(int i = -nhx; i <= nhx; ++i)
  {
    vertex_array[v++] = (d_m * i);
    vertex_array[v++] = 0.f;
    vertex_array[v++] = (-nhz) * d_m;

    vertex_array[v++] = (d_m * i);
    vertex_array[v++] = 0.f;
    vertex_array[v++] = nhz * d_m;
  }

  /* generate the lines perpendicular to the z-axis */
  for(int i = -nhz; i <= nhz; ++i)
  {
    vertex_array[v++] = (-nhx) * d_m;
    vertex_array[v++] = 0.f;
    vertex_array[v++] = d_m * i;

    vertex_array[v++] = nhx * d_m;
    vertex_array[v++] = 0.f;
    vertex_array[v++] = (d_m * i);
  }
}

static void
init()
{
  log_init();

  GLenum glerror;

  if(SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "fatal: failed to init SDL2: SDL error: %s\n", SDL_GetError());
    exit(EXIT_SUCCESS);
  }

  window = SDL_CreateWindow("sdl2-opengl", 
                         SDL_WINDOWPOS_UNDEFINED,
                         SDL_WINDOWPOS_UNDEFINED,
                         SCREEN_WIDTH_PX,
                         SCREEN_HEIGHT_PX,
                         SDL_WINDOW_OPENGL);
  if(!window)
  {
    fprintf(stderr, "fatal: failed to create window: SDL error: %s\n", SDL_GetError());
    exit(EXIT_SUCCESS);
  }

  glcontext = SDL_GL_CreateContext(window);
  if(!glcontext)
  {
    fprintf(stderr, "fatal: failed to create opengl context: SDL error: %s\n", SDL_GetError());
    exit(EXIT_SUCCESS);
  }

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
  
  assert((!SDL_GL_SetSwapInterval(1) || !SDL_GL_SetSwapInterval(0)));

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if((glerror = glGetError()) != GL_NO_ERROR)
  {
    fprintf(stderr, "fatal: glMatrixMode: opengl error: %s\n", gluErrorString(glerror));
    exit(EXIT_SUCCESS);
  }

  gluPerspective(60.0f, 
                 (double)SCREEN_WIDTH_PX / (double)SCREEN_HEIGHT_PX,
                 1.0f,
                 1024.0f);

  glClearColor(0.f, 0.f, 0.f, 1.f);
  if((glerror = glGetError()) != GL_NO_ERROR)
  {
    fprintf(stderr, "fatal: glClearColor: opengl error: %s\n", gluErrorString(glerror));
    exit(EXIT_SUCCESS);
  }

  glViewport(0, 0, (GLsizei)SCREEN_WIDTH_PX, (GLsizei)SCREEN_HEIGHT_PX);
}

static void
run()
{
  struct clock real_clock;
  clock_init(&real_clock, CLOCK_MONOTONIC);
  log_write(LOG_INFO, 
            "real-time clock resolution = %ld nanoseconds\n", 
            clock_resolution_ns(&real_clock));


  generate_xz_grid(xzgrid, 10.f, xz_grid_lines, xz_grid_lines);

  glEnableClientState(GL_VERTEX_ARRAY);
      
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glEnable(GL_CULL_FACE);

  struct spaceship nautilus;
  spaceship_init(&nautilus,
            (struct vector4f){0.f, 0.f, 0.f, 1.f},
            (struct vector4f){0.f, 0.f, -1.f, 1.f},
            (struct vector4f){0.f, 1.f, 0.f, 0.f});

  struct spaceship_camera camera;
  shipcam_init(&camera, &nautilus);

  float angle_deg = 0.f;
  float angle_vel_degPs = 10.f;

  //int camera_yaw_dir = 0; /* -1 == -change, 0=no change, +1=positive change */
  //float camera_yaw_deg = 0.f;
  //float camera_yaw_vel_degPs = 20.f;

  //int camera_pitch_dir = 0; /* -1 == -change, 0=no change, +1=positive change */
  //float camera_pitch_deg = 0.f;
  //float camera_pitch_vel_degPs = 20.f;

  //int camera_roll_dir = 0; /* -1 == -change, 0=no change, +1=positive change */
  //float camera_roll_deg = 0.f;
  //float camera_roll_vel_degPs = 20.f;

  double next_tick_s = TICK_DELTA_S;
  bool redraw = true;
  bool is_done = false;
  int tick_count;
  clock_reset(&real_clock);
  while(!is_done)
  {
    /* handle window events */
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
      switch(event.type)
      {
      case SDL_QUIT:
        is_done = true;
      case SDL_KEYDOWN:
        if(event.key.repeat != 0)
        {
          break;
        }
        if(event.key.keysym.sym == SDLK_w)
        {
          spaceship_boost(&nautilus, BOOST_FORWARD);
        }
        else if(event.key.keysym.sym == SDLK_s)
        {
          spaceship_boost(&nautilus, BOOST_REVERSE);
        }
        else if(event.key.keysym.sym == SDLK_i)
        {
          spaceship_pitch(&nautilus, ROTATE_CCW);
        }
        else if(event.key.keysym.sym == SDLK_k)
        {
          spaceship_pitch(&nautilus, ROTATE_CW);
        }
        else if(event.key.keysym.sym == SDLK_j)
        {
          spaceship_roll(&nautilus, ROTATE_CW);
        }
        else if(event.key.keysym.sym == SDLK_l)
        {
          spaceship_roll(&nautilus, ROTATE_CCW);
        }

        //else if(event.key.keysym.sym == SDLK_l)
        //{
        //  camera_yaw_dir = 1; 
        //}
        //else if(event.key.keysym.sym == SDLK_j)
        //{
        //  camera_yaw_dir = -1; 
        //}

        //else if(event.key.keysym.sym == SDLK_i)
        //{
        //  camera_pitch_dir = 1; 
        //}
        //else if(event.key.keysym.sym == SDLK_k)
        //{
        //  camera_pitch_dir = -1; 
        //}

        //else if(event.key.keysym.sym == SDLK_o)
        //{
        //  camera_roll_dir = 1; 
        //}
        //else if(event.key.keysym.sym == SDLK_u)
        //{
        //  camera_roll_dir = -1; 
        //}

        break;
      case SDL_KEYUP:
        if(event.key.repeat != 0)
        {
          break;
        }
        if(event.key.keysym.sym == SDLK_w)
        {
          spaceship_boost(&nautilus, BOOST_NONE);
        }
        else if(event.key.keysym.sym == SDLK_s)
        {
          spaceship_boost(&nautilus, BOOST_NONE);
        }
        else if(event.key.keysym.sym == SDLK_i)
        {
          spaceship_pitch(&nautilus, ROTATE_NONE);
        }
        else if(event.key.keysym.sym == SDLK_k)
        {
          spaceship_pitch(&nautilus, ROTATE_NONE);
        }
        else if(event.key.keysym.sym == SDLK_j)
        {
          spaceship_roll(&nautilus, ROTATE_NONE);
        }
        else if(event.key.keysym.sym == SDLK_l)
        {
          spaceship_roll(&nautilus, ROTATE_NONE);
        }

        //else if(event.key.keysym.sym == SDLK_l)
        //{
        //  camera_yaw_dir = 0; 
        //}
        //else if(event.key.keysym.sym == SDLK_j)
        //{
        //  camera_yaw_dir = 0; 
        //}

        //else if(event.key.keysym.sym == SDLK_i)
        //{
        //  camera_pitch_dir = 0; 
        //}
        //else if(event.key.keysym.sym == SDLK_k)
        //{
        //  camera_pitch_dir = 0; 
        //}

        //else if(event.key.keysym.sym == SDLK_o)
        //{
        //  camera_roll_dir = 0; 
        //}
        //else if(event.key.keysym.sym == SDLK_u)
        //{
        //  camera_roll_dir = 0; 
        //}

        break;
      case SDL_WINDOWEVENT:
        switch(event.window.event)
        {
        case SDL_WINDOWEVENT_RESIZED:
          gluPerspective(60.0f, 
                         (double)event.window.data1 / (double)event.window.data2,
                         1.0f,
                         1024.0f);
          glViewport(0, 0, (double)event.window.data1, (double)event.window.data2);
          break;
        }
        break;
      }
    }

    double time_s = clock_time_s(&real_clock);
    tick_count = 0;
    while(time_s > next_tick_s && tick_count < MAX_TICKS_PER_FRAME)
    {
      angle_deg += angle_vel_degPs * TICK_DELTA_S;

      spaceship_tick(&nautilus);
      shipcam_tick(&camera);

      //camera_yaw_deg += cam_yaw_vel_degPs * cam_yaw_dir * TICK_DELTA_S;
      //camera_pitch_deg += cam_pitch_vel_degPs * cam_pitch_dir * TICK_DELTA_S;
      //camera_roll_deg += cam_roll_vel_degPs * cam_roll_dir * TICK_DELTA_S;

      next_tick_s += TICK_DELTA_S;
      ++tick_count;
      redraw = true;
    }

    if(redraw)
    {
      glClear(GL_COLOR_BUFFER_BIT);

      /* set cameraera position - i.e. view matrix */
      //glMatrixMode(GL_MODELVIEW);
      //glLoadIdentity();
      //glTranslatef(0.0f, 5.0f, -20.0f);
      //glRotatef(30.f, 1.0f, 0.0f, 0.0f);
      //glRotatef(angle_deg, 1.0f, 0.0f, 0.0f);
      
      //
      // the model appears to be inverted if I use my own view matrix, if
      // using the opengl matrix math to build the view it is not inverted.
      //
      // it is both the winding order and the spaceship direction that is inverted,
      // it is asthough the spaceship is scaled by -1.0 on the z-axis.
      //
      // With my view the spaceship points down -z but the winding order is inside
      // out, with opengl view the spaceship points up +z and the winding order is
      // correct.
      //
      // NOPE! i was wrong, in both cases the spaceship points up +z, but with my
      // view the winding order is inside out.
      //
      // NOPE! wrong again, they are actually the same. IT IS VERY HARD TO TELL!
      //
      // need a better testing setup/arrangement.
      //
      
      /* set the view matrix */
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      //glTranslatef(0.0f, 0.0f, -10.0f);
      //glRotatef(camera_yaw_deg, 0.0f, 1.0f, 0.0f);
      //glRotatef(camera_pitch_deg, 1.0f, 0.0f, 0.0f);
      //glRotatef(camera_roll_deg, 0.0f, 0.0f, 1.0f);
      //glRotatef(70.f, 0.0f, 1.0f, 0.0f);
      glLoadMatrixf(flatten44fm(&camera.wv));

      //glEnableClientState(GL_COLOR_ARRAY);
      
      glDisableClientState(GL_COLOR_ARRAY);

      /* bottom grid */
      glVertexPointer(3, GL_FLOAT, 0, xzgrid);
      glColor3f(0.5f, 0.5f, 0.5f);
      glPushMatrix();
      glTranslatef(0.0f, -505.f, 0.f);
      glDrawArrays(GL_LINES, 0, xz_grid_vertex_count);
      glPopMatrix();

      /* top grid */
      glPushMatrix();
      glTranslatef(0.0f, 505.f, 0.f);
      glDrawArrays(GL_LINES, 0, xz_grid_vertex_count);
      glPopMatrix();

      /* back grid (+)z */
      glPushMatrix();
      glTranslatef(0.f, 0.f, 505.f);
      glRotatef(90.f, 1.0f, 0.0f, 0.0f);
      glDrawArrays(GL_LINES, 0, xz_grid_vertex_count);
      glPopMatrix();

      /* front grid (-)z */
      glPushMatrix();
      glTranslatef(0.f, 0.f, -505.f);
      glRotatef(90.f, 1.0f, 0.0f, 0.0f);
      glDrawArrays(GL_LINES, 0, xz_grid_vertex_count);
      glPopMatrix();

      glEnableClientState(GL_COLOR_ARRAY);

      /* draw world space axes */
      glVertexPointer(3, GL_FLOAT, 0, axis_vertices);
      glColorPointer(3, GL_FLOAT, 0, axis_colors);
      glDrawArrays(GL_LINES, 0, 12);

      /* draw cube */
      //glPushMatrix();
      //glRotatef(angle_deg, 0.0f, 1.0f, 0.0f);
      //glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
      //glColorPointer(3, GL_FLOAT, 0, cube_colors);
      //glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, cube_indices);
      //glPopMatrix();

      /* draw spaceship */
      glPushMatrix();
      glMultMatrixf(flatten44fm(&nautilus.mw)); 
      //glTranslatef(0.f, 0.f, -50.f);
      //glVertexPointer(3, GL_FLOAT, 0, spaceship_vertices);
      //glColorPointer(3, GL_FLOAT, 0, spaceship_colors);
      //glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, spaceship_indices);
      glVertexPointer(3, GL_FLOAT, 0, spaceship_vertices);
      glDisableClientState(GL_COLOR_ARRAY);
      glPolygonMode(GL_FRONT, GL_FILL);
      glColor3f(1.f, 0.f, 0.5f);
      glDrawElements(GL_TRIANGLES, 18, GL_UNSIGNED_BYTE, spaceship_top_indices);
      glColor3f(0.f, 1.f, 1.f);
      glDrawElements(GL_TRIANGLES, 18, GL_UNSIGNED_BYTE, spaceship_bottom_indices);
      glPolygonMode(GL_FRONT, GL_LINE);
      glColor3f(1.f, 1.f, 1.0f);
      glDrawElements(GL_TRIANGLES, 18, GL_UNSIGNED_BYTE, spaceship_top_indices);
      glDrawElements(GL_TRIANGLES, 18, GL_UNSIGNED_BYTE, spaceship_bottom_indices);
      //glColor3f(1.f, 1.f, 1.f);
      //glDrawElements(GL_LINES, 28, GL_UNSIGNED_BYTE, spaceship_wireframe_indices);
      glPopMatrix();

      SDL_GL_SwapWindow(window);
      redraw = false;
    }
  }
}

static void
shutdown()
{
}

int 
main(int argc, char *argv[])
{
  init();
  run();
  shutdown();
  exit(EXIT_SUCCESS);
}


