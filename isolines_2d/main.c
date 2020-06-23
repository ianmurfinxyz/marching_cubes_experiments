#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <GL/glu.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include "clock.h"
#include "isolines.h"

#define TICK_DELTA_S 0.0166666
#define MAX_TICKS_PER_FRAME 5

#define SCREEN_WIDTH_PX 1280
#define SCREEN_HEIGHT_PX 720

static GLfloat axis_vertices[] = {
   0.f  , 0.f  , 0.f  ,
   200.f, 0.f  , 0.f  ,   /* (+)x-axis */
   0.f  , 0.f  , 0.f  ,
  -200.f, 0.f  , 0.f  ,   /* (-)x-axis */
   0.f  , 0.f  , 0.f  ,
   0.f  , 200.f, 0.f  ,   /* (+)y-axis */
   0.f  , 0.f  , 0.f  ,
   0.f  ,-200.f, 0.f  ,   /* (-)y-axis */
}; /* size = 24 */

static GLfloat axis_colors[] = {
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,       /* (+)x-axis */
  1.0f, 0.5f, 0.0f,
  1.0f, 0.5f, 0.0f,       /* (-)x-axis */
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,       /* (+)y-axis */
  1.0f, 1.0f, 0.0f,
  1.0f, 1.0f, 0.0f,       /* (-)y-axis */
}; /* size = 8 */

static SDL_GLContext glcontext;
static SDL_Window *window;

static void
init()
{
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

  glEnableClientState(GL_VERTEX_ARRAY);

  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glEnable(GL_CULL_FACE);

  struct {
    float x;
    float y;
    float z;
    int y_move; /* -1=(-)axis movement, 0=no movement, +1=(+)axis movement */
    int x_move;
  } camera;
  camera.x = camera.y = 0.f;
  camera.z = 20.f;
  camera.y_move = camera.x_move = 0;
  float camera_delta_pos_m = 10.f * TICK_DELTA_S;

  init_isolines((struct point2d_t){1.f, 1.f});

  double next_tick_s = TICK_DELTA_S;
  bool redraw = true;
  bool is_done = false;
  int tick_count;
  clock_reset(&real_clock);
  while(!is_done)
  {
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
      switch(event.type)
      {
      case SDL_QUIT:
        is_done = true;
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
      case SDL_KEYDOWN:
        if(event.key.repeat != 0)
        {
          break;
        }
        else if(event.key.keysym.sym == SDLK_i)
        {
          camera.y_move = 1;
        }
        else if(event.key.keysym.sym == SDLK_k)
        {
          camera.y_move = -1;
        }
        else if(event.key.keysym.sym == SDLK_j)
        {
          camera.x_move = 1;
        }
        else if(event.key.keysym.sym == SDLK_l)
        {
          camera.x_move = -1;
        }
        break;
      case SDL_KEYUP:
        if(event.key.repeat != 0)
        {
          break;
        }
        else if(event.key.keysym.sym == SDLK_i)
        {
          camera.y_move = 0;
        }
        else if(event.key.keysym.sym == SDLK_k)
        {
          camera.y_move = 0;
        }
        else if(event.key.keysym.sym == SDLK_j)
        {
          camera.x_move = 0;
        }
        else if(event.key.keysym.sym == SDLK_l)
        {
          camera.x_move = 0;
        }
        break;
      }
    }

    double time_s = clock_time_s(&real_clock);
    tick_count = 0;
    while(time_s > next_tick_s && tick_count < MAX_TICKS_PER_FRAME)
    {
      camera.x += camera.x_move * camera_delta_pos_m;
      camera.y += camera.y_move * camera_delta_pos_m;

      tick_isolines();

      next_tick_s += TICK_DELTA_S;
      ++tick_count;
      redraw = true;
    }

    if(redraw)
    {
      glClear(GL_COLOR_BUFFER_BIT);

      /* set the view matrix */
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glTranslatef(camera.x, -camera.y, -camera.z);

      glEnableClientState(GL_COLOR_ARRAY);

      /* draw world space axes */
      glLineWidth(2.0);
      glVertexPointer(3, GL_FLOAT, 0, axis_vertices);
      glColorPointer(3, GL_FLOAT, 0, axis_colors);
      glDrawArrays(GL_LINES, 0, 8);
      glLineWidth(1.0);
      glDisableClientState(GL_COLOR_ARRAY);

      draw_isolines();

      SDL_GL_SwapWindow(window);
      redraw = false;
    }
  }
}

int 
main(int argc, char *argv[])
{
  init();
  run();
  exit(EXIT_SUCCESS);
}


