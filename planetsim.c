#include <SDL2/SDL.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#define SCREEN_WIDTH 900
#define SCREEN_HEIGHT 600
#define OFFSET_COORD_X SCREEN_WIDTH / 2
#define OFFSET_COORD_Y SCREEN_HEIGHT / 2

#define COLOUR_WHITE 0xffffffff
#define COLOUR_BLACK 0x00000000
#define COLOUR_LIGHT 0xC7F0D8
#define COLOUR_DARK 0x43523D

#define DAMPENING 1.001
#define DELTA_T 0.1

#define TRAJ_LENGTH 300
#define TRAJ_WIDTH 2

typedef struct Body {
  double x, y, radius;
  double v_x, v_y;
  double mass;
} Body;

void FillBody(SDL_Surface *surface, struct Body body, Uint32 colour) {
  //
  double low_x = body.x - body.radius;
  double low_y = body.y - body.radius;
  double high_x = body.x + body.radius;
  double high_y = body.y + body.radius;
  double radsq = body.radius * body.radius;
  //
  for (double x = low_x; x < high_x; x++) {
    for (double y = low_y; y < high_y; y++) {
      double x_off = (x - body.x);
      double y_off = (y - body.y);
      double center_dist_sq = x_off * x_off + y_off * y_off;
      if (center_dist_sq < radsq) {
        SDL_Rect pixel = (SDL_Rect){x, y, 1, 1};
        SDL_FillRect(surface, &pixel, colour);
      }
    }
  }
}

void checkEdges(struct Body *body) {
   if (body->x + body->radius > SCREEN_WIDTH) {
    body->x = SCREEN_WIDTH - body->radius;
    body->v_x = -body->v_x * DAMPENING;
  }
  //
  if (body->y + body->radius > SCREEN_HEIGHT) {
    body->y = SCREEN_HEIGHT - body->radius;
    body->v_y = -body->v_y * DAMPENING;
  }
  //
  if (body->y - body->radius < 0) {
    body->y = body->radius;
    body->v_y = -body->v_y * DAMPENING;
  }
  //
  if (body->x - body->radius < 0) {
    body->x = body->radius;
    body->v_x = -body->v_x * DAMPENING;
  }
}

void step(struct Body *body, struct Body *body2) {
  //
  body->x += body->v_x * DELTA_T;
  body->y += body->v_y * DELTA_T;
  //
  body2->x += body2->v_x * DELTA_T;
  body2->y += body2->v_y * DELTA_T;
  //
  double distance = sqrt(pow(body->x - body2->x, 2) + pow(body->y - body2->y, 2));
  double nbx = (body2->x - body->x) / distance;
  double nby = (body2->y - body->y) / distance;
  double grav_f = 100 / pow(distance, 2);
  // gravity clamp
  if(distance <= (body->radius + body2->radius))
    grav_f = 100 / pow(body->radius + body2->radius, 2);
  double bx = grav_f * nbx;
  double by = grav_f * nby;
  //
  body->v_x += bx * body2->mass;
  body->v_y += by * body2->mass;
  body2->v_x += -bx * body->mass;
  body2->v_y += -by * body->mass;
  //
  checkEdges(body);
  checkEdges(body2);
 }

void FillTrajectory(SDL_Surface *surface,
                    struct Body trajectory[TRAJ_LENGTH]) {
  //
  for (int32_t i = 0; i < TRAJ_LENGTH; i++) {
    trajectory[i].radius = (double) TRAJ_WIDTH;
    FillBody(surface, trajectory[i], COLOUR_DARK);
  }
}

void UpdateTrajectory(struct Body trajectory[TRAJ_LENGTH],
                      struct Body body) {
  //
  // shift left by 1 and append latest circle to end
  struct Body traj_shifted[TRAJ_LENGTH];
  for (int32_t i = 1; i < TRAJ_LENGTH; i++)
    traj_shifted[i-1] = trajectory[i];
  for(int32_t i = 0;i<TRAJ_LENGTH;i++)
    trajectory[i] = traj_shifted[i];
  //
  trajectory[TRAJ_LENGTH-1] = body;
}

double random_double(double lower_bound, double upper_bound, long max_rand){
  return lower_bound + (upper_bound - lower_bound) * (rand() % max_rand) / max_rand;
}

int main() {
  //
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    printf("ERROR: SDL INIT: %s", SDL_GetError());
    exit(1);
  }
  //
  SDL_Window *window =
      SDL_CreateWindow("Bouncy Ball", SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
  if (!window) {
    printf("ERROR: %s", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_Surface *surface = SDL_GetWindowSurface(window);
  //
  SDL_Rect back = (SDL_Rect){0, 0, SCREEN_WIDTH, SCREEN_HEIGHT};
  SDL_Event event;
  //
  srand(time(NULL));
  const long max_rand = 1000000L;
  double lower_bound = -15;
  double upper_bound = 25;
  double random_double1 = random_double(lower_bound, upper_bound, max_rand);
  double random_double2 = random_double(lower_bound, upper_bound, max_rand);
  //
  struct Body body = (struct Body){OFFSET_COORD_X - 100, OFFSET_COORD_Y - 100, 18, random_double1, 25, 100};
  struct Body body2 = (struct Body) {OFFSET_COORD_X+100, OFFSET_COORD_Y + 100, 8,  random_double2, -25, 10};
  //
  struct Body trajectory[TRAJ_LENGTH] = {{0}};
  struct Body trajectory2[TRAJ_LENGTH] = {{0}};
  //
  int32_t run = 1;
  while (run) {
    //
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        run = 0;
      }
    }
    //
    SDL_FillRect(surface, &back, COLOUR_LIGHT);
    //
    FillBody(surface, body, COLOUR_DARK);
    FillBody(surface, body2, COLOUR_DARK);
    //
    step(&body, &body2);
    //
    FillTrajectory(surface, trajectory);
    FillTrajectory(surface, trajectory2);

    UpdateTrajectory(trajectory, body);
    UpdateTrajectory(trajectory2, body2);
    //
    SDL_UpdateWindowSurface(window);
    SDL_Delay(20);
  }
  //
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
