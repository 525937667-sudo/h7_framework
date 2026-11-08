#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "alg_sysid.h"

int main(int argc, char **argv)
{
  if (argc < 2) return 2;
  FILE *file = fopen(argv[1], "r");
  if (!file) return 2;
  char line[1024];
  unsigned first_time, second_time;
  if (!fgets(line, sizeof(line), file) || !fgets(line, sizeof(line), file) ||
      sscanf(line, "%u,", &first_time) != 1 || !fgets(line, sizeof(line), file) ||
      sscanf(line, "%u,", &second_time) != 1 || second_time <= first_time)
    return 2;
  unsigned interval = second_time - first_time;
  rewind(file);
  SysidConfig config = {.sampling_time = 0.004f, .window_time = 0.048f,
                        .current_scale = 0.3f, .omega_scale = 8.0f,
                        .position_harmonics = {10.0f, 20.0f, 36.0f}};
  config.sampling_time = 0.001f * (float)interval;
  if (argc > 2 && strcmp(argv[2], "--basic") == 0)
    memset(config.position_harmonics, 0, sizeof(config.position_harmonics));
  static SysidObject sysid;
  SysidModel model;
  if (!SysidInit(&sysid, &config)) return 2;
  (void)fgets(line, sizeof(line), file);
  unsigned previous_time = 0U, samples = 0U;
  while (fgets(line, sizeof(line), file))
  {
    unsigned time, count, age;
    float position, omega, current, command;
    if (sscanf(line, "%u,%u,%u,%f,%f,%f,%f", &time, &count, &age,
               &position, &omega, &current, &command) != 7) return 2;
    if (samples > 0U && (time <= previous_time || time - previous_time != interval))
    {
      fprintf(stderr, "Replay requires uniform sample intervals\n");
      fclose(file);
      return 2;
    }
    previous_time = time;
    samples++;
    (void)SysidUpdateWithPosition(&sysid, current, omega, position);
  }
  fclose(file);
  int valid = SysidGetModel(&sysid, &model);
  printf("valid=%d count=%u held_out=%u M=%.9g B=%.9g F=%.9g fit=%.6f\n",
          valid, model.count, sysid.validation_count, model.inertia,
          model.viscous, model.friction, model.fitness);
  return valid ? 0 : 1;
}
