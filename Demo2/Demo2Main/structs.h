#ifndef STRUCTS_H
#define STRUCTS_H

struct stat_buffer {
  int size;
  int used;
  float buf[30];
};
typedef struct stat_buffer StatBuffer;

struct pid_tunables {
  float p;
  float i;
  float d;
};
typedef struct pid_tunables PIDTunables;

#endif