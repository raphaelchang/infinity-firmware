#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

void controller_init(void);
void controller_update(void);
void controller_apply_zsm(int *a, int *b, int *c);

#endif