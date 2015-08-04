/*
 * MazeRun.h
 *
 *  Created on: Jul 7, 2015
 *      Author: NHH
 */

#ifndef MAZE_RUN_H_
#define MAZE_RUN_H_

#include <stdint.h>
#include <stdbool.h>

#define MAZE_WIDTH		16
#define MAZE_HEIGHT		16

#define MAZE_WALL_FRONT				(0x01<<0)
#define MAZE_WALL_RIGHT				(0x01<<1)
#define MAZE_WALL_BEHIND			(0x01<<2)
#define MAZE_WALL_LEFT				(0x01<<3)
#define MAZE_WALL_IS_UPDATED	(0x01<<7)

typedef enum
{
	MAZE_DIRECTION_FORWARD = 0,
	MAZE_DIRECTION_BACKWARD,
	MAZE_DIRECTION_LEFT,
	MAZE_DIRECTION_RIGHT
} ENM_MAZE_DIRECTION;

#endif /* MAZE_RUN_H_ */
