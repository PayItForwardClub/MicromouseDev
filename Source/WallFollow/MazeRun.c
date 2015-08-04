/*
 * MazeRun.c
 *
 *  Created on: Jul 7, 2015
 *      Author: NHH
 */
#include "include.h"
#include "MazeRun.h"

/*
MAZE
x: 0->MAZE_WIDTH
y
0
|
MAZE_HEIGHT
*/

static void vMazeUpdateHorizontalWall(uint8_t x, uint8_t y, uint8_t wall);
static void vMazeUpdateVerticalWall(uint8_t x, uint8_t y, uint8_t wall);
static bool bMazeIsHasWall(uint8_t x, uint8_t y, uint8_t wall);

static uint32_t aui32_maze_vertical_wall[MAZE_HEIGHT];		//LEFT/RIGHT_WALL
static uint32_t aui32_maze_horizontal_wall[MAZE_WIDTH];		//FRONT/BEHIND_WALL

static uint8_t a2ui8_mazeWallInfo[MAZE_HEIGHT][MAZE_WIDTH];
static uint16_t a2ui16_mazeFloodfillInfo[MAZE_HEIGHT][MAZE_WIDTH];
static uint8_t ui8_Goal[2] = {0, 0};
static uint8_t ui8_CurrentPosition[2] = {0, 0};

void vMazeInit(void)
{
	int i;
	
	memset(&a2ui8_mazeWallInfo[0][0], 0, MAZE_HEIGHT*MAZE_WIDTH);
	
	for (i = 0; i < MAZE_WIDTH; i++)
	{
		a2ui8_mazeWallInfo[0][i] |= MAZE_WALL_FRONT;
		vMazeUpdateHorizontalWall(i, 0, MAZE_WALL_FRONT);
		a2ui8_mazeWallInfo[MAZE_HEIGHT-1][i] |= MAZE_WALL_BEHIND;
		vMazeUpdateHorizontalWall(i, MAZE_HEIGHT-1, MAZE_WALL_BEHIND);		
	}
	
	for (i = 0; i < MAZE_HEIGHT; i++)
	{
		a2ui8_mazeWallInfo[i][0] |= MAZE_WALL_LEFT;
		vMazeUpdateVerticalWall(0, i, MAZE_WALL_LEFT);
		a2ui8_mazeWallInfo[i][MAZE_WIDTH-1] |= MAZE_WALL_RIGHT;		
		vMazeUpdateVerticalWall(MAZE_WIDTH-1, i, MAZE_WALL_RIGHT);
	}
}

void vMazeUpdate(uint8_t x, uint8_t y, uint8_t wall, ENM_MAZE_DIRECTION Dir)
{
	wall &= 0x0f;
	
	switch (Dir)
	{
		case MAZE_DIRECTION_FORWARD:
			a2ui8_mazeWallInfo[y][x] |= (wall | MAZE_WALL_IS_UPDATED);
			vMazeUpdateHorizontalWall(x, y, wall);
			vMazeUpdateVerticalWall(x, y, wall);
			break;
		case MAZE_DIRECTION_BACKWARD:
			wall = (((wall >> 2) | ((wall & 0x03) << 2)));
			a2ui8_mazeWallInfo[y][x] = wall | MAZE_WALL_IS_UPDATED;
			vMazeUpdateHorizontalWall(x, y, wall);
			vMazeUpdateVerticalWall(x, y, wall);
			break;
		case MAZE_DIRECTION_LEFT:
			wall = (((wall >> 1) | ((wall & 0x01) << 3)));
			a2ui8_mazeWallInfo[y][x] = wall | MAZE_WALL_IS_UPDATED;
			vMazeUpdateHorizontalWall(x, y, wall);
			vMazeUpdateVerticalWall(x, y, wall);
			break;
		case MAZE_DIRECTION_RIGHT:
			wall = ((((wall & 0x07) << 1) | (wall >> 3)));
			a2ui8_mazeWallInfo[y][x] = wall | MAZE_WALL_IS_UPDATED;
			vMazeUpdateHorizontalWall(x, y, wall);
			vMazeUpdateVerticalWall(x, y, wall);
			break;
	}
}

void vMazeFloodFillUpdate(void)
{
	static int i, j, i1;
	static uint16_t count;
	memset(&a2ui16_mazeFloodfillInfo[0][0], 0xFF, MAZE_HEIGHT*MAZE_WIDTH*2);
	
	a2ui16_mazeFloodfillInfo[ui8_Goal[1]][ui8_Goal[0]] = 0;
	i = ui8_Goal[1];
	j = ui8_Goal[0];
	
	count = 1;

	while (count)
	{
		count = 0;
		for (i = 0; i < MAZE_HEIGHT; i++)
		{
			for (j = 0; j < MAZE_WIDTH; j++)
			{
				if ((i > 0) && (a2ui16_mazeFloodfillInfo[i][j] < a2ui16_mazeFloodfillInfo[i-1][j]) && (a2ui16_mazeFloodfillInfo[i - 1][j] == 0xFFFF))
				{
					if (!bMazeIsHasWall(j, i, MAZE_WALL_FRONT))
					{
						a2ui16_mazeFloodfillInfo[i-1][j] |= a2ui16_mazeFloodfillInfo[i][j] + 1;
						count++;
					}
				}
				if ((i < MAZE_HEIGHT - 1) && (a2ui16_mazeFloodfillInfo[i][j] < a2ui16_mazeFloodfillInfo[i+1][j]) && (a2ui16_mazeFloodfillInfo[i + 1][j] == 0xFFFF))
				{
					if (!bMazeIsHasWall(j, i, MAZE_WALL_BEHIND))
					{
						a2ui16_mazeFloodfillInfo[i+1][j] = a2ui16_mazeFloodfillInfo[i][j] + 1;
						count++;
					}
				}
				if ((j > 0) && (a2ui16_mazeFloodfillInfo[i][j] < a2ui16_mazeFloodfillInfo[i][j - 1]) && (a2ui16_mazeFloodfillInfo[i][j - 1] == 0xFFFF))
				{
					if (!bMazeIsHasWall(j, i, MAZE_WALL_LEFT))
					{
						a2ui16_mazeFloodfillInfo[i][j - 1] = a2ui16_mazeFloodfillInfo[i][j]+ 1;
						count++;
					}
				}
				if ((j < MAZE_WIDTH - 1) && (a2ui16_mazeFloodfillInfo[i][j] < a2ui16_mazeFloodfillInfo[i][j + 1]) && (a2ui16_mazeFloodfillInfo[i][j + 1] == 0xFFFF))
				{
					if (!bMazeIsHasWall(j, i, MAZE_WALL_RIGHT))
					{
						a2ui16_mazeFloodfillInfo[i][j + 1] = a2ui16_mazeFloodfillInfo[i][j] + 1;
						count++;
					}
				}
			}
		}
	}
}

void vMazeSetGoal(uint8_t x, uint8_t y)
{
	ui8_Goal[0] = x;
	ui8_Goal[1] = y;
}

void vMazeSetCurrentPosition(uint8_t x, uint8_t y)
{
	ui8_CurrentPosition[0] = x;
	ui8_CurrentPosition[1] = y;
}

static void vMazeUpdateHorizontalWall(uint8_t x, uint8_t y, uint8_t wall)
{
	if (wall & MAZE_WALL_FRONT)
		aui32_maze_vertical_wall[x] |= ((uint32_t)0x01) << y;
	if (wall & MAZE_WALL_BEHIND)
		aui32_maze_vertical_wall[x] |= ((uint32_t)0x02) << y;
}

static void vMazeUpdateVerticalWall(uint8_t x, uint8_t y, uint8_t wall)
{
	if (wall & MAZE_WALL_LEFT)
		aui32_maze_vertical_wall[y] |= ((uint32_t)0x01) << x;
	if (wall & MAZE_WALL_RIGHT)
		aui32_maze_vertical_wall[y] |= ((uint32_t)0x02) << x;
}

static bool bMazeIsHasWall(uint8_t x, uint8_t y, uint8_t wall)
{
	switch (wall)
	{
		case MAZE_WALL_FRONT:
			if ((aui32_maze_vertical_wall[x] & (((uint32_t)0x01) << y)) == 0)
				return false;
			else
				return true;
			break;
		case MAZE_WALL_BEHIND:
			if ((aui32_maze_vertical_wall[x] & (((uint32_t)0x02) << y)) == 0)
				return false;
			else
				return true;
			break;
		case MAZE_WALL_LEFT:
			if ((aui32_maze_vertical_wall[y] & ((uint32_t)0x01) << x) == 0)
				return false;
			else
				return true;
			break;
		case MAZE_WALL_RIGHT:
			if ((aui32_maze_vertical_wall[y] & ((uint32_t)0x02) << x) == 0)
				return false;
			else
				return true;
			break;
	}
	return false;
}
