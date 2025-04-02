#ifndef __FLIP_FLUID_H__
#define __FLIP_FLUID_H__

#include <stdint.h>
#include <math.h>
#include <string.h>

#define U_FIELD 0
#define V_FIELD 1

#define FLUID_CELL 0
#define AIR_CELL 1
#define SOLID_CELL 2

#define MAX_PARTICLES 1000
#define FLUID_NUM_X 18
#define FLUID_NUM_Y 18
#define FLUID_NUM_CELLS 324
#define FLUID_DENSITY 1000.0f
#define CELL_SPACING 55.5555555f
#define CELL_SPACING1 0.018f
#define CELL_SPACING2 27.77777777f
#define PARTICLE_NUMS 209
#define PARTICLE_RADIUS 18.75f
#define PARTICLE_SPACING1 0.0242424235f
#define PARTICLE_NUM_X 25
#define PARTICLE_NUM_Y 25
#define PARTICLE_NUM_CELLS 625
#define SQRT_2 1.41421356237f

typedef struct
{
	float gravity_x;
	float gravity_y;
	float dt;
	float flip_ratio;
	uint32_t num_pressure_iters;
	uint32_t num_particle_iters;
	float over_relaxation;
} Fluid_Setting;

typedef struct
{
	float density;
	float h;
	float h1;
	float h2;
	uint32_t fluid_num_x;
	uint32_t fluid_num_y;
	uint32_t fluid_num_cells;
	Fluid_Setting fluid_setting;

	float u[FLUID_NUM_CELLS];
	float v[FLUID_NUM_CELLS];
	float du[FLUID_NUM_CELLS];
	float dv[FLUID_NUM_CELLS];
	float prevU[FLUID_NUM_CELLS];
	float prevV[FLUID_NUM_CELLS];
	float p[FLUID_NUM_CELLS];
	float s[FLUID_NUM_CELLS];
	uint32_t cell_type[FLUID_NUM_CELLS];

	// Particle
	uint32_t particle_nums;
	uint32_t particle_num_x;
	uint32_t particle_num_y;
	uint32_t particle_num_cells;
	float particle_position_x[MAX_PARTICLES];
	float particle_position_y[MAX_PARTICLES];
	float particle_velocity_x[MAX_PARTICLES];
	float particle_velocity_y[MAX_PARTICLES];
	float particle_acceleration_x[MAX_PARTICLES];
	float particle_acceleration_y[MAX_PARTICLES];
	float particle_density[FLUID_NUM_CELLS];
	float particle_rest_density;
	float particle_radius;
	float particle_inv_spacing;

	float particle_in_each_cell[PARTICLE_NUM_CELLS];
	uint32_t particle_index_array[PARTICLE_NUM_CELLS + 1];
	uint32_t particle_dense_array[MAX_PARTICLES];
} FlipFluid;

void Fluid_Init(FlipFluid *fluid);
void Fluid_Integrate_Particles(FlipFluid *fluid, float dt, float gravity_x, float gravity_y);
void Fluid_Push_Particle_Apart(FlipFluid *fluid, uint32_t num_iters);
void Fluid_Handle_Collisions(FlipFluid *fluid);
void Fluid_Update_Density(FlipFluid *fluid);
void Fluid_Transfer_Velocities(FlipFluid *fluid, uint8_t to_grid, float flip_ratio);
void Fluid_Solve_Incompressibility(FlipFluid *fluid, uint32_t numIters, float dt, float over_relaxation);
void Fluid_Simulate(FlipFluid *fluid);
void Fluid_Create_Particles(FlipFluid *fluid, float num_x, float num_y);
void Fluid_Draw(FlipFluid *fluid, uint16_t *display_buffer);

#endif // __FLIP_FLUID_H__