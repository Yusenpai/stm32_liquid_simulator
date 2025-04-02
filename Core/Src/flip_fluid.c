#include "flip_fluid.h"

static inline float clamp(float x, float a, float b)
{
	return x < a ? a : (x > b ? b : x);
}

static void Fluid_Setup(FlipFluid *fluid);

void Fluid_Init(FlipFluid *fluid)
{
	/* Fluid */
	fluid->density = FLUID_DENSITY;
	fluid->fluid_num_x = FLUID_NUM_X;
	fluid->fluid_num_y = FLUID_NUM_Y;
	fluid->h = CELL_SPACING;
	fluid->h1 = CELL_SPACING1;
	fluid->h2 = CELL_SPACING2;
	fluid->fluid_num_cells = FLUID_NUM_CELLS;

	memset(fluid->u, 0, sizeof(fluid->u));
	memset(fluid->v, 0, sizeof(fluid->v));
	memset(fluid->du, 0, sizeof(fluid->du));
	memset(fluid->dv, 0, sizeof(fluid->dv));
	memset(fluid->prevU, 0, sizeof(fluid->prevU));
	memset(fluid->prevV, 0, sizeof(fluid->prevV));
	memset(fluid->p, 0, sizeof(fluid->p));
	memset(fluid->s, 0, sizeof(fluid->s));
	memset(fluid->cell_type, FLUID_CELL, sizeof(fluid->cell_type));

	/* Particle */
	memset(fluid->particle_position_x, 0, sizeof(fluid->particle_position_x));
	memset(fluid->particle_position_y, 0, sizeof(fluid->particle_position_y));
	memset(fluid->particle_velocity_x, 0, sizeof(fluid->particle_velocity_x));
	memset(fluid->particle_velocity_y, 0, sizeof(fluid->particle_velocity_y));
	memset(fluid->particle_acceleration_x, 0, sizeof(fluid->particle_acceleration_x));
	memset(fluid->particle_acceleration_y, 0, sizeof(fluid->particle_acceleration_y));
	memset(fluid->particle_density, 0, sizeof(fluid->particle_density));
	memset(fluid->particle_in_each_cell, 0, sizeof(fluid->particle_in_each_cell));
	memset(fluid->particle_index_array, 0, sizeof(fluid->particle_index_array));
	memset(fluid->particle_dense_array, 0, sizeof(fluid->particle_dense_array));
	fluid->particle_rest_density = 0.0f;
	fluid->particle_radius = PARTICLE_RADIUS;
	fluid->particle_inv_spacing = PARTICLE_SPACING1;
	fluid->particle_num_x = PARTICLE_NUM_X;
	fluid->particle_num_y = PARTICLE_NUM_Y;
	fluid->particle_num_cells = PARTICLE_NUM_CELLS;
	fluid->particle_nums = 0;

	/* Fluid setting */
	fluid->fluid_setting.gravity_x = 0.0f;
	fluid->fluid_setting.gravity_y = -1000.0f;
	fluid->fluid_setting.dt = 1.0f / 60.0f;
	fluid->fluid_setting.dt *= 2.0f;
	fluid->fluid_setting.flip_ratio = 0.9f;
	fluid->fluid_setting.num_pressure_iters = 10;
	fluid->fluid_setting.num_particle_iters = 2;
	fluid->fluid_setting.over_relaxation = 1.9f;

	/* Create particle and setup solid cells */
	Fluid_Create_Particles(fluid, 25, 10);
	Fluid_Setup(fluid);
}

void Fluid_Integrate_Particles(FlipFluid *fluid, float dt, float gravity_x, float gravity_y)
{
	for (uint32_t i = 0; i < fluid->particle_nums; i++)
	{
		/* Use full Verlet velocity */
		fluid->particle_position_x[i] += fluid->particle_velocity_x[i] * dt + 0.5f * fluid->particle_acceleration_x[i] * dt * dt;
		fluid->particle_position_y[i] += fluid->particle_velocity_y[i] * dt + 0.5f * fluid->particle_acceleration_y[i] * dt * dt;
		fluid->particle_velocity_x[i] += 0.5f * (fluid->particle_acceleration_x[i] + gravity_x) * dt;
		fluid->particle_velocity_y[i] += 0.5f * (fluid->particle_acceleration_y[i] + gravity_y) * dt;
		fluid->particle_acceleration_x[i] = gravity_x;
		fluid->particle_acceleration_y[i] = gravity_y;
	}
}

void Fluid_Push_Particle_Apart(FlipFluid *fluid, uint32_t numIters)
{
	memset(fluid->particle_in_each_cell, 0, sizeof(fluid->particle_in_each_cell));

	/* Count particles in each cell */
	for (uint32_t i = 0; i < fluid->particle_nums; i++)
	{
		const float x = fluid->particle_position_x[i];
		const float y = fluid->particle_position_y[i];

		const uint32_t xi = (uint32_t)clamp(floor(x * fluid->particle_inv_spacing), 0, fluid->particle_num_x - 1);
		const uint32_t yi = (uint32_t)clamp(floor(y * fluid->particle_inv_spacing), 0, fluid->particle_num_y - 1);
		const uint32_t cellNr = xi * fluid->particle_num_y + yi;
		fluid->particle_in_each_cell[cellNr]++;
	}

	// Calculate partial sums
	uint32_t partial_sums = 0;
	for (uint32_t i = 0; i < fluid->particle_num_cells; i++)
	{
		partial_sums += fluid->particle_in_each_cell[i];
		fluid->particle_index_array[i] = partial_sums;
	}
	fluid->particle_index_array[fluid->particle_num_cells] = partial_sums; // guard

	// Fill Particle in cell
	for (uint32_t i = 0; i < fluid->particle_nums; i++)
	{
		const float x = fluid->particle_position_x[i];
		const float y = fluid->particle_position_y[i];

		const uint32_t xi = (uint32_t)clamp(floor(x * fluid->particle_inv_spacing), 0, fluid->particle_num_x - 1);
		const uint32_t yi = (uint32_t)clamp(floor(y * fluid->particle_inv_spacing), 0, fluid->particle_num_y - 1);
		const uint32_t cellNr = xi * fluid->particle_num_y + yi;
		fluid->particle_index_array[cellNr]--;
		fluid->particle_dense_array[fluid->particle_index_array[cellNr]] = i;
	}

	// Push particles apart
	const float minDist = 2.0f * fluid->particle_radius;
	const float minDist2 = minDist * minDist;
	for (uint32_t iter = 0; iter < numIters; iter++)
	{
		for (uint32_t i = 0; i < fluid->particle_nums; i++)
		{
			const float px = fluid->particle_position_x[i];
			const float py = fluid->particle_position_y[i];

			const uint32_t pxi = (uint32_t)floor(px * fluid->particle_inv_spacing);
			const uint32_t pyi = (uint32_t)floor(py * fluid->particle_inv_spacing);
			const uint32_t x0 = (uint32_t)fmax(pxi - 1, 0);
			const uint32_t y0 = (uint32_t)fmax(pyi - 1, 0);
			const uint32_t x1 = (uint32_t)fmin(pxi + 1, fluid->particle_num_x - 1);
			const uint32_t y1 = (uint32_t)fmin(pyi + 1, fluid->particle_num_y - 1);

			for (uint32_t xi = x0; xi <= x1; xi++)
			{
				for (uint32_t yi = y0; yi <= y1; yi++)
				{
					const uint32_t cellNr = xi * fluid->particle_num_y + yi;
					const uint32_t first = fluid->particle_index_array[cellNr];
					const uint32_t last = fluid->particle_index_array[cellNr + 1];
					for (uint32_t j = first; j < last; j++)
					{
						const uint32_t id = fluid->particle_dense_array[j];
						if (id == i)
							continue;
						const float qx = fluid->particle_position_x[id];
						const float qy = fluid->particle_position_y[id];

						float dx = qx - px;
						float dy = qy - py;
						const float d2 = dx * dx + dy * dy;
						if (d2 > minDist2 || d2 == 0.0f)
							continue;
						const float d = sqrt(d2);
						const float s = 0.5f * (minDist - d) / d;
						dx *= s;
						dy *= s;
						fluid->particle_position_x[i] -= dx;
						fluid->particle_position_y[i] -= dy;
						fluid->particle_position_x[id] += dx;
						fluid->particle_position_y[id] += dy;
					}
				}
			}
		}
	}
}

void Fluid_Handle_Collisions(FlipFluid *fluid)
{
	const float h = fluid->h;
	const float r = fluid->particle_radius;
	const float l1 = 6 * h + SQRT_2 * r;
	const float l2 = 30 * h - SQRT_2 * r;
	const float l3 = SQRT_2 * r - 12 * h;
	const float l4 = 12 * h - SQRT_2 * r;
	float min_x;
	float min_y;
	float max_x;
	float max_y;

	for (uint32_t k = 0; k < fluid->particle_nums; k++)
	{
		float x = fluid->particle_position_x[k];
		float y = fluid->particle_position_y[k];

		min_x = h + r;
		max_x = 17 * h - r;
		min_y = h + r;
		max_y = 17 * h - r;

		if (x < min_x)
		{
			x = min_x;
			fluid->particle_velocity_x[k] = 0.0f;
		}
		if (x > max_x)
		{
			x = max_x;
			fluid->particle_velocity_x[k] = 0.0f;
		}
		if (y < min_y)
		{
			y = min_y;
			fluid->particle_velocity_y[k] = 0.0f;
		}
		if (y > max_y)
		{
			y = max_y;
			fluid->particle_velocity_y[k] = 0.0f;
		}
		if (y < -x + l1)
		{
			float d = -x + l1 - y;
			float d2 = 0.5f * d;
			x += d2;
			y += d2;
			fluid->particle_velocity_y[k] = 0.0f;
		}
		if (y > -x + l2)
		{
			float d = y + x - l2;
			float d2 = 0.5f * d;
			x -= d2;
			y -= d2;
			fluid->particle_velocity_y[k] = 0.0f;
		}
		if (y < x + l3)
		{
			float d = x + l3 - y;
			float d2 = 0.5f * d;
			x -= d2;
			y += d2;
			fluid->particle_velocity_y[k] = 0.0f;
		}
		if (y > x + l4)
		{
			float d = y - x - l4;
			float d2 = 0.5f * d;
			x += d2;
			y -= d2;
			fluid->particle_velocity_y[k] = 0.0f;
		}
		fluid->particle_position_x[k] = x;
		fluid->particle_position_y[k] = y;
	}
}

void Fluid_Update_Density(FlipFluid *fluid)
{
	const uint32_t n = fluid->fluid_num_y;
	const float h = fluid->h;
	const float h1 = fluid->h1;
	const float h2 = fluid->h2;
	float *d = fluid->particle_density;
	memset(fluid->particle_density, 0, sizeof(fluid->particle_density));

	for (uint32_t i = 0; i < fluid->particle_nums; i++)
	{
		float x = fluid->particle_position_x[i];
		float y = fluid->particle_position_y[i];
		x = clamp(x, h, (fluid->fluid_num_x - 1) * h);
		y = clamp(y, h, (fluid->fluid_num_y - 1) * h);

		const uint32_t x0 = (uint32_t)floor((x - h2) * h1);
		const uint32_t x1 = fmin(x0 + 1, fluid->fluid_num_x - 2);
		const uint32_t y0 = (uint32_t)floor((y - h2) * h1);
		const uint32_t y1 = fmin(y0 + 1, fluid->fluid_num_y - 2);

		const float tx = ((x - h2) - x0 * h) * h1;
		const float ty = ((y - h2) - y0 * h) * h1;
		const float sx = 1.0f - tx;
		const float sy = 1.0f - ty;

		if (x0 < fluid->fluid_num_x && y0 < fluid->fluid_num_y)
			d[x0 * n + y0] += sx * sy;
		if (x1 < fluid->fluid_num_x && y0 < fluid->fluid_num_y)
			d[x1 * n + y0] += tx * sy;
		if (x1 < fluid->fluid_num_x && y1 < fluid->fluid_num_y)
			d[x1 * n + y1] += tx * ty;
		if (x0 < fluid->fluid_num_x && y1 < fluid->fluid_num_y)
			d[x0 * n + y1] += sx * ty;
	}

	if (fluid->particle_rest_density == 0.0f)
	{
		float sum = 0.0f;
		uint32_t numFluidCells = 0;
		for (uint32_t i = 0; i < fluid->fluid_num_cells; i++)
		{
			if (fluid->cell_type[i] == FLUID_CELL)
			{
				sum += d[i];
				numFluidCells++;
			}
		}
		if (numFluidCells > 0)
			fluid->particle_rest_density = sum / numFluidCells;
	}
}

void Fluid_Transfer_Velocities(FlipFluid *fluid, uint8_t toGrid, float flip_ratio)
{
	const uint32_t n = fluid->fluid_num_y;
	const float h = fluid->h;
	const float h1 = fluid->h1;
	const float h2 = fluid->h2;
	if (toGrid)
	{
		memcpy(fluid->prevU, fluid->u, sizeof(fluid->prevU));
		memcpy(fluid->prevV, fluid->v, sizeof(fluid->prevV));
		memset(fluid->du, 0, sizeof(fluid->du));
		memset(fluid->dv, 0, sizeof(fluid->dv));
		memset(fluid->u, 0, sizeof(fluid->u));
		memset(fluid->v, 0, sizeof(fluid->v));

		for (uint32_t i = 0; i < fluid->fluid_num_cells; i++)
			fluid->cell_type[i] = fluid->s[i] == 0.0f ? SOLID_CELL : AIR_CELL;

		for (uint32_t i = 0; i < fluid->particle_nums; i++)
		{
			const float x = fluid->particle_position_x[i];
			const float y = fluid->particle_position_y[i];
			const uint32_t xi = (uint32_t)clamp(floor(x * h1), 0, fluid->fluid_num_x - 1);
			const uint32_t yi = (uint32_t)clamp(floor(y * h1), 0, fluid->fluid_num_y - 1);
			const uint32_t cellNr = xi * n + yi;
			if (fluid->cell_type[cellNr] == AIR_CELL)
				fluid->cell_type[cellNr] = FLUID_CELL;
		}
	}

	for (uint32_t component = 0; component < 2; component++)
	{

		float dx = component == 0 ? 0.0f : h2;
		float dy = component == 0 ? h2 : 0.0f;

		float *f = component == 0 ? fluid->u : fluid->v;
		float *prevF = component == 0 ? fluid->prevU : fluid->prevV;
		float *d = component == 0 ? fluid->du : fluid->dv;
		float *vel_f = component == 0 ? fluid->particle_velocity_x : fluid->particle_velocity_y;

		for (uint32_t i = 0; i < fluid->particle_nums; i++)
		{
			float x = fluid->particle_position_x[i];
			float y = fluid->particle_position_y[i];

			x = clamp(x, h, (fluid->fluid_num_x - 1) * h);
			y = clamp(y, h, (fluid->fluid_num_y - 1) * h);

			const uint32_t x0 = fmin((uint32_t)floor((x - dx) * h1), fluid->fluid_num_x - 2);
			const uint32_t x1 = fmin(x0 + 1, fluid->fluid_num_x - 2);
			const uint32_t y0 = fmin((uint32_t)floor((y - dy) * h1), fluid->fluid_num_y - 2);
			const uint32_t y1 = fmin(y0 + 1, fluid->fluid_num_y - 2);

			const float tx = ((x - dx) - x0 * h) * h1;
			const float ty = ((y - dy) - y0 * h) * h1;
			const float sx = 1.0f - tx;
			const float sy = 1.0f - ty;

			const float d0 = sx * sy;
			const float d1 = tx * sy;
			const float d2 = tx * ty;
			const float d3 = sx * ty;

			const uint32_t nr0 = x0 * n + y0;
			const uint32_t nr1 = x1 * n + y0;
			const uint32_t nr2 = x1 * n + y1;
			const uint32_t nr3 = x0 * n + y1;

			if (toGrid)
			{
				const float pv = vel_f[i];
				f[nr0] += pv * d0;
				d[nr0] += d0;
				f[nr1] += pv * d1;
				d[nr1] += d1;
				f[nr2] += pv * d2;
				d[nr2] += d2;
				f[nr3] += pv * d3;
				d[nr3] += d3;
			}
			else
			{
				uint32_t offset = component == 0 ? n : 1;
				const float valid0 = fluid->cell_type[nr0] != AIR_CELL || fluid->cell_type[nr0 - offset] != AIR_CELL ? 1.0f : 0.0f;
				const float valid1 = fluid->cell_type[nr1] != AIR_CELL || fluid->cell_type[nr1 - offset] != AIR_CELL ? 1.0f : 0.0f;
				const float valid2 = fluid->cell_type[nr2] != AIR_CELL || fluid->cell_type[nr2 - offset] != AIR_CELL ? 1.0f : 0.0f;
				const float valid3 = fluid->cell_type[nr3] != AIR_CELL || fluid->cell_type[nr3 - offset] != AIR_CELL ? 1.0f : 0.0f;

				const float v = vel_f[i];
				const float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;

				if (d > 0.0f)
				{
					const float picV = (valid0 * d0 * f[nr0] + valid1 * d1 * f[nr1] + valid2 * d2 * f[nr2] + valid3 * d3 * f[nr3]) / d;
					const float corr = (valid0 * d0 * (f[nr0] - prevF[nr0]) + valid1 * d1 * (f[nr1] - prevF[nr1]) + valid2 * d2 * (f[nr2] - prevF[nr2]) + valid3 * d3 * (f[nr3] - prevF[nr3])) / d;
					const float flipV = v + corr;
					vel_f[i] = (1.0f - flip_ratio) * picV + flip_ratio * flipV;
				}
			}
		}

		if (toGrid)
		{
			for (uint32_t i = 0; i < sizeof(fluid->u); i++)
			{
				if (d[i] > 0.0f)
					f[i] /= d[i];
			}

			// restore solid cells

			for (uint32_t i = 0; i < fluid->fluid_num_x; i++)
			{
				for (uint32_t j = 0; j < fluid->fluid_num_y; j++)
				{
					const uint32_t solid = fluid->cell_type[i * n + j] == SOLID_CELL;
					if (solid || (i > 0 && fluid->cell_type[(i - 1) * n + j] == SOLID_CELL))
						fluid->u[i * n + j] = fluid->prevU[i * n + j];
					if (solid || (j > 0 && fluid->cell_type[i * n + j - 1] == SOLID_CELL))
						fluid->v[i * n + j] = fluid->prevV[i * n + j];
				}
			}
		}
	}
}

void Fluid_Solve_Incompressibility(FlipFluid *fluid, uint32_t numIters, float dt, float overRelaxation)
{
	memset(fluid->p, 0, sizeof(fluid->p));
	memcpy(fluid->prevU, fluid->u, sizeof(fluid->prevU));
	memcpy(fluid->prevV, fluid->v, sizeof(fluid->prevV));

	const uint32_t n = fluid->fluid_num_y;
	const float cp = fluid->density * fluid->h / dt;
	for (uint32_t iter = 0; iter < numIters; iter++)
	{
		for (uint32_t i = 1; i < fluid->fluid_num_x - 1; i++)
		{
			for (uint32_t j = 1; j < fluid->fluid_num_y - 1; j++)
			{
				if (fluid->cell_type[i * n + j] != FLUID_CELL)
					continue;

				const uint32_t center = i * n + j;
				const uint32_t left = (i - 1) * n + j;
				const uint32_t right = (i + 1) * n + j;
				const uint32_t bottom = i * n + j - 1;
				const uint32_t top = i * n + j + 1;

				const float sx0 = fluid->s[left];
				const float sx1 = fluid->s[right];
				const float sy0 = fluid->s[bottom];
				const float sy1 = fluid->s[top];
				const float s = sx0 + sx1 + sy0 + sy1;
				if (s == 0.0f)
					continue;

				float div = fluid->u[right] - fluid->u[center] +
							fluid->v[top] - fluid->v[center];

				if (fluid->particle_rest_density > 0.0f)
				{
					const float k = 1.0f;
					const float compression = fluid->particle_density[i * n + j] - fluid->particle_rest_density;
					if (compression > 0.0f)
						div = div - k * compression;
				}

				const float p = -overRelaxation * div / s;
				fluid->p[center] += cp * p;

				fluid->u[center] -= sx0 * p;
				fluid->u[right] += sx1 * p;
				fluid->v[center] -= sy0 * p;
				fluid->v[top] += sy1 * p;
			}
		}
	}
}

void Fluid_Simulate(FlipFluid *fluid)
{
	float dt = fluid->fluid_setting.dt;
	float gravity_x = fluid->fluid_setting.gravity_x;
	float gravity_y = fluid->fluid_setting.gravity_y;
	float flip_ratio = fluid->fluid_setting.flip_ratio;
	float over_relaxation = fluid->fluid_setting.over_relaxation;
	uint32_t num_pressure_iters = fluid->fluid_setting.num_pressure_iters;
	uint32_t num_particle_iters = fluid->fluid_setting.num_particle_iters;
	uint32_t time_step = 1;
	float sub_dt = dt / time_step;

	for (uint32_t step = 0; step < time_step; step++)
	{
		Fluid_Integrate_Particles(fluid, sub_dt, gravity_x, gravity_y);
		Fluid_Push_Particle_Apart(fluid, num_particle_iters);
		Fluid_Handle_Collisions(fluid);
		Fluid_Transfer_Velocities(fluid, 1, flip_ratio);
		Fluid_Update_Density(fluid);
		Fluid_Solve_Incompressibility(fluid, num_pressure_iters, sub_dt, over_relaxation);
		Fluid_Transfer_Velocities(fluid, 0, flip_ratio);
	}
}

void Fluid_Create_Particles(FlipFluid *fluid, float num_x, float num_y)
{
	const uint32_t res = 17;
	const float tankHeight = 1000.0f;
	const float h = tankHeight / res;
	const float r = 0.3f * h;
	const float dx = 2.0f * r;
	const float dy = sqrt(3.0f) / 2.0f * dx;
	uint32_t numParticles = num_x * num_y;
	fluid->particle_nums = numParticles;
	for (uint32_t i = 0; i < num_x; i++)
	{
		for (uint32_t j = 0; j < num_y; j++)
		{
			uint32_t index = i * num_y + j;
			fluid->particle_position_x[index] = h + r + dx * i + (j % 2 == 0 ? 0.0f : r);
			fluid->particle_position_y[index] = h + r + dy * j;
		}
	}
}

void Fluid_Draw(FlipFluid *fluid, uint16_t *display_buffer)
{
	for (uint32_t i = 0; i < 16; i++)
	{
		for (uint32_t j = 0; j < 16; j++)
		{
			uint32_t index = (i + 1) * fluid->fluid_num_y + j + 1;
			switch (fluid->cell_type[index])
			{
			case FLUID_CELL:
				display_buffer[j] |= (1 << i);
				break;
			case AIR_CELL:
				display_buffer[j] &= ~(1 << i);
				break;
			case SOLID_CELL:
				display_buffer[j] &= ~(1 << i);
				break;
			default:
				break;
			}
		}
	}
}

static void Fluid_Setup(FlipFluid *fluid)
{
	// setup grid cells
	uint32_t n = fluid->fluid_num_y;
	for (uint32_t i = 0; i < fluid->fluid_num_x; i++)
	{
		for (uint32_t j = 0; j < fluid->fluid_num_y; j++)
		{
			float s = 1.0f; // fluid
			if (i == 0 || i == fluid->fluid_num_x - 1 || j == 0 || j == fluid->fluid_num_y - 1)
			{
				s = 0.0f; // solid
			}
			if (i + j <= 5 || (i >= 13 && j <= 4 && i - j >= 12))
			{
				s = 0.0f;
			}
			else if ((i >= 13 && j >= 13 && i + j >= 29) || (i <= 4 && j >= 13 && j - i >= 12))
			{
				s = 0.0f;
			}
			fluid->s[i * n + j] = s;
		}
	}
}
