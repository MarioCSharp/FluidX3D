#include "setup.hpp"
#include <fstream>

/*void main_setup() {
	std::ofstream csv("results.csv");
	csv << "t,Fx,Fy,Fz,Tx,Ty,Tz\n";
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 880u);
	const float lbm_Re = 20000.0f;
	const float lbm_u = 0.035f;
	const ulong lbm_T = 1000ull;

	units.set_m_kg_s(1.0f, 1.0f, 1.0f);
	LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));


	const float size = 1.0f * lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f * size, lbm.center().z);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(-15.0f));

	lbm.voxelize_stl(get_exe_path() + "stl/plane_model.stl", center, rotation, size, TYPE_S | TYPE_X);

	const float3 lbm_com = lbm.object_center_of_mass(TYPE_S | TYPE_X);

	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();
	parallel_for(lbm.get_N(), [&](ulong n) {
		uint x = 0u, y = 0u, z = 0u;
		lbm.coordinates(n, x, y, z);
		if (lbm.flags[n] != TYPE_S) lbm.u.x[n] = lbm_u; 
		if (x == 0u || x == Nx - 1u || y == 0u || y == Ny - 1u || z == 0u || z == Nz - 1u) lbm.flags[n] = TYPE_E;
		});

	lbm.run(0u, lbm_T);
	while (lbm.get_t() < lbm_T) {
		const float3 lbm_force = lbm.object_force(TYPE_S | TYPE_X);
		const float3 lbm_torque = lbm.object_torque(lbm_com, TYPE_S | TYPE_X);

		const float si_force_x = units.si_F(lbm_force.x);   // drag
		const float si_force_y = units.si_F(lbm_force.y);   // lift
		const float si_force_z = units.si_F(lbm_force.z);   // side force
		const float si_torque_x = units.si_T(lbm_torque.x);
		const float si_torque_y = units.si_T(lbm_torque.y);
		const float si_torque_z = units.si_T(lbm_torque.z);

		csv << lbm.get_t() << ","
			<< si_force_x << ","
			<< si_force_y << ","
			<< si_force_z << ","
			<< si_torque_x << ","
			<< si_torque_y << ","
			<< si_torque_z << "\n";

		// Print results to console
		std::cout << "t=" << lbm.get_t()
			<< " | Fx (drag force)=" << si_force_x
			<< " Fy (lift force)=" << si_force_y
			<< " Fz (side force)=" << si_force_z
			<< " | Tx (roll torque)=" << si_torque_x
			<< " Ty (pitch torque)=" << si_torque_y
			<< " Tz (yaw torque)=" << si_torque_z
			<< std::endl;

		lbm.run(1u, lbm_T);
	}

	csv.close();
}*/

void main_setup() {
    std::ofstream csv("results.csv");
    csv << "t,Fx,Fy,Fz,Tx,Ty,Tz\n";

    const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 880u);
    const float lbm_Re = 10000.0f;
    const float lbm_u = 0.025f;
    const ulong lbm_T = 750ull;

    LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));

    units.set_m_kg_s(1.0f, 1.0f, 1.0f);

    const float size = 1.0f * lbm.size().x;
    const float3 center = float3(lbm.center().x, 0.55f * size, lbm.center().z);
    const float3x3 rotation = float3x3(float3(1, 0, 0), radians(0.0f));

    lbm.voxelize_stl(get_exe_path() + "stl/car_step_model.stl", center, rotation, size, TYPE_S | TYPE_X);

    const float3 lbm_com = lbm.object_center_of_mass(TYPE_S | TYPE_X);

    const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();

    const uint floor_thickness = 1u;
    const uint floor_z_start = (uint)(0.25f * (float)Nz);
    const uint floor_z_end = floor_z_start + floor_thickness;

    parallel_for(lbm.get_N(), [&](ulong n) {
        uint x = 0u, y = 0u, z = 0u;
        lbm.coordinates(n, x, y, z);

        if (z >= floor_z_start && z <= floor_z_end && lbm.flags[n] != TYPE_S) {
            lbm.flags[n] = TYPE_S;
        }

        if (lbm.flags[n] != TYPE_S) lbm.u.x[n] = lbm_u;

        // Boundaries
        if (x == 0u || x == Nx - 1u || y == 0u || y == Ny - 1u || z == 0u || z == Nz - 1u) {
            lbm.flags[n] = TYPE_E;
        }
        });

    lbm.run(0u, lbm_T);
    while (lbm.get_t() < lbm_T) {
        const float3 lbm_force = lbm.object_force(TYPE_S | TYPE_X);
        const float3 lbm_torque = lbm.object_torque(lbm_com, TYPE_S | TYPE_X);

        const float si_force_x = units.si_F(lbm_force.x);   // Drag
        const float si_force_y = units.si_F(lbm_force.y);   // Lift
        const float si_force_z = units.si_F(lbm_force.z);   // Side force
        const float si_torque_x = units.si_T(lbm_torque.x);
        const float si_torque_y = units.si_T(lbm_torque.y);
        const float si_torque_z = units.si_T(lbm_torque.z);

        csv << lbm.get_t() << ","
            << si_force_x << ","
            << si_force_y << ","
            << si_force_z << ","
            << si_torque_x << ","
            << si_torque_y << ","
            << si_torque_z << "\n";

        std::cout << "t=" << lbm.get_t()
            << " | Fx (drag force)=" << si_force_x
            << " Fy (lift force)=" << si_force_y
            << " Fz (side force)=" << si_force_z
            << " | Tx (roll torque)=" << si_torque_x
            << " Ty (pitch torque)=" << si_torque_y
            << " Tz (yaw torque)=" << si_torque_z
            << std::endl;

        lbm.run(1u, lbm_T);
    }

    csv.close();
}