#include "setup.hpp"
#include <fstream>

void main_setup() {
    std::ofstream csv("results.csv");
    csv << "t,Fx,Fy,Fz,Tx,Ty,Tz\n";

    const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 1280u);
    const float lbm_Re = 1000000.0f;
    const float lbm_u = 0.075f;
    const ulong lbm_T = 10000ull;

    LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
    units.set_m_kg_s(1.0f, 1.0f, 1.0f);   

    const float size = 1.0f * lbm.size().x;
    const float3 center = float3(lbm.center().x, 0.55f * size, lbm.center().z);
    const float3x3 rotation = float3x3(float3(1, 0, 0), radians(0.0f));

    lbm.voxelize_stl(get_exe_path() + "../stl/car_step_model.stl", center, rotation, size, TYPE_S | TYPE_X);

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

        if (lbm.flags[n] != TYPE_S) lbm.u.y[n] = lbm_u;
        if (x == 0u || x == Nx - 1u || y == 0u || y == Ny - 1u || z == 0u || z == Nz - 1u)
            lbm.flags[n] = TYPE_E;
        });


    lbm.run(0u, lbm_T);
    while (lbm.get_t() < lbm_T) {

        const float3 lbm_force = lbm.object_force(TYPE_S | TYPE_X);
        const float3 lbm_torque = lbm.object_torque(lbm_com, TYPE_S | TYPE_X);

        const float Fx = units.si_F(lbm_force.x);
        const float Fy = units.si_F(lbm_force.y);
        const float Fz = units.si_F(lbm_force.z);
        const float Tx = units.si_T(lbm_torque.x);
        const float Ty = units.si_T(lbm_torque.y);
        const float Tz = units.si_T(lbm_torque.z);

        csv << lbm.get_t() << "," << Fx << "," << Fy << "," << Fz << ","
            << Tx << "," << Ty << "," << Tz << "\n";

        std::cout << "t=" << lbm.get_t()
            << " | Fx=" << Fx << " Fy=" << Fy << " Fz=" << Fz
            << " | Tx=" << Tx << " Ty=" << Ty << " Tz=" << Tz
            << std::endl;

        lbm.run(1u, lbm_T);
    }

    csv.close();
}

//
//void main_setup() { 
//    std::ofstream csv("results.csv");
//    csv << "t,Fx,Fy,Fz,Tx,Ty,Tz\n";
//
//    const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 1000u);
//    const float lbm_u = 0.075f;
//    const float lbm_length = 0.8f * (float)lbm_N.y;
//
//    const float si_T = 0.25f;
//    const float si_u = 100.0f / 3.6f;
//    const float si_length = 5.5f, si_width = 2.0f;
//    const float si_nu = 1.48e-5f, si_rho = 1.225f;
//    units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
//
//    const float lbm_nu = units.nu(si_nu);
//    const ulong lbm_T = units.t(si_T);
//    LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
//
//    Mesh* body = read_stl("stl/plane_model.stl");
//
//    const float scale = lbm_length / body->get_bounding_box_size().y;
//    body->scale(scale);
//
//    const float3 offset = float3(
//        lbm.center().x - body->get_bounding_box_center().x,
//        1.0f - body->pmin.y,         
//        4.0f - body->pmin.z
//    );
//    body->translate(offset);
//    body->set_center(body->get_bounding_box_center());
//
//    lbm.voxelize_mesh_on_device(body, TYPE_S | TYPE_X);
//
//    const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();
//    parallel_for(lbm.get_N(), [&](ulong n) {
//        uint x = 0u, y = 0u, z = 0u; lbm.coordinates(n, x, y, z);
//        if (lbm.flags[n] != TYPE_S) lbm.u.y[n] = lbm_u;                    
//        if (x == 0u || x == Nx - 1u || y == 0u || y == Ny - 1u || z == Nz - 1u) lbm.flags[n] = TYPE_E; 
//        if (z == 0u) lbm.flags[n] = TYPE_S;                                  
//        });
//
//    lbm.graphics.visualization_modes = VIS_FLAG_SURFACE | VIS_Q_CRITERION;
//
//#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
//    lbm.run(0u, lbm_T); // initialize
//    while (lbm.get_t() <= lbm_T) {
//        if (lbm.graphics.next_frame(lbm_T, 30.0f)) {
//            lbm.graphics.write_frame(get_exe_path() + "export/");
//        }
//
//        // advance in chunks
//        lbm.run(10u, lbm_T);
//
//        // forces/torques on car body only
//        const float3 com = lbm.object_center_of_mass(TYPE_S | TYPE_X);
//        const float3 f_lbm = lbm.object_force(TYPE_S | TYPE_X);
//        const float3 t_lbm = lbm.object_torque(com, TYPE_S | TYPE_X);
//
//        // to SI
//        csv << lbm.get_t() << ","
//            << units.si_F(f_lbm.x) << "," << units.si_F(f_lbm.y) << "," << units.si_F(f_lbm.z) << ","
//            << units.si_T(t_lbm.x) << "," << units.si_T(t_lbm.y) << "," << units.si_T(t_lbm.z) << "\n";
//    }
//#else
//    lbm.run(0u, lbm_T);
//    while (lbm.get_t() <= lbm_T) {
//        lbm.run(50u, lbm_T);
//
//        /*const float3 com = lbm.object_center_of_mass(TYPE_S | TYPE_X);
//        const float3 f_lbm = lbm.object_force(TYPE_S | TYPE_X);
//        const float3 t_lbm = lbm.object_torque(com, TYPE_S | TYPE_X);*/
//
//        /*csv << lbm.get_t() << ","
//            << units.si_F(f_lbm.x) << "," << units.si_F(f_lbm.y) << "," << units.si_F(f_lbm.z) << ","
//            << units.si_T(t_lbm.x) << "," << units.si_T(t_lbm.y) << "," << units.si_T(t_lbm.z) << "\n";*/
//    }
//#endif
//
//    csv.close();
//}
