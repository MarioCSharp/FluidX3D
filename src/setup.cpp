#include "setup.hpp"
void main_setup() {
    const float si_x = 2.0f;        
    const float si_u = 30.0f;       
    const float si_rho = 1.225f;    
    const float si_nu = 1.48E-5f;   

    const float lbm_u = 0.1f;      
    const float lbm_rho = 1.0f;     
    const float lbm_x = 256.0f;     

    units.set_m_kg_s(lbm_x, lbm_u, lbm_rho, si_x, si_u, si_rho);
    const float lbm_nu = units.nu(si_nu);

    const uint Nx = 512u;  
    const uint Ny = 256u;  
    const uint Nz = 128u;  
    LBM lbm(Nx, Ny, Nz, lbm_nu);

    const float3 center = float3(128.0f, lbm.center().y, lbm.center().z);
    const float3x3 rotation = float3x3(float3(0, 0, 1), radians(-90.0f)); 

    lbm.voxelize_stl(get_exe_path() + "../stl/mirror_step_model.stl", center, rotation, 64.0f, TYPE_S | TYPE_X);

    const uint Nx_lbm = lbm.get_Nx(), Ny_lbm = lbm.get_Ny(), Nz_lbm = lbm.get_Nz();
    parallel_for(lbm.get_N(), [&](ulong n) {
        uint x = 0u, y = 0u, z = 0u;
        lbm.coordinates(n, x, y, z);

        if (x == 0u && lbm.flags[n] != TYPE_S) {
            lbm.u.x[n] = lbm_u;
            lbm.u.y[n] = 0.0f;
            lbm.u.z[n] = 0.0f;
            lbm.flags[n] = TYPE_E; 
        }
        if (x == Nx_lbm - 1u && lbm.flags[n] != TYPE_S) {
            lbm.flags[n] = TYPE_E;
        }
        if (z == 0u && lbm.flags[n] != TYPE_S) {
            lbm.flags[n] = TYPE_S;
        }
        if ((y == 0u || y == Ny_lbm - 1u) && lbm.flags[n] != TYPE_S) {
            lbm.flags[n] = TYPE_S;
        }
        });

    lbm.run(0u); 

    for (uint i = 0; i < 50; i++) {
        lbm.run(100u); 

        const float3 lbm_force = lbm.object_force(TYPE_S | TYPE_X);

        const float3 si_force = float3(
            units.si_F(lbm_force.x),
            units.si_F(lbm_force.y),
            units.si_F(lbm_force.z)
        );

        print_info("Force X (drag): " + to_string(si_force.x) + " N");
        print_info("Force Y (side): " + to_string(si_force.y) + " N");
        print_info("Force Z (lift): " + to_string(si_force.z) + " N");

        const float frontal_area = 0.5f; 
        const float dynamic_pressure = 0.5f * si_rho * si_u * si_u;
        const float Cd = si_force.x / (dynamic_pressure * frontal_area);
        const float Cl = si_force.z / (dynamic_pressure * frontal_area);

        print_info("Drag coefficient Cd: " + to_string(Cd));
        print_info("Lift coefficient Cl: " + to_string(Cl));
        print_info("Downforce: " + to_string(-si_force.z) + " N");
    }
}