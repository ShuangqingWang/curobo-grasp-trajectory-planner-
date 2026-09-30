# AUBO-i12H mesh provenance

These seven binary STL files were exported from AUBO's official i12/i12H STEP assembly:

- URL: `https://aubocdn.aubo-robotics.cn/official_website/101_Robot_Arm_I/i12/AUBO-i12-DRW_Robot_Machine_A0.STEP`
- STEP internal file name: `i12H.STEP`
- STEP SHA-256: `3003f06588fecdf72fed3a36412b5fd2b41a9809af53da43f2d21272834a8948`
- Imported: `2026-07-19`

The STEP assembly contains seven robot components. Each component was transformed from the assembled zero pose into its nominal i12H link frame using the controller's theoretical modified-DH chain, converted from millimetres to metres, and exported as binary STL. The URDF then applies this physical robot's calibrated joint origins from `model/aubo_i12h_kinematics.yaml`.

Production collision spheres are in `model/aubo_i12h_curobo.yml`.
