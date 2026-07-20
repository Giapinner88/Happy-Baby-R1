# ROS 2 source workspace

`src/` là source root cho colcon. Các package R1 sở hữu (`r1_bringup`,
`r1_controllers`, `r1_description`, `r1_hardware_interface`, `r1_messages`)
là chỗ đặt integration code. Symlink `unitree_*` trỏ tới vendor và không sửa
trực tiếp. `build/`, `install/`, `log/` là output tái sinh của workspace này.
