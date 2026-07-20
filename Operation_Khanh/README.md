# Operation_Khanh

Gói vận hành được triển khai từ máy phát triển sang máy tính nhúng của Unitree R1. Thư mục này tách biệt khỏi workflow train/simulation trên workstation.

- `high_level/`: runner C++ cho locomotion và motion imitation, policy/motion asset đã được chọn, cùng script build/deploy cho `~/HB/high_level_2` trên máy nhúng.
- `low_level/`: DDS bridge, GUI kiểm tra/tuning khớp và script deploy cho `~/HB/low_level` trên máy nhúng.

Không commit `build/`, cache, Python bytecode hoặc binary thử nghiệm cục bộ. Chỉ chạy deploy sau khi đã có bằng chứng simulation/dry-run phù hợp và tuân thủ quy trình an toàn của dự án.

Xem hướng dẫn cụ thể tại [high_level/README.md](high_level/README.md) và [low_level/docs/PC2_DEPLOY_GUIDE.md](low_level/docs/PC2_DEPLOY_GUIDE.md).
