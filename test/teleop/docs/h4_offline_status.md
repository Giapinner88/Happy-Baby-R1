# Trạng thái tích hợp Quest teleop với policy H4 (offline)

Nhánh `feat/hb-quest-teleop` chỉ chuẩn bị giao thức và kiểm tra an toàn. **Chưa
có đường teleop H4 được nghiệm thu live trên robot.** `Application` có nhánh
phân quyền H4 và config hiện chọn đúng contract/SHA của policy đang chạy; cờ này
chỉ mở sau khi các preflight và gate mô phỏng đạt. START/đường legacy vẫn
được giữ làm rollback. Không đặt
`HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1` để thử trên robot từ nhánh này.

Đã có mapping đầu theo quy ước R1 của Happy-Baby-R1; sidecar nhận JSONL H4
chỉ khi chọn tường minh `--stream-contract h4`, cần `enabled` và
`upstream_sequence_id`, gửi UTL1 STOP lúc bắt đầu/nhả cò/timeout/kết thúc.
Byte `pad=1` chỉ đánh dấu stream có deadman tường minh; `pad=0` của stream cũ
không thể cấp quyền H4. Receiver C++ yêu cầu STOP sau khi vào một phiên mới,
và sau mất tín hiệu phải có STOP mới trước khi nhận target. Các chức năng này
được kiểm thử bằng fake state/socket; không phải kiểm thử motor.

Điều còn thiếu trước khi mở đường live:

1. Source `xr_teleoperate_v1_6` đã được xác định và ghim từ upstream chính
   thức; xem `config/upstream_vendor_contract.json`. Hai file chưa có trong
   checkout local nên chưa thể chạy FK/IK ở đây hoặc mở launcher vendor.
2. Adapter Quest → vendor IK → JSONL H4 có giới hạn URDF/vận tốc, kiểm chứng
   parity với trace nguồn và tín hiệu cò Quest thật. Không đổi nhãn luồng IK
   coupled hiện có thành vendor.
3. Kiểm thử tích hợp phân quyền trong `Application` với đúng artifact H4:
   history mask, thu tay/head, gesture, chuyển state và mô phỏng kín vòng.
   Unit test giao thức/hash/receiver và build không thay thế kiểm chứng này.
4. Gate phần cứng treo robot, E-stop và bằng chứng chiều khớp/target/encoder.

Kiểm tra offline hiện có:

```bash
PYTHONPATH=teleop/src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest teleop/tests -q
cmake --build controller/build --target run_r1 teleop_receiver_test -j2
ctest --test-dir controller/build --output-on-failure -R '^teleop_receiver_test$'
```

`flat_plus_profiles_preflight_test` vẫn cần các model H4/H5 fixture nằm trong
asset bị ignore; thiếu fixture không phải nghiệm thu policy thất bại. Không
thay thế test này bằng kết luận robot chạy an toàn.

`run_r1 --preflight` nạp được model H4 đang chọn trong worktree này, nhưng
các dance/motion asset bị ignore không có đủ ở worktree, nên một số bài nhảy
và getup/liedown được báo không khả dụng. Preflight không kiểm chứng chuyển
động hay điều khiển live.

## Pin IK upstream

GitHub ghi `v1.6` phát hành ngày 2026-07-29 và có hỗ trợ R1-A5. Repository
không có Git tag `v1.6`, vì vậy manifest ghim snapshot bất biến
`64ed45b4177e6297936940866df623b72621643a` thay vì một tag chuyển động.
Tại snapshot đó, `robot_arm_ik.py` khai báo `R1_A5_ArmIK` và trỏ tới
`assets/r1/r1_a5.urdf`. SHA-256 của hai file được ghi trong manifest.

Trong máy hiện tại, source và URDF vẫn chưa được checkout. Môi trường Conda
`tv` có `logging_mp` nhưng thiếu `pinocchio`, `casadi` và `meshcat`; do đó chưa
khởi tạo hay replay được solver. Giữ launcher vendor khóa, không gắn nhãn IK
coupled hiện tại thành `R1_A5_ArmIK`; phần H4/transport chỉ có bằng chứng
offline ở các test bên trên.
