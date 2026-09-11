# Baseline configuration

[`upstream_stream.json`](upstream_stream.json) là cấu hình active duy nhất. Nó
khóa model, frame contract và semantics của vendor `R1_A5_ArmIK` dùng cho cả
`make teleop` và hardware target producer.

Mỗi run chụp snapshot thành `experiment_config.json`; không diễn giải run cũ
bằng file editable hiện tại. Các profile coupled/differential/offline đã bị bỏ
khỏi active workspace và chỉ còn được mô tả trong `../history/` cùng Git history.
