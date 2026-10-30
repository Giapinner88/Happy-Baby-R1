# Test lại bốn policy hoàn tất — đúng checkpoint14999

Tất cả4096env/15000iter khi train; đánh giá GPU, đúng PD/action của từng run. Không dùng action scale mới của M1/M2/M3. Hai seed eval20261024/7341,4rep/seed;95tình huống/policy + probe dáng đi.

| Policy | Đi: sống | XY error tiến/lùi m/s | Push: sống | Hồi phục event | Gesture hoàn thành | Gesture+push hồi phục |
|---|---:|---:|---:|---:|---:|---:|
| NoMirror · damping2.0 | 136/136 | 0.0823 | 463/464 | 517/528 | 160/160 | 96/96 |
| Mirror · damping2.0 | 136/136 | 0.0764 | 460/464 | 512/528 | 160/160 | 96/96 |
| NoMirror · damping2.5 | 136/136 | 0.0729 | 461/464 | 525/528 | 159/160 | 96/96 |
| Mirror · damping2.5 | 136/136 | 0.0693 | 463/464 | 524/528 | 159/160 | 96/96 |

## Giữ vị trí và push mạnh khi đi

| Policy | Gesture drift cuối max cm | Case tệ nhất | Lượt drift>5cm | Walk push1m/s ngã | Hồi phục | P90 hồi phục s |
|---|---:|---|---:|---:|---:|---:|
| NoMirror · damping2.0 | 9.17 | gesture_traitim | 2/64 | 1/32 | 29/32 | 2.4 |
| Mirror · damping2.0 | 46.87 | gesture_traitim | 8/64 | 4/32 | 25/32 | 2.112 |
| NoMirror · damping2.5 | 52.94 | gesture_traitim | 15/64 | 2/32 | 30/32 | 2.152 |
| Mirror · damping2.5 | 5.21 | gesture_traitim | 1/64 | 1/32 | 30/32 | 2.0340000000000003 |

## Hai chân

| Policy | Lệch bước .8m/s cm | Lệch bước1.2m/s cm | Lệch bước1.5m/s cm |
|---|---:|---:|---:|
| NoMirror · damping2.0 | 5.29 | 2.84 | 3.78 |
| Mirror · damping2.0 | 3.77 | 2.80 | 3.94 |
| NoMirror · damping2.5 | 4.49 | 3.87 | 4.37 |
| Mirror · damping2.5 | 3.33 | 5.28 | 5.24 |

## Quay tại chỗ và bề rộng chân khi đi nhanh

| Policy | Quay−.6: drift m | Quay+.6: drift m |1.5m/s bề rộng chân P05 cm |
|---|---:|---:|---:|
| NoMirror · damping2.0 | 0.045 | 0.103 | 10.76 |
| Mirror · damping2.0 | 0.070 | 0.082 | 10.13 |
| NoMirror · damping2.5 | 0.078 | 0.053 | 9.79 |
| Mirror · damping2.5 | 0.072 | 0.080 | 9.36 |

## Kết luận sau khi test đủ bốn checkpoint cuối

Ưu tiên **Mirror damping2.5** nếu chọn kết quả tổng thể trong bộ test: XY error thấp nhất .0693m/s, ngã khi đi+push mạnh1/32, nominal gesture drift max5.21cm và chỉ1/64lượt vượt5cm. Kết luận trước ưu tiên NoMirror2.0 dựa trên snapshot giữa chừng của damping2.5 đã được cập nhật.

Nếu bắt buộc damping2.0/NoMirror theo cấu hình người dùng đã chốt, ưu tiên **NoMirror damping2.0**; không tự thay M1/M2/M3 sang mirror hoặc damping2.5. NoMirror2.0 còn có bước hai chân đều hơn Mirror2.5 ở1.2/1.5m/s: lệch2.84/3.78cm so với5.28/5.24cm. Mirror loss không bảo đảm mọi thước đo dáng đi đối xứng hơn ở mọi tốc độ.

Hai bản2.5 đều sống160/160lượt gesture nhưng chỉ hoàn thành159/160: mỗi bản có một lần HB guard thu tay khi gesture kèm push. Mirror2.5: traitim_push_right, seed7341, rep1; NoMirror2.5: traitim_push_back, seed7341, rep2. Không gọi đây là ngã và không bỏ qua sự kiện thu tay.

Đây là lựa chọn thực dụng trong bộ test, chưa phải chứng minh ưu thế thống kê ngoài phân bố. Không so reward tổng và không kết luận damping là nguyên nhân duy nhất. Trước deploy cần giữ đúng PD/action contract của chính checkpoint.

## Giới hạn diễn giải

Hồi phục đo lần đầu giữ tiêu chí .5s trong cửa sổ6s, chỉ tính event sống hết cửa sổ; không đồng nghĩa đã về vị trí cũ hoặc hồi phục bền mãi tới cuối episode. Đọc cùng sống/ngã và drift. Push là xung vận tốc delta-v, không phải lực Newton. Gesture dùng ArmGesturePlayer thật của HB và q/dq mask trước history; không mô phỏng đủ communication/servo runtime. Hai seed đánh giá không thay thế nhiều seed training. Test native PD so sánh hệ policy+controller, không tách riêng nguyên nhân damping. Không có bài địa hình dốc/chướng ngại.

Chi tiết mọi case và raw trajectory nằm trong từng thư mục policy; snapshot manifest khóa checkpoint/ONNX hash.
