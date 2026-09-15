# Golden Machine Selection & Specification

**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-001
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Proposed

## 1. Mục đích (Purpose)

Tài liệu này xác định cấu hình phần cứng tiêu chuẩn ("Golden Machine") cho dự án nghiên cứu robot hình người Unitree R1. Việc chuẩn hóa hạ tầng phần cứng này nhằm:
* Đảm bảo tính tương thích và hiệu suất tuyệt đối với các engine mô phỏng nặng như **Isaac Lab** và **MuJoCo**.
* Giảm thiểu các biến số và sai số cấu hình trong quá trình chuyển giao thuật toán từ giả lập ra thực tế (**Sim-to-Real**).
* Tối ưu hóa thời gian biên dịch cho toàn bộ workspace ROS2 và các kiến trúc điều khiển phức tạp.

## 2. Thông số kỹ thuật nền tảng (Hardware Context)

Dự án hiện đang vận hành và nhắm mục tiêu phát triển trên phiên bản **Unitree R1 EDU U2**. Mẫu humanoid này được trang bị 26 bậc tự do (DOF), bao gồm cả hệ thống khớp cổ và đầu linh hoạt. Nền tảng tính toán tích hợp (onboard computer) cung cấp mức hiệu năng xấp xỉ 100 TOPS, cho phép xử lý trực tiếp các dự án AI và thị giác máy tính nâng cao ngay trên phần cứng của robot.

## 3. Thông số kỹ thuật máy trạm đề xuất (Technical Specifications)

Để đồng bộ với hệ điều hành đang sử dụng là Ubuntu 20.04 LTS và baseline ROS 2 Foxy, cấu hình Golden Machine được chốt như sau:

| Thành phần | Thông số kỹ thuật chi tiết | Ghi chú |
| :--- | :--- | :--- |
| **Model/Code** | 100-100001277WOF | Mã cấu hình CPU theo danh mục mua sắm. |
| **CPU** | AMD Ryzen 9 9950X | Nền tảng xử lý chính cho biên dịch và mô phỏng cường độ cao. |
| **GPU** | Dual NVIDIA RTX 4500 Ada, 24GB GDDR6, 4xDP | Cấu hình 2 GPU phục vụ Isaac Lab, AI pipeline và render song song. |
| **Mainboard** | ASROCK X670E TAICHI (tấm mạch in đã lắp ráp) | Bo mạch chủ nền tảng AM5 cao cấp cho tính ổn định lâu dài. |
| **RAM** | Corsair VENGEANCE RGB DDR5 64GB (2x32GB), 6000MHz, 1.35V, CMH64GX5M2D6000C40 | Dung lượng và băng thông phù hợp đa tác vụ mô phỏng + ROS2. |
| **Storage** | Kingston SSD NV3 1000GB, M.2 2280 NVMe (SNV3S/1000G) | Ổ hệ thống và dữ liệu tốc độ cao chuẩn NVMe. |
| **PSU** | Super Flower Leadex Platinum 2000W, 80 Plus Platinum (SF-2000F14HP) | Công suất lớn đáp ứng cấu hình 2 GPU chuyên dụng. |
| **Cooling** | Cooler Master MasterLiquid 360 Atmos ARGB | Tản nhiệt AIO 360mm cho CPU tải nặng kéo dài. |
| **Case** | Thermaltake View 600 TG Full Tower (không nguồn, không quạt) | Không gian lắp đặt lớn cho workstation đa GPU. |
| **OS** | Ubuntu 20.04.x LTS (Focal Fossa) | Baseline hiện tại của nhóm; dùng ROS 2 Foxy. |

### 3.1. Máy tương đương trên Lightning AI (Cloud Training Studio)

Lightning AI dùng cho huấn luyện/evaluation Isaac Lab, không dùng làm node vận hành robot trực tiếp. Môi trường local của robot vẫn giữ baseline Ubuntu 20.04 LTS + ROS 2 Foxy; môi trường Lightning tách riêng thành host Studio và Docker container Isaac Lab.

| Thành phần | Yêu cầu tối thiểu | Ghi chú |
| :--- | :--- | :--- |
| **GPU** | 1x NVIDIA L4 24GB cho smoke test; ưu tiên 2x GPU >= 24GB hoặc H100 cho train dài | L4 đã đủ để xác thực Isaac Lab headless và `rsl_rl` mẫu, nhưng không phải baseline tối đa. |
| **CPU** | >= 16 vCPU | Ưu tiên CPU hiệu năng cao cho build và mô phỏng. |
| **RAM** | >= 64GB | Tránh nghẽn bộ nhớ khi chạy Isaac Lab nhiều environment. |
| **Storage** | >= 1TB NVMe | Lưu dataset, checkpoint, log training và video debug. |
| **Host OS** | Lightning AI Studio Ubuntu 24.04 LTS | Host chỉ cung cấp Docker, GPU driver và storage. |
| **Training runtime** | Docker Isaac Lab/Isaac Sim, thường nền Ubuntu 22.04 LTS | Không cài Isaac Lab native trên host nếu mục tiêu là training ổn định. |

Checklist thiết lập trên Lightning AI:
1. Chọn Studio ở chế độ GPU trước khi dựng Docker; CPU-only Studio sẽ lỗi khi container cần NVML.
2. Kiểm tra `nvidia-smi` trên host và `docker run --gpus all ... nvidia-smi` trước khi train.
3. Dùng Docker Isaac Lab/NGC image đúng release thay vì trộn package native trên host.
4. Tắt X11 forwarding trên Studio headless; dùng `--headless`, EGL/Vulkan và video offline khi cần debug.
5. Đồng bộ workspace bằng Git + LFS nếu cần, nhưng lưu checkpoint/model về `data/models/...` hoặc object storage đã review.

### 3.2. Thiết lập Golden Machine trên Lightning AI (Team Workspace -> Hardware)

Mục tiêu là tạo một Team Workspace chuẩn và chọn phần cứng cloud mạnh nhất có sẵn để mô phỏng đúng năng lực Golden Machine.

Quy trình đề xuất:
1. Tạo Team Workspace mới với tên thống nhất theo dự án (ví dụ: `r1-golden-machine`).
2. Chọn template Team Workspace: "team" (phù hợp thiết lập dùng chung cho nhiều người).
3. Thiết lập quyền truy cập (Owner/Editor/Viewer) và bật audit log nếu có.
4. Tạo Project/Studio chính, cấu hình Git repo mặc định và bật Git LFS.
5. Chọn Studio template: "machine learning" (phù hợp pipeline huấn luyện + mô phỏng).
6. Chọn host Studio Ubuntu 24.04 LTS hoặc image Lightning tương thích, sau đó chạy Isaac Lab trong Docker container đúng release.
7. Chọn phần cứng (không giới hạn budget):
   - Model máy: ưu tiên loại máy đa GPU cao cấp (ví dụ: node H100 80GB x2 hoặc H100 80GB x4).
   - GPU: NVIDIA H100 80GB (SXM/PCIe), tối thiểu 2 GPU cho Isaac Lab và huấn luyện song song.
   - CPU: AMD EPYC 9654 (96C/192T) hoặc Intel Xeon Platinum 8480+ (56C/112T).
   - RAM: 256GB (tối thiểu 128GB), ưu tiên DDR5 ECC.
   - Storage: NVMe 2TB (tối thiểu 1TB) cho dataset, checkpoint, log training và video debug.
8. Tạo snapshot/base image sau khi Docker, NVIDIA Container Toolkit, NGC login và Isaac Lab container đã chạy smoke test.
9. Thiết lập object storage và policy đồng bộ dataset/model/checkpoint theo chuẩn dự án.
10. Kiểm tra benchmark tối thiểu (`nvidia-smi` trong container + Isaac Lab headless sample) để xác nhận hiệu năng.

## 4. Lý do lựa chọn (Justification & Rationale)

### 4.1. Khả năng đáp ứng Isaac Lab (NVIDIA Omniverse)
Động lực học của nền tảng 26 DOF trên R1 phức tạp hơn rất nhiều so với robot bốn chân truyền thống. **Isaac Lab** đòi hỏi GPU NVIDIA có dung lượng VRAM lớn và sức mạnh từ nhân Tensor để có thể xử lý các phép toán mô phỏng va chạm và render vật lý theo thời gian thực. Kiến trúc RTX 40-series đảm bảo quá trình đào tạo Reinforcement Learning diễn ra ổn định mà không gặp hiện tượng tràn bộ nhớ (Out-Of-Memory).

### 4.2. Hiệu năng xử lý Điều khiển & Mô phỏng
Các tác vụ cốt lõi như biên dịch mã C/C++ cho bộ điều khiển cấp thấp, chạy MuJoCo ở tần số cao để kiểm chứng động học, và xử lý luồng dữ liệu liên tục từ robot yêu cầu tài nguyên tính toán đơn nhân và đa nhân cực kỳ mạnh mẽ. Kiến trúc CPU thế hệ mới giúp giảm thiểu tối đa thời gian chờ đợi giữa các chu kỳ phát triển.

### 4.3. Kiến trúc mạng (Network Architecture)
Bo mạch chủ tích hợp Dual Ethernet là yêu cầu kỹ thuật bắt buộc. Một cổng sẽ được cấu hình Static IP để thiết lập kết nối trực tiếp đến Jetson Orin NX của R1 thông qua giao thức **CycloneDDS**. Thiết lập này giúp loại bỏ hoàn toàn độ trễ và nhiễu từ các thiết bị mạng nội bộ khác. Cổng thứ hai sẽ đảm nhiệm kết nối Internet phục vụ cho việc clone repository và quản lý dependencies.

## 5. Danh mục phần mềm đi kèm (Standard Software Stack)

Sau khi cài đặt hệ điều hành, Golden Machine cần được thiết lập tuần tự theo môi trường chuẩn:
1.  **Môi trường Python:** Dùng Python hệ thống 3.8 cho ROS 2 Foxy; dùng Conda env riêng cho AI/Simulation.
2.  **ROS 2 Environment:** Cài đặt ROS 2 Foxy Desktop Full trên Ubuntu 20.04.
3.  **GPU Drivers:** Tích hợp NVIDIA Driver ổn định và CUDA Toolkit tương thích với Ubuntu 20.04.
4.  **Middleware:** Cấu hình QoS cho CycloneDDS thông qua file XML được chia sẻ nội bộ.

## 6. Tài liệu liên quan

* Hướng dẫn cài Ubuntu: [../operations/ubuntu_20_04_lts_setup_guide.md](../operations/ubuntu_20_04_lts_setup_guide.md)
* Hướng dẫn môi trường dev: [../operations/development_environment_setup_guide.md](../operations/development_environment_setup_guide.md)
* Thiết lập mạng/DDS: [../operations/network_setup_checklist.md](../operations/network_setup_checklist.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)
