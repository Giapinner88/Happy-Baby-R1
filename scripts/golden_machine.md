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

Để đồng bộ với hệ điều hành Ubuntu 22.04 LTS và tương thích với yêu cầu của R1, cấu hình Golden Machine được chốt như sau:

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
| **OS** | Ubuntu 22.04.x LTS (Jammy Jellyfish) | Phiên bản hệ điều hành tiêu chuẩn hỗ trợ `unitree_sdk2`. |

### 3.1. Máy tương đương trên Lightning AI (Cloud Workstation)

Để đảm bảo khả năng mô phỏng và huấn luyện tương đương khi làm việc trên Lightning AI, cần chọn máy ảo cloud có cấu hình tối thiểu như sau:

| Thành phần | Yêu cầu tối thiểu | Ghi chú |
| :--- | :--- | :--- |
| **GPU** | 2x NVIDIA GPU, mỗi GPU >= 24GB VRAM | Mục tiêu tương đương RTX 4500 Ada 24GB. |
| **CPU** | >= 16 vCPU | Ưu tiên CPU hiệu năng cao cho build và mô phỏng. |
| **RAM** | >= 64GB | Tránh nghẽn bộ nhớ khi chạy Isaac Lab + ROS2. |
| **Storage** | >= 1TB NVMe | Lưu dataset, checkpoint, rosbag2. |
| **OS Image** | Ubuntu 22.04 LTS | Đồng bộ môi trường với máy trạm vật lý. |

Checklist thiết lập trên Lightning AI:
1. Chọn image Ubuntu 22.04 LTS, bật CUDA 12.x tương thích GPU.
2. Cài ROS2 Humble Desktop Full và cấu hình CycloneDDS giống máy trạm.
3. Đồng bộ workspace bằng Git + LFS (nếu cần) và restore các dataset/model từ object storage.
4. Đặt biến môi trường và cấu hình driver theo chuẩn Golden Machine để tránh sai lệch Sim-to-Real.

### 3.2. Thiết lập Golden Machine trên Lightning AI (Team Workspace -> Hardware)

Mục tiêu là tạo một Team Workspace chuẩn và chọn phần cứng cloud mạnh nhất có sẵn để mô phỏng đúng năng lực Golden Machine.

Quy trình đề xuất:
1. Tạo Team Workspace mới với tên thống nhất theo dự án (ví dụ: `r1-golden-machine`).
2. Chọn template Team Workspace: "team" (phù hợp thiết lập dùng chung cho nhiều người).
3. Thiết lập quyền truy cập (Owner/Editor/Viewer) và bật audit log nếu có.
4. Tạo Project/Studio chính, cấu hình Git repo mặc định và bật Git LFS.
5. Chọn Studio template: "machine learning" (phù hợp pipeline huấn luyện + mô phỏng).
6. Chọn image nền Ubuntu 22.04 LTS, sau đó cấu hình CUDA 12.x và driver NVIDIA tương thích.
7. Chọn phần cứng (không giới hạn budget):
	- Model máy: ưu tiên loại máy đa GPU cao cấp (ví dụ: node H100 80GB x2 hoặc H100 80GB x4).
	- GPU: NVIDIA H100 80GB (SXM/PCIe), tối thiểu 2 GPU cho Isaac Lab và huan luyen song song.
	- CPU: AMD EPYC 9654 (96C/192T) hoac Intel Xeon Platinum 8480+ (56C/112T).
	- RAM: 256GB (toi thieu 128GB), uu tien DDR5 ECC.
	- Storage: NVMe 2TB (toi thieu 1TB) cho dataset, checkpoint, rosbag2.
8. Tạo snapshot/base image sau khi cài xong ROS2 Humble, Isaac Lab, MuJoCo, unitree_sdk2.
9. Thiết lập object storage và policy đồng bộ dataset/model/checkpoint theo chuẩn dự án.
10. Kiểm tra benchmark tối thiểu (build ROS2 + chạy Isaac Lab sample) để xác nhận hiệu năng.

## 4. Lý do lựa chọn (Justification & Rationale)

### 4.1. Khả năng đáp ứng Isaac Lab (NVIDIA Omniverse)
Động lực học của nền tảng 26 DOF trên R1 phức tạp hơn rất nhiều so với robot bốn chân truyền thống. **Isaac Lab** đòi hỏi GPU NVIDIA có dung lượng VRAM lớn và sức mạnh từ nhân Tensor để có thể xử lý các phép toán mô phỏng va chạm và render vật lý theo thời gian thực. Kiến trúc RTX 40-series đảm bảo quá trình đào tạo Reinforcement Learning diễn ra ổn định mà không gặp hiện tượng tràn bộ nhớ (Out-Of-Memory).

### 4.2. Hiệu năng xử lý Điều khiển & Mô phỏng
Các tác vụ cốt lõi như biên dịch mã C/C++ cho bộ điều khiển cấp thấp, chạy MuJoCo ở tần số cao để kiểm chứng động học, và xử lý luồng dữ liệu liên tục từ robot yêu cầu tài nguyên tính toán đơn nhân và đa nhân cực kỳ mạnh mẽ. Kiến trúc CPU thế hệ mới giúp giảm thiểu tối đa thời gian chờ đợi giữa các chu kỳ phát triển.

### 4.3. Kiến trúc mạng (Network Architecture)
Bo mạch chủ tích hợp Dual Ethernet là yêu cầu kỹ thuật bắt buộc. Một cổng sẽ được cấu hình Static IP để thiết lập kết nối trực tiếp đến Jetson Orin NX của R1 thông qua giao thức **CycloneDDS**. Thiết lập này giúp loại bỏ hoàn toàn độ trễ và nhiễu từ các thiết bị mạng nội bộ khác. Cổng thứ hai sẽ đảm nhiệm kết nối Internet phục vụ cho việc clone repository và quản lý dependencies.

## 5. Danh mục phần mềm đi kèm (Standard Software Stack)

Sau khi cài đặt hệ điều hành, Golden Machine cần được thiết lập tuần tự theo môi trường chuẩn:
1.  **Môi trường Python:** Thiết lập `pyenv` hoặc `virtualenv` với phiên bản **Python 3.10.12** làm môi trường mặc định (tương thích tối đa với Exudyn và SDK).
2.  **ROS2 Environment:** Cài đặt bản phân phối ROS2 Humble Desktop Full.
3.  **GPU Drivers:** Tích hợp NVIDIA Driver (Stable) và CUDA Toolkit 12.x.
4.  **Middleware:** Cấu hình QoS cho CycloneDDS thông qua file XML được chia sẻ nội bộ.