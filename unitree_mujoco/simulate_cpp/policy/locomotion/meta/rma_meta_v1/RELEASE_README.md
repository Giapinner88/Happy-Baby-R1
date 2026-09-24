# R1 RMA-meta release

Đây là release staging fail-closed. Cả `rma_meta` và `meta_policy` cũ đều đang tắt trong `params/deploy.rma_meta.yaml`.

Sau khi chuyển máy, kiểm tra bằng:

```bash
cd PATH_TO_RELEASE
python3 tools/check_rma_meta_release.py --release-dir .
```
