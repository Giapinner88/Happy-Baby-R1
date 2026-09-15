# AI modules

Điểm đặt mã AI phụ trợ tách khỏi control loop: `vision/` cho perception và
`voice_interaction/` cho giao tiếp giọng nói. Không để module AI gửi low-level
command trực tiếp; đi qua interface/safety gate của `src/` và `hardware/`.
