# Giả lập chất lỏng trên STM32U585

Đây là một mạch in nhỏ do mình thiết kế với vi điều khiển STM32U585CIU6, chạy giả lập chất lỏng, hiển thị chất lỏng lên màn hình LED đơn; có thể tương tác với người dùng bằng cách nghiêng, lắc. Ý tưởng dựa trên project của mixela (YT: [Fluid Simulation Pedant](https://www.youtube.com/watch?v=jis1MC5Tm8k&t=2007s))

## Cài đặt

Sử dụng STM32CubeIDE import project dưới dạng CMake, hoặc sử dụng VSCode kết hợp STM32 VSCode Extension và Cmake Tools. Build với cấu hình Release. File elf sau khi build nằm ở build/Release/.

## Phần mềm

### Hiển thị

Màn hình gồm 216 LED, tạo thành từ ma trận LED 16x16, với 4 góc bị khuyết để tạo thành hình tròn. LED được điều khiển trực tiếp bằng chân GPIO của vi điều khiển. Ma trận LED được nối với kỹ thuật Charlieplexing:

Ưu điểm:

- Số lượng chân GPIO dùng để điều khiển màn hình LED là tối thiểu
- Tận dụng tốc độ bật tắt cao của chân GPIO vi điều khiển, tối đa 80MHz (nguồn :datasheet). Không cần dùng IC ngoài.
- Đơn giản hoá vẽ mạch in: Số lượng via sử dụng ít, cho phép đặt linh kiện ngay dưới màn hình LED. Bạn có thể xem phần giải thích ở video của mixela ().
- Độ sáng đồng nhất, do tại một thời điểm chỉ có một LED sáng.

Nhược điểm:

- Khó hiểu, khó sử dụng: ứng dụng tính chất 3 trạng thái của chân GPIO để điều khiển LED, dẫn đến khó sử dụng hơn những phương pháp điều khiển LED khác.
- Phần mềm phức tạp: Cần phải điều khiển GPIO với tốc độ rất cao, tiêu tốn nhiều tài nguyên CPU. Tại một thời điểm chỉ điều khiển được 1 LED sáng/tắt nên tốc độ quét trên LED phải rất cao.

### Điều khiển màn hình LED với STM32U585

- Có tất cả 216 LED. Do giới hạn của charlieplexing nên chỉ điều khiển sáng/tắt được 1 LED theo ý muốn. Trong một khoảng thời gian nhỏ (1ms), chia thành 216 phần thời gian bằng nhau, mỗi phần thời gian dành cho 1 LED. Do mắt lưu ảnh (nguồn -) nên mắt sẽ nhìn thấy toàn bộ bức ảnh do 216 LED tạo thành. Một khung ảnh (tạo thành từ 216 LED) cần 1ms để hiển thị. Vậy tốc độ làm mới (FPS) là 1000Hz.
- Điều khiển 1 LED sáng tắt bằng cách ghi vào hai thanh ghi ODR và MODER của GPIOA. ODR điều khiển mức điện áp (0/1). MODER điều khiển chế độ là ngõ ra hay ngõ vào (trở kháng cao).
- Mỗi LED sáng tương ứng với một giá trị của ODR và một giá trị MODER. Tất cả giá trị này được tính sẵn cho 216 LED và lưu vào một mảng. Khi cần sáng LED nào thì lấy giá trị tương ứng từ mảng này, lưu vào một mảng tạm. Led không sáng thì ODR = 0 và MODER = 0.
- Mảng tạm chứa tất cả các LED cần sáng để tạo thành ảnh. GPDMA thay thế cho CPU di chuyển dữ liệu từ mảng này sang hai thanh ghi GPIOA->ODR và GPIOA->MODER với tốc độ cao. Với mỗi LED GPDMA sử dụng hai kênh, một kênh để ghi vào ODR, một kênh ghi vào MODER. Hai kênh tuy độc lập lẫn nhau, như đều được trigger bởi một TIMER. Khi trigger xảy ra, cả hai thanh ghi ODR và MODER gần như được ghi cùng lúc, giúp giảm hiện tượng nhiễu màn hình. Kết hợp với chế độ circular mode và 2D addressing, GPDMA tự động chuyển sang LED tiếp theo cần sáng rồi chờ trigger từ timer; khi tới phần tử cuối trong mảng tạm, GPDMA tự động quay vòng địa chỉ về phần tử đầu tiên.
- TIMER15 được cấu hình đếm lên và reset mỗi 1/216 ms. Tín hiệu trigger gửi tới GPDMA điều khiển hoạt động của nó.
  
### Mô phỏng chất lỏng

- CPU của STM32U585 dành 100% thời gian để chạy mô phỏng chất lỏng. Phần mềm mô phỏng chất lỏng sử dụng phương pháp FLIP (Fluid Implicit Particle). Tham khảo tại [đây](https://www.sciencedirect.com/science/article/abs/pii/0010465588900203)
- Phần mềm được port sang C để tương thích với vi điều khiển. Phần mềm chưa hoàn toàn tối ưu. Nếu bạn muốn tự viết một cái, tham khảo kênh YT [Ten Minute Physics](https://www.youtube.com/@TenMinutePhysics/videos).
- MPU6050 là cảm biến dùng để đo hướng và độ lớn của gia tốc trọng trường. Giá trị này được truyền vào hàm mô phỏng chất lỏng. Giao tiếp I2C với MPU6050 ở chế độ non-block sử dụng DMA hoặc ngắt.
- CPU chạy ở tần số 160MHz. CPU chạy hàm mô phỏng mỗi 1/60 cho tốc độ cập nhật khung hình là 60 FPS.

### Chế độ tiết kiệm điện

- Khi không tương tác với người dùng (để yên một chỗ) lâu hơn 5s, vi điều khiển sẽ đi ngủ. Khi lắc mạnh sẽ tỉnh dậy. MPU6050 có lõi xử lý bên trong có khả năng gọi MCU tỉnh dậy khi phát hiện gia tốc lớn. Cái này là ý tưởng thôi chứ mình lười nên chưa làm.
- Mạch xài pin LIR2450 là loại pin đồng xu sạc lại được. Tuy nhiên nó có dung lượng nhỏ (khoảng 50mAh). Sử dụng liên tục trong khoảng 1h là hết pin.

## Phần cứng

- Mạch in hình tròn bán kính 1.5cm. Vừa đủ lớn để nhét vừa màn hình 16x16 LED (mỗi LED 0402).
- Do màn hình LED phải dùng rất nhiều Via nên đặt linh kiện ở mặt sau rất khó. May là charlieplexing có biến thể sử dụng ít via hơn, vừa đủ chỗ đặt con vi xử lý và mấy linh kiện bé.
- Mạch in 6 lớp. Dùng mạch in 4 lớp vẫn dư chỗ. Nhưng bên JLCPCB đang giảm giá với mình muốn cái mạch được mạ vàng nên dùng 6 lớp.
- Vi điều khiển STM32U585CIU6. Con này khá là mới (ra năm 2021). Mình dùng nó vì mình có sẵn. Nếu bạn muốn tự mình thiết kế lại, có thể thay con này bằng con nhỏ hơn, với điều kiện:
  - Có FPU. Giả lập chất lỏng làm toán chấm động là chính.
  - Có ít nhất 2 kênh DMA.
  - Điều khiển màn LED cần 16 chân GPIO. Chung 1 port càng tốt, vì chỉ cần điều khiển 2 thanh ghi của port đó để điều khiển cái màn hình LED (ODR và MODER). Nếu làm 2 port thì phải dùng 4 kênh DMA để điều khiển.
  - Tần số hoạt động > 100MHz. Cái này mình đoán dựa trên video của mixela và phần cứng của mình. CPU phải đủ mạnh mới mô phỏng được ở 60FPS. Không thì bạn có thể giảm FPS xuống và chấp nhận dùng chip yếu hơn.
  - Số chân GPIO: đủ cho 16 chân điều khiển màn hình, không nên quá nhiều chân vì nó sẽ to và không đặt lên mạch được.
- Vỏ: mình có vẽ một cái vỏ nữa, nhưng nó khá là xấu.