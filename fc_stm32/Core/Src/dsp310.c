#include "dsp310.h"
#include "math.h"

// --- Định nghĩa thanh ghi nội bộ ---
#define PSR_CONFIG      0x06
#define TEMP_CONFIG     0x07
#define MEAS_CFG        0x08
#define CFG_REG         0x09
#define PROD_ID_REG     0x0D
#define COEF_REG_BASE   0x10
#define TMP_COEF_SRCE   0x28
#define RESULT_BASE     0x00

// Hệ số scale cho Oversampling 16x
static const uint32_t kP = 253952;
static const uint32_t kT = 253952;

// --- Biến phục vụ DMA (Quản lý trạng thái) ---
static uint8_t dsp310_dma_buf[6];                 // Buffer hứng 6 byte từ DMA
static volatile uint8_t dsp310_dma_busy = 0;  // Cờ báo DMA đang bận truyền nhận
static volatile uint8_t dsp310_data_ready = 0; // Cờ báo dữ liệu đã về RAM sẵn sàng xử lý
static DSP310_t *dsp310_dma_dev = NULL; // Con trỏ lưu thiết bị hiện tại phục vụ hàm ngắt

// --- Hàm Private (Giữ nguyên Polling cho quá trình Init ngắn) ---
static void DSP310_WriteReg(DSP310_t *dev, uint8_t reg, uint8_t value) {
	HAL_I2C_Mem_Write(dev->hi2c, DSP310_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 10);
}

static uint8_t DSP310_ReadReg(DSP310_t *dev, uint8_t reg) {
	uint8_t value = 0;
	HAL_I2C_Mem_Read(dev->hi2c, DSP310_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 10);
	return value;
}

static uint8_t DSP310_CalibCoefficient(DSP310_t *dev) {
	uint8_t buf[18] = { 0 };
	if (HAL_I2C_Mem_Read(dev->hi2c, DSP310_I2C_ADDR, COEF_REG_BASE,
	I2C_MEMADD_SIZE_8BIT, buf, 18, 50) != HAL_OK) {
		return 0;
	}
	dev->c0 = ((int32_t) buf[0] << 4) | (buf[1] >> 4);
	if (dev->c0 & (1 << 11))
		dev->c0 |= 0xFFFFF000;

	dev->c1 = ((int32_t) (buf[1] & 0x0F) << 8) | buf[2];
	if (dev->c1 & (1 << 11))
		dev->c1 |= 0xFFFFF000;

	dev->c00 = ((int32_t) buf[3] << 12) | ((int32_t) buf[4] << 4)
			| (buf[5] >> 4);
	if (dev->c00 & (1 << 19))
		dev->c00 |= 0xFFF00000;

	dev->c10 = ((int32_t) (buf[5] & 0x0F) << 16) | ((int32_t) buf[6] << 8)
			| buf[7];
	if (dev->c10 & (1 << 19))
		dev->c10 |= 0xFFF00000;

	dev->c01 = ((int32_t) buf[8] << 8) | buf[9];
	if (dev->c01 & (1 << 15))
		dev->c01 |= 0xFFFF0000;

	dev->c11 = ((int32_t) buf[10] << 8) | buf[11];
	if (dev->c11 & (1 << 15))
		dev->c11 |= 0xFFFF0000;

	dev->c20 = ((int32_t) buf[12] << 8) | buf[13];
	if (dev->c20 & (1 << 15))
		dev->c20 |= 0xFFFF0000;

	dev->c21 = ((int32_t) buf[14] << 8) | buf[15];
	if (dev->c21 & (1 << 15))
		dev->c21 |= 0xFFFF0000;

	dev->c30 = ((int32_t) buf[16] << 8) | buf[17];
	if (dev->c30 & (1 << 15))
		dev->c30 |= 0xFFFF0000;

	return 1;
}

// --- Hàm Public ---
uint8_t DSP310_Init(DSP310_t *dev, I2C_HandleTypeDef *hi2c) {
	dev->hi2c = hi2c;
	uint8_t status = 0;

	if (DSP310_ReadReg(dev, PROD_ID_REG) != 0x10) {
		return 0;
	}

	for (int i = 0; i < 20; i++) {
		HAL_Delay(10);
		status = DSP310_ReadReg(dev, MEAS_CFG);
		if ((status & 0xC0) == 0xC0)
			break;
	}

	if ((status & 0xC0) != 0xC0) {
		return 0;
	}

	if (!DSP310_CalibCoefficient(dev)) {
		return 0;
	}

	uint8_t tmp_coef_srce = DSP310_ReadReg(dev, TMP_COEF_SRCE) & 0x80;

	DSP310_WriteReg(dev, PSR_CONFIG, 0x44);
	DSP310_WriteReg(dev, TEMP_CONFIG, tmp_coef_srce | 0x44);
	DSP310_WriteReg(dev, CFG_REG, 0x03 << 2);
	DSP310_WriteReg(dev, MEAS_CFG, 0x07);

	// Đăng ký thiết bị toàn cục cho bộ lọc DMA
	dsp310_dma_dev = dev;
	dsp310_dma_busy = 0;
	dsp310_data_ready = 0;

	return 1;
}

/**
 * @brief  Hàm đọc không chặn (Non-blocking) sử dụng I2C DMA.
 *         Gọi hàm này liên tục trong vòng lặp main.
 * @retval 1: Vừa cập nhật dữ liệu mới thành công.
 *         0: Đang đợi phần cứng hoặc chưa có dữ liệu mới (Không gây treo).
 */
uint8_t DSP310_Read(DSP310_t *dev) {
	// Trường hợp 1: DMA vừa nhận xong dữ liệu, tiến hành tính toán (CPU xử lý nhanh)
	if (dsp310_data_ready) {
		// Xử lý dữ liệu từ buffer DMA toàn cục
		int32_t Praw = ((int32_t) dsp310_dma_buf[0] << 16)
				| ((int32_t) dsp310_dma_buf[1] << 8) | dsp310_dma_buf[2];
		if (Praw & 0x800000)
			Praw |= 0xFF000000;

		int32_t Traw = ((int32_t) dsp310_dma_buf[3] << 16)
				| ((int32_t) dsp310_dma_buf[4] << 8) | dsp310_dma_buf[5];
		if (Traw & 0x800000)
			Traw |= 0xFF000000;

		float Praw_sc = (float) Praw / kP;
		float Traw_sc = (float) Traw / kT;

		dev->temperature = (float) dev->c0 * 0.5f + (float) dev->c1 * Traw_sc;

		dev->pressure_Pa = (float) dev->c00
				+ Praw_sc
						* ((float) dev->c10
								+ Praw_sc
										* ((float) dev->c20
												+ Praw_sc * (float) dev->c30))
				+ Traw_sc * (float) dev->c01
				+ Traw_sc * Praw_sc
						* ((float) dev->c11 + Praw_sc * (float) dev->c21);

		dev->pressure_hPa = dev->pressure_Pa * 0.01f;
		dev->altitude =
				44330.0f
						* (1.0f
								- powf((dev->pressure_hPa / 1013.25f),
										(1.0f / 5.255f)));

		dsp310_data_ready = 0; // Xóa cờ sau khi xử lý xong
		return 1; // Trả về 1 báo hiệu có dữ liệu mới xuất hiện
	}

	// Trường hợp 2: Nếu DMA đang bận làm việc dưới nền, không kích hoạt lệnh mới
	if (dsp310_dma_busy) {
		return 0;
	}

	// Trường hợp 3: Thảnh thơi, kiểm tra thanh ghi xem cảm biến đã đo xong chưa
	uint8_t status = DSP310_ReadReg(dev, MEAS_CFG);
	if ((status & 0x30) == 0x30) {
		// Cảm biến sẵn sàng dữ liệu mới -> Kích hoạt DMA đọc ngầm
		dsp310_dma_busy = 1;
		dsp310_dma_dev = dev; // Đảm bảo gán đúng thiết bị

		if (HAL_I2C_Mem_Read_DMA(dev->hi2c, DSP310_I2C_ADDR, RESULT_BASE,
		I2C_MEMADD_SIZE_8BIT, (uint8_t*) dsp310_dma_buf, 6) != HAL_OK) {
			// Nếu kích hoạt lỗi (Ví dụ: Dây lỏng đột ngột làm HAL trả về HAL_BUSY/HAL_ERROR)
			dsp310_dma_busy = 0; // Reset cờ để vòng sau thử lại, hoàn toàn không bị block
		}
	}

	return 0; // Chưa có dữ liệu mới ở vòng này
}


void DSP310_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (dsp310_dma_dev != NULL && hi2c == dsp310_dma_dev->hi2c) {
		dsp310_dma_busy = 0;     // Giải phóng cờ bận
		dsp310_data_ready = 1;   // Báo chương trình chính vào tính toán dữ liệu
	}
}
void DSP310_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (dsp310_dma_dev != NULL && hi2c == dsp310_dma_dev->hi2c) {
		dsp310_dma_busy = 0;     // Tự động giải phóng cờ bận để tránh treo cứng
		dsp310_data_ready = 0;   // Hủy dữ liệu lỗi vòng này

		// Bạn có thể xử lý xóa lỗi phần cứng mềm tại đây nếu cần:
		// __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF | I2C_FLAG_BERR);
	}
}
/**
 * @brief  Hủy khởi tạo cảm biến, giải phóng tài nguyên DMA và xóa cờ.
 * @param  dev: Con trỏ quản lý thiết bị
 */
void DSP310_DeInit(DSP310_t *dev) {
	if (dev != NULL && dev->hi2c != NULL) {
		// 1. Dừng mọi quá trình truyền nhận DMA đang chạy trên I2C này (nếu có)
		HAL_I2C_Master_Abort_IT(dev->hi2c, DSP310_I2C_ADDR);

		// 2. Tắt chế độ Background Mode của cảm biến bằng cách đưa về Standby (0x00) nếu I2C còn phản hồi
		uint8_t standby_val = 0x00;
		HAL_I2C_Mem_Write(dev->hi2c, DSP310_I2C_ADDR, MEAS_CFG,
		I2C_MEMADD_SIZE_8BIT, &standby_val, 1, 5);
	}

	// 3. Reset các biến trạng thái nội bộ của thư viện về 0
	dsp310_dma_busy = 0;
	dsp310_data_ready = 0;
	dsp310_dma_dev = NULL;
}
